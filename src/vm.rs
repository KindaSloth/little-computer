use std::{
    fs::File,
    io::{self, BufReader, ErrorKind, Read, Write},
    process,
};

use byteorder::{BigEndian, ReadBytesExt};

#[allow(non_snake_case)]
pub struct Registers {
    pub R_R0: u16,   /* general purpose register */
    pub R_R1: u16,   /* general purpose register */
    pub R_R2: u16,   /* general purpose register */
    pub R_R3: u16,   /* general purpose register */
    pub R_R4: u16,   /* general purpose register */
    pub R_R5: u16,   /* general purpose register */
    pub R_R6: u16,   /* general purpose register */
    pub R_R7: u16,   /* general purpose register */
    pub R_PC: u16,   /* program counter */
    pub R_COND: u16, /* register to store information about the previous calculation */
}

#[allow(non_camel_case_types)]
pub enum ConditionFlags {
    FL_POS,
    FL_ZRO,
    FL_NEG,
}

impl ConditionFlags {
    pub fn get_flag_value(self) -> u16 {
        match self {
            ConditionFlags::FL_POS => 1 << 0, // Positive
            ConditionFlags::FL_ZRO => 1 << 1, // Zero
            ConditionFlags::FL_NEG => 1 << 2, // Negative
        }
    }
}

/*
TRAP Codes

The LC-3 provides a few predefined routines for performing common tasks and interacting with I/O devices,
these are called trap routines which you can think of as the operating system or API for the LC-3.

Each trap routine is assigned a trap code which identifies it (similar to an opcode).
To execute one, the TRAP instruction is called with the trap code of the desired routine.

In the official LC-3, trap routines are written in assembly, here I'll write them using Rust functions.

When a trap code is called, the PC is moved to that code’s address. The CPU executes the procedure’s instructions, and
when it is complete, the PC is reset to the location following the initial call.

This is why programs start at address 0x3000 instead of 0x0. The lower addresses are left empty to leave space for the trap routine code.
*/
pub enum TrapCode {
    /// get character from keyboard
    Getc = 0x20,
    /// output a character
    Out = 0x21,
    /// output a word string
    Puts = 0x22,
    /// input a string
    In = 0x23,
    /// output a byte string
    Putsp = 0x24,
    /// halt the program
    Halt = 0x25,
}

const PC_START: u16 = 0x3000;

impl Registers {
    pub fn new() -> Self {
        Self {
            R_R0: 0,
            R_R1: 0,
            R_R2: 0,
            R_R3: 0,
            R_R4: 0,
            R_R5: 0,
            R_R6: 0,
            R_R7: 0,
            R_PC: PC_START,
            R_COND: 0,
        }
    }

    pub fn get(&self, index: u16) -> u16 {
        match index {
            0 => self.R_R0,
            1 => self.R_R1,
            2 => self.R_R2,
            3 => self.R_R3,
            4 => self.R_R4,
            5 => self.R_R5,
            6 => self.R_R6,
            7 => self.R_R7,
            8 => self.R_PC,
            9 => self.R_COND,
            _ => panic!("Index out of bound while getting register: {}", index),
        }
    }

    pub fn update(&mut self, index: u16, val: u16) {
        match index {
            0 => self.R_R0 = val,
            1 => self.R_R1 = val,
            2 => self.R_R2 = val,
            3 => self.R_R3 = val,
            4 => self.R_R4 = val,
            5 => self.R_R5 = val,
            6 => self.R_R6 = val,
            7 => self.R_R7 = val,
            8 => self.R_PC = val,
            9 => self.R_COND = val,
            _ => panic!("Index out of bound while updating register: {}", index),
        }
    }

    // The R_COND register stores condition flags which provide information about the most recently executed calculation.
    // Any time a value is written to a register, we use  'update_r_cond_register' fn to update the flags to indicate its sign.
    pub fn update_flags(&mut self, r: u16) {
        let r_cond_index = 9;
        let r = self.get(r);

        if r == 0 {
            self.update(r_cond_index, ConditionFlags::FL_ZRO.get_flag_value())
        } else if (r >> 15) != 0 {
            self.update(r_cond_index, ConditionFlags::FL_NEG.get_flag_value())
        } else {
            self.update(r_cond_index, ConditionFlags::FL_POS.get_flag_value())
        }
    }
}

#[allow(non_camel_case_types)]
pub enum Instructions {
    OP_BR = 0, /* branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP,   /* execute trap */
}

const MEMORY_SIZE: usize = u16::MAX as usize + 1;

#[allow(non_camel_case_types)]
enum MemoryMappedRegisters {
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02, /* keyboard data */
}

pub struct VM {
    pub registers: Registers,
    pub memory: [u16; MEMORY_SIZE],
}

impl VM {
    pub fn new() -> Self {
        Self {
            registers: Registers::new(),
            memory: [0; MEMORY_SIZE],
        }
    }

    pub fn mem_write(&mut self, address: u16, val: u16) {
        self.memory[address as usize] = val;
    }

    pub fn mem_read(&mut self, address: u16) -> u16 {
        if address == MemoryMappedRegisters::MR_KBSR as u16 {
            let mut buffer = [0; 1];
            std::io::stdin()
                .read_exact(&mut buffer)
                .expect("failed to get character from keyboard");

            if buffer[0] != 0 {
                self.memory[MemoryMappedRegisters::MR_KBSR as usize] = 1 << 15;
                self.memory[MemoryMappedRegisters::MR_KBDR as usize] = buffer[0] as u16;
            } else {
                self.memory[MemoryMappedRegisters::MR_KBSR as usize] = 0;
            }
        }

        return self.memory[address as usize];
    }

    pub fn load_file_into_memory(&mut self, filepath: String) -> io::Result<()> {
        let file = File::open(filepath)?;
        let mut file_reader = BufReader::new(file);

        let base_address = file_reader.read_u16::<BigEndian>()?;
        let mut address = base_address;

        loop {
            match file_reader.read_u16::<BigEndian>() {
                Ok(instruction) => {
                    self.mem_write(address, instruction);
                    address += 1;
                }
                Err(e) => {
                    if e.kind() == ErrorKind::UnexpectedEof {
                        println!("OK")
                    } else {
                        println!("failed to load file into memory: {}", e);
                    }

                    return Ok(());
                }
            }
        }
    }

    pub fn execute_instruction(&mut self, instruction: u16) {
        if let Some(op) = get_op_code(instruction >> 12) {
            match op {
                Instructions::OP_BR => self.branch(instruction),
                Instructions::OP_ADD => self.add(instruction),
                Instructions::OP_LD => self.load(instruction),
                Instructions::OP_ST => self.store(instruction),
                Instructions::OP_JSR => self.jump_register(instruction),
                Instructions::OP_AND => self.and(instruction),
                Instructions::OP_LDR => self.load_register(instruction),
                Instructions::OP_STR => self.store_register(instruction),
                Instructions::OP_RTI => (),
                Instructions::OP_NOT => self.not(instruction),
                Instructions::OP_LDI => self.ldi(instruction),
                Instructions::OP_STI => self.store_indirect(instruction),
                Instructions::OP_JMP => self.jump(instruction),
                Instructions::OP_RES => (),
                Instructions::OP_LEA => self.load_effective_address(instruction),
                Instructions::OP_TRAP => self.trap(instruction),
            }
        }
    }

    fn branch(&mut self, instruction: u16) {
        let r_pc_index = 8;

        let pc_offset = sign_extend((instruction) & 0x1FF, 9);
        let cond_flag = (instruction >> 9) & 0x7;

        if cond_flag & self.registers.R_COND != 0 {
            let r_pc = self.registers.R_PC;
            let val = r_pc as u32 + pc_offset as u32;

            self.registers.update(r_pc_index, val as u16);
        }
    }

    fn add(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let sr1 = (instruction >> 6) & 0x7;

        let imm_flag = (instruction >> 5) & 0x1;

        if imm_flag != 0 {
            let imm5 = sign_extend(instruction & 0x1F, 5);

            let val = self.registers.get(sr1) as u32 + imm5 as u32;

            self.registers.update(destination_register, val as u16);
        } else {
            let sr2 = instruction & 0x7;

            let val = self.registers.get(sr1) as u32 + self.registers.get(sr2) as u32;

            self.registers.update(destination_register, val as u16);
        }

        self.registers.update_flags(destination_register);
    }

    fn load(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let pc_offset = sign_extend(instruction & 0x1FF, 9);

        let r_pc = self.registers.R_PC;

        let val = self.mem_read((r_pc as u32 + pc_offset as u32) as u16);

        self.registers.update(destination_register, val);

        self.registers.update_flags(destination_register);
    }

    fn store(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let pc_offset = sign_extend(instruction & 0x1FF, 9);

        let r_pc = self.registers.R_PC;

        let address = (r_pc as u32 + pc_offset as u32) as u16;
        let val = self.registers.get(destination_register);

        self.mem_write(address, val);
    }

    fn jump_register(&mut self, instruction: u16) {
        let r_pc_index = 8;
        let r_7_index = 7;

        let r_pc = self.registers.R_PC;
        self.registers.update(r_7_index, r_pc);

        let long_flag = (instruction >> 11) & 1;

        if long_flag != 0 {
            let long_pc_offset = sign_extend(instruction & 0x7FF, 11);
            let val = r_pc as u32 + long_pc_offset as u32;
            self.registers.update(r_pc_index, val as u16);
        } else {
            let sr1 = (instruction >> 6) & 0x7;
            let val = self.registers.get(sr1);
            self.registers.update(r_pc_index, val);
        }
    }

    fn and(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let sr1 = (instruction >> 6) & 0x7;

        let imm_flag = (instruction >> 5) & 0x1;

        if imm_flag != 0 {
            let imm5 = sign_extend(instruction & 0x1F, 5);

            let val = self.registers.get(sr1) & imm5;

            self.registers.update(destination_register, val);
        } else {
            let sr2 = instruction & 0x7;

            let val = self.registers.get(sr1) & self.registers.get(sr2);

            self.registers.update(destination_register, val);
        }

        self.registers.update_flags(destination_register);
    }

    fn load_register(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let sr1 = (instruction >> 6) & 0x7;

        let offset = sign_extend(instruction & 0x3F, 6);

        let val = self.mem_read((self.registers.get(sr1) as u32 + offset as u32) as u16);

        self.registers.update(destination_register, val);

        self.registers.update_flags(destination_register);
    }

    fn store_register(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let sr1 = (instruction >> 6) & 0x7;

        let offset = sign_extend(instruction & 0x3F, 6);

        let address = self.registers.get(sr1) as u32 + offset as u32;
        let val = self.registers.get(destination_register);

        self.mem_write(address as u16, val);
    }

    fn not(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let sr1 = (instruction >> 6) & 0x7;

        let val = !self.registers.get(sr1);

        self.registers.update(destination_register, val);

        self.registers.update_flags(destination_register);
    }

    fn ldi(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let pc_offset = sign_extend(instruction & 0x1FF, 9);

        let address_location =
            self.mem_read((self.registers.R_PC as u32 + pc_offset as u32) as u16);
        let value_from_mem = self.mem_read(address_location);
        self.registers.update(destination_register, value_from_mem);

        self.registers.update_flags(destination_register);
    }

    fn store_indirect(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let pc_offset = sign_extend(instruction & 0x1FF, 9);

        let r_pc = self.registers.R_PC;

        let address = self.mem_read((r_pc as u32 + pc_offset as u32) as u16);
        let val = self.registers.get(destination_register);

        self.mem_write(address, val);
    }

    fn jump(&mut self, instruction: u16) {
        let r_pc_index = 8;

        let sr1 = (instruction >> 6) & 0x7;

        let val = self.registers.get(sr1);

        self.registers.update(r_pc_index, val);
    }

    fn load_effective_address(&mut self, instruction: u16) {
        let destination_register = (instruction >> 9) & 0x7;

        let pc_offset = sign_extend(instruction & 0x1FF, 9);

        let r_pc = self.registers.R_PC;

        let val = r_pc as u32 + pc_offset as u32;

        self.registers.update(destination_register, val as u16);

        self.registers.update_flags(destination_register);
    }

    fn trap(&mut self, instruction: u16) {
        self.registers.R_R7 = self.registers.R_PC;

        match instruction & 0xFF {
            0x20 => {
                // Get character
                let mut buffer = [0; 1];
                std::io::stdin()
                    .read_exact(&mut buffer)
                    .expect("failed to get character from keyboard");
                self.registers.R_R0 = buffer[0] as u16;
                self.registers.update_flags(0);
            }
            0x21 => {
                // Write out character
                let c = self.registers.R_R0;
                print!("{}", c as u8 as char);
            }
            0x22 => {
                // Puts
                let mut index = self.registers.R_R0;
                let mut c = self.mem_read(index);
                while c != 0x0000 {
                    print!("{}", (c as u8) as char);
                    index += 1;
                    c = self.mem_read(index);
                }
                io::stdout().flush().expect("failed to flush");
            }
            0x23 => {
                // In, Print a prompt on the screen and read a single character from the keyboard.
                // The character is echoed onto the console monitor, and its ASCII code is copied into R0.
                // The high eight bits of R0 are cleared.
                print!("Enter a  character : ");
                io::stdout().flush().expect("failed to flush");
                let char = std::io::stdin()
                    .bytes()
                    .next()
                    .and_then(|result| result.ok())
                    .map(|byte| byte as u16)
                    .expect("failed to get character from keyboard");
                self.registers.update(0, char);
                self.registers.update_flags(0);
            }
            0x24 => {
                // Putsp
                let mut index = self.registers.R_R0;
                let mut c = self.mem_read(index);
                while c != 0x0000 {
                    let c1 = ((c & 0xFF) as u8) as char;
                    print!("{}", c1);
                    let c2 = ((c >> 8) as u8) as char;
                    if c2 != '\0' {
                        print!("{}", c2);
                    }
                    index += 1;
                    c = self.mem_read(index);
                }
                io::stdout().flush().expect("failed to flush");
            }
            0x25 => {
                println!("");
                println!("HALT detected");
                io::stdout().flush().expect("failed to flush");
                process::exit(1);
            }
            _ => {
                println!("invalid TRAP");
                process::exit(1);
            }
        }
    }
}

fn get_op_code(instruction: u16) -> Option<Instructions> {
    match instruction {
        0 => Some(Instructions::OP_BR),
        1 => Some(Instructions::OP_ADD),
        2 => Some(Instructions::OP_LD),
        3 => Some(Instructions::OP_ST),
        4 => Some(Instructions::OP_JSR),
        5 => Some(Instructions::OP_AND),
        6 => Some(Instructions::OP_LDR),
        7 => Some(Instructions::OP_STR),
        8 => Some(Instructions::OP_RTI),
        9 => Some(Instructions::OP_NOT),
        10 => Some(Instructions::OP_LDI),
        11 => Some(Instructions::OP_STI),
        12 => Some(Instructions::OP_JMP),
        13 => Some(Instructions::OP_RES),
        14 => Some(Instructions::OP_LEA),
        15 => Some(Instructions::OP_TRAP),
        _ => None,
    }
}

fn sign_extend(mut x: u16, bit_count: u16) -> u16 {
    if ((x >> (bit_count - 1)) & 1) != 0 {
        x |= 0xFFFF << bit_count;
    }

    x
}
