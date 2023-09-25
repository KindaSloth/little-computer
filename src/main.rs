use lc3vm::vm::VM;

fn main() {
    let mut vm = VM::new();

    vm.load_file_into_memory("./resources/hello-world.obj".to_string())
        .expect("failed to load file into memory");

    loop {
        let instruction = vm.mem_read(vm.registers.R_PC);
        vm.registers.R_PC += 1;

        vm.execute_instruction(instruction);
    }
}
