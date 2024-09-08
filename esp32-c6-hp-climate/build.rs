fn main() {
    println!("cargo:rerun-if-changed=../esp32-c6-lp-climate/target/riscv32imac-unknown-none-elf/release/esp32-c6-lp-climate");
}