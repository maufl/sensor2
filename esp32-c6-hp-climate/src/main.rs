
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{lp_io::LowPowerOutputOpenDrain, Io}, i2c::lp_i2c::LpI2c, lp_core::{LpCore, LpCoreWakeupSource}, peripherals::Peripherals, prelude::*, system::SystemControl
};
use esp_println::{print, println};

#[derive(Debug)]
#[repr(C)]
struct Measurement {
    temperature: f32,
    pressure: f32,
    humidity: f32,
    cnt: u32,
}

const SHARED_MEMORY: u32 = 0x5000_2000;

#[entry]
fn main() -> ! {    
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
 
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let lp_sda = LowPowerOutputOpenDrain::new(io.pins.gpio6);
    let lp_scl = LowPowerOutputOpenDrain::new(io.pins.gpio7);

    let lp_i2c = LpI2c::new(peripherals.LP_I2C0, lp_sda, lp_scl, 100.kHz());

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code =
        load_lp_code!("../esp32-c6-lp-climate/target/riscv32imac-unknown-none-elf/release/esp32-c6-lp-climate");

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_i2c);
    println!("lpcore run");

    let ptr = SHARED_MEMORY as *mut Measurement;
    loop {
        print!("Current {:?}           \u{000d}", unsafe {
            ptr.read_volatile()
        });
        delay.delay_millis(1_000);
    }
}
