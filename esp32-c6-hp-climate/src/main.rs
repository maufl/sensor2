
#![no_std]
#![no_main]

use core::time::Duration;

use bme280::i2c::BME280;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{lp_io::LowPowerOutputOpenDrain, Io}, i2c::{lp_i2c::LpI2c, I2C}, lp_core::{LpCore, LpCoreWakeupSource}, peripherals::{self, Peripherals}, prelude::*, rtc_cntl::{sleep::TimerWakeupSource, Rtc}, system::SystemControl
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

    read_with_hp(peripherals);
}

fn read_with_hp(peripherals: Peripherals) -> ! {
    
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(peripherals.I2C0, io.pins.gpio6, io.pins.gpio7, 100.kHz(), &clocks, None);

    let mut bme280 = BME280::new_primary(i2c);
    println!("Initializing sensor");
    if let Err(e) = bme280.init(&mut delay) {
        println!("Error initializing sensor {:?}", e)
    };
    println!("Finished initializing sensor");
    for _ in 0..10 {
        match bme280.measure(&mut delay) {
            Ok(m) => print!("Measured t:{} p:{} h:{}       \u{000d}", m.temperature, m.pressure, m.humidity),
            Err(e) => println!("Error reading sensor {:?}", e),
        };
        delay.delay_millis(2_000);
    }

    let mut rtc = Rtc::new(peripherals.LPWR, None);

    let timer = TimerWakeupSource::new(Duration::from_secs(5));
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer]);

    loop {}
}

fn read_with_lp(peripherals: Peripherals) -> ! {
 
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
