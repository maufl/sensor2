#![no_std]
#![no_main]

use defmt_rtt as _;

use core::time::Duration;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    att::Uuid,
    Ble, HciConnector,
};
use bme280::i2c::BME280;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation}, delay::Delay, gpio::{lp_io::LowPowerOutputOpenDrain, GpioPin}, i2c::{
        lp_i2c::LpI2c,
        master::{Config, I2c},
    }, lp_core::{LpCore, LpCoreWakeupSource}, peripherals::{Peripherals, ADC1, BT, RADIO_CLK, RNG, TIMG0}, prelude::*, rng::Rng, rtc_cntl::{sleep::TimerWakeupSource, Rtc}, timer::timg::TimerGroup
};

use defmt::println;
use esp_wifi::ble::controller::BleConnector;

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
    defmt::info!("Starting up");
    
    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::max();
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    if cfg!(feature = "lp") {
        read_with_lp(peripherals)
    } else {
        read_with_hp(peripherals);
    }
}

fn read_battery_charge(analog_pin: GpioPin<2>, adc: ADC1) -> u16 {
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(analog_pin, Attenuation::Attenuation2p5dB);
    let mut adc1 = Adc::new(adc, adc1_config);

    return nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
}

fn sent_ble_advertisment(ble: &mut Ble) {
    let _ = ble.cmd_set_le_advertising_data(
        create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16(0xFCD2)]),
            AdStructure::CompleteLocalName("esp32c6"),
        ])
        .unwrap(),
    );
    let _ = ble.cmd_set_le_advertise_enable(true);
}

fn read_with_hp(peripherals: Peripherals) -> ! {
    let Peripherals {
        LPWR,
        I2C0,
        ADC1,
        GPIO2,
        GPIO6,
        GPIO7,
        TIMG0,
        RNG,
        RADIO_CLK,
        mut BT,
        ..
    } = peripherals;
    let mut delay = Delay::new();

    let i2c = I2c::new(
        I2C0,
        Config {
            frequency: 100.kHz(),
            timeout: None,
        },
    )
    .with_sda(GPIO6)
    .with_scl(GPIO7);

    let mut bme280 = BME280::new_primary(i2c);
    defmt::info!("Initializing sensor");
    if let Err(e) = bme280.init(&mut delay) {
        println!("Error initializing sensor {:?}", e)
    };
    defmt::info!("Finished initializing sensor");

    println!("Starting setup of Bluetooth stack");
    let timg0 = TimerGroup::new(TIMG0);
    println!("Initializing Wifi controller");
    let wifi_controller = esp_wifi::init(timg0.timer0, Rng::new(RNG), RADIO_CLK)
       .expect("To initialize Wifi controller");
    println!("Setting up BLE connector");
    let connector = BleConnector::new(&wifi_controller, &mut BT);
    let now = || esp_hal::time::now().duration_since_epoch().to_millis();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);
    println!("Finished setup of Bluetooth stack, initializing");
    println!("{:?}", ble.init());
    let _ = ble.cmd_set_le_advertising_parameters();

    let battery_charge = read_battery_charge(GPIO2, ADC1);
    loop {
        match bme280.measure(&mut delay) {
            Ok(m) => {
                println!(
                    "Measured t:{} p:{} h:{} b:{}      \u{000d}",
                    m.temperature, m.pressure, m.humidity, battery_charge
                );
                sent_ble_advertisment(&mut ble);
            }
            Err(e) => println!("Error reading sensor {:?}", e),
        };
        delay.delay_millis(2_000);
    }

    let mut rtc = Rtc::new(LPWR);

    let timer = TimerWakeupSource::new(Duration::from_secs(5));
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer]);
}

fn read_with_lp(peripherals: Peripherals) -> ! {
    let delay = Delay::new();

    let lp_sda = LowPowerOutputOpenDrain::new(peripherals.GPIO6);
    let lp_scl = LowPowerOutputOpenDrain::new(peripherals.GPIO7);

    let lp_i2c = LpI2c::new(peripherals.LP_I2C0, lp_sda, lp_scl, 100.kHz());

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../esp32-c6-lp-climate/target/riscv32imac-unknown-none-elf/release/esp32-c6-lp-climate"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_i2c);
    println!("lpcore run");

    let ptr = SHARED_MEMORY as *mut Measurement;
    loop {
        let m = unsafe { ptr.read_volatile() };
        println!(
            "Measured t:{} p:{} h:{}      \u{000d}",
            m.temperature, m.pressure, m.humidity
        );
        delay.delay_millis(1_000);
    }
}
