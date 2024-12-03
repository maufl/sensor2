#![no_std]
#![no_main]

use defmt_rtt as _;

use core::time::Duration;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    Ble, HciConnector,
};
use bme280::i2c::BME280;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::{lp_io::LowPowerOutputOpenDrain, GpioPin},
    i2c::{
        lp_i2c::LpI2c,
        master::{Config, I2c},
    },
    lp_core::{LpCore, LpCoreWakeupSource},
    peripherals::{Peripherals, ADC1, BT, I2C0, LPWR},
    prelude::*,
    rng::Rng,
    rtc_cntl::{sleep::TimerWakeupSource, Rtc},
    timer::timg::TimerGroup,
};

use defmt::println;
use esp_wifi::{ble::controller::BleConnector, EspWifiController};

#[derive(Debug)]
#[repr(C)]
struct Measurement {
    /// In degree C
    temperature: f32,
    /// In Pascal
    pressure: f32,
    /// Relative in %
    humidity: f32,
    cnt: u32,
}

const DEEP_SLEEP_SECONDS: u64 = 60;

const SHARED_MEMORY: u32 = 0x5000_2000;

const U12_MAX: u16 = 1 << 12 - 1;

#[entry]
fn main() -> ! {
    defmt::info!("Starting up");

    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::Clock80MHz;
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    if cfg!(feature = "lp") {
        read_with_lp(peripherals)
    } else {
        read_with_hp(peripherals);
    }
}

fn read_battery_charge(analog_pin: GpioPin<2>, adc: ADC1) -> (u16, f32) {
    let mut adc1_config = AdcConfig::new();
    // At an attenuation of 2.5 dB the input voltage range that can be read is ~100mV-1250mV
    // The board contains an additional voltage devider with 1 MO - 4.7 MO
    // u12::max == 1250mV -> bat / u12::max * 1.25 * 5.7 ??
    let mut pin = adc1_config.enable_pin(analog_pin, Attenuation::Attenuation2p5dB);
    let mut adc1 = Adc::new(adc, adc1_config);
    let bat = nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
    (bat, bat as f32 / U12_MAX as f32 * 1.25 * 5.7)
}

fn sent_ble_single_advertisment(ble: &mut Ble, measurement: &Measurement, battery_charge: f32, raw_bat: u16) {
    let [t0, t1] = ((measurement.temperature * 100.0) as u16).to_le_bytes();
    let [h0, h1] = ((measurement.humidity * 100.0) as u16).to_le_bytes();
    // pressure doesn't have to be scaled since it's already in Pascal
    let [p0, p1, p2, _] = ((measurement.pressure) as u32).to_le_bytes();
    let [b0, b1] = ((battery_charge * 1000.0) as u16).to_le_bytes();
    let [r0, r1] = raw_bat.to_le_bytes();
    let _ = ble.cmd_set_le_advertising_data(
        create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceData16 {
                uuid: 0xFCD2,
                data: &[
                    0x40, 0x02, t0, t1, 0x03, h0, h1, 0x04, p0, p1, p2, 0x0C, b0, b1, 0x3D, r0, r1
                ],
            },
            AdStructure::CompleteLocalName("esp32"),
        ])
        .unwrap(),
    );
    let _ = ble.cmd_set_le_advertise_enable(true);
}

fn measure_climate(
    i2c0: I2C0,
    gpio06: GpioPin<6>,
    gpio07: GpioPin<7>,
) -> Result<Measurement, bme280::Error<esp_hal::i2c::master::Error>> {
    let mut delay = Delay::new();
    let i2c = I2c::new(
        i2c0,
        Config {
            frequency: 100.kHz(),
            timeout: None,
        },
    )
    .with_sda(gpio06)
    .with_scl(gpio07);

    let mut bme280 = BME280::new_primary(i2c);
    defmt::debug!("Initializing sensor");
    bme280.init(&mut delay)?;
    bme280.measure(&mut delay).map(|m| Measurement {
        temperature: m.temperature,
        humidity: m.humidity,
        pressure: m.pressure,
        cnt: 0,
    })
}

fn goto_deep_sleep(lpwr: LPWR) -> ! {
    let mut rtc = Rtc::new(lpwr);
    let timer = TimerWakeupSource::new(Duration::from_secs(DEEP_SLEEP_SECONDS));
    defmt::debug!("sleeping!");
    rtc.sleep_deep(&[&timer]);
}

fn sent_ble_advertisments<'d>(wifi_controller: &EspWifiController<'d>, mut bt: BT, measurement: &Measurement, battery_charge: f32, raw_bat: u16) {
    let delay = Delay::new();
    defmt::debug!("Setting up BLE connector");
    let connector = BleConnector::new(&wifi_controller, &mut bt);
    let now = || esp_hal::time::now().duration_since_epoch().to_millis();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);
    defmt::debug!("Finished setup of Bluetooth stack, initializing");
    if let Err(err) = ble.init() {
        return defmt::error!("Error initializing BTL stack {}", err)
    }
    let _ = ble.cmd_set_le_advertising_parameters();

    sent_ble_single_advertisment(&mut ble, &measurement, battery_charge, raw_bat);
    delay.delay_millis(100);

    if let Err(e) = ble.cmd_reset() {
        defmt::debug!("Error resetting BLE stack {:?}", e)
    }
}

fn read_with_hp(peripherals: Peripherals) -> ! {
    let Peripherals {
        LPWR,
        LP_CORE,
        I2C0,
        ADC1,
        GPIO2,
        GPIO6,
        GPIO7,
        TIMG0,
        RNG,
        RADIO_CLK,
        BT,
        ..
    } = peripherals;

    defmt::debug!("Disabling LP core");
    let mut lp_core = LpCore::new(LP_CORE);
    lp_core.stop();

    let measurement = measure_climate(I2C0, GPIO6, GPIO7).expect("measurement to succeed");
    let (raw_bat, battery_charge) = read_battery_charge(GPIO2, ADC1);

    defmt::debug!("Starting setup of Bluetooth stack");
    let timg0 = TimerGroup::new(TIMG0);
    defmt::debug!("Initializing Wifi controller");
    let wifi_controller = esp_wifi::init(timg0.timer0, Rng::new(RNG), RADIO_CLK)
        .expect("To initialize Wifi controller");

    sent_ble_advertisments(&wifi_controller, BT, &measurement, battery_charge, raw_bat);

    if let Err(err) = wifi_controller.deinit() {
        defmt::debug!("Error deinitializing wifi controller {:?}", err);
    }

    goto_deep_sleep(LPWR)
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
