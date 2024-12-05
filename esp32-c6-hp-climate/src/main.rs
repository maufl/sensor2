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
    peripherals::{Peripherals, ADC1, BT, I2C0, LPWR, LP_CORE, LP_I2C0},
    prelude::*,
    reset::{wakeup_cause, SleepSource},
    rng::Rng,
    rtc_cntl::{
        sleep::{TimerWakeupSource, WakeFromLpCoreWakeupSource},
        Rtc,
    },
    timer::timg::TimerGroup,
};
use esp_wifi::{ble::controller::BleConnector, EspWifiController};

use shared::{Measurement, MeasurmentBuffer, SHARED_MEMORY_ADDRESS};

const DEEP_SLEEP_SECONDS: u64 = 10 * 60;
const U12_MAX: u16 = (1 << 12) - 1;

#[entry]
fn main() -> ! {
    defmt::info!("Starting up");

    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::Clock80MHz;
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    let Peripherals {
        LPWR,
        LP_CORE,
        I2C0,
        LP_I2C0,
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

    let battery_charge = read_battery_charge(GPIO2, ADC1);

    let measurement = if cfg!(feature = "lp") {
        if let SleepSource::Ulp = wakeup_cause() {
            read_lp_measurement()
        } else {
            start_lp_core(GPIO6, GPIO7, LP_I2C0, LP_CORE);
            sleep_wake_lp_core(LPWR);
        }
    } else {
        defmt::debug!("Disabling LP core");
        LpCore::new(LP_CORE).stop();
        measure_climate(I2C0, GPIO6, GPIO7).expect("measurement to succeed")
    };

    defmt::debug!("Starting setup of Bluetooth stack");
    let timg0 = TimerGroup::new(TIMG0);
    defmt::debug!("Initializing Wifi controller");
    let wifi_controller = match esp_wifi::init(timg0.timer0, Rng::new(RNG), RADIO_CLK) {
        Ok(v) => v,
        Err(err) => {
            defmt::error!("Failed to initialize wifi: {}", err);
            loop {}
        }
    };

    sent_ble_advertisments(&wifi_controller, BT, &measurement, battery_charge);

    if let Err(err) = wifi_controller.deinit() {
        defmt::debug!("Error deinitializing wifi controller {:?}", err);
    }

    if cfg!(feature = "lp") {
        sleep_wake_lp_core(LPWR);
    } else {
        sleep_wake_timer(LPWR);
    }
}

type AdcCal = esp_hal::analog::adc::AdcCalBasic<esp_hal::peripherals::ADC1>;

fn read_battery_charge(analog_pin: GpioPin<2>, adc: ADC1) -> f32 {
    let mut adc1_config = AdcConfig::new();
    // At an attenuation of 2.5 dB the input voltage range that can be read is ~100mV-1250mV
    // The board contains an additional voltage devider with 1 MO - 4.7 MO
    // u12::max == 1250mV -> bat / u12::max * 1.25 * 5.7 ??
    let mut pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(analog_pin, Attenuation::Attenuation2p5dB);
    let mut adc1 = Adc::new(adc, adc1_config);
    let bat = nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
    (bat as f32 / U12_MAX as f32) * 7.125
}

fn sent_ble_single_advertisment(
    ble: &mut Ble,
    measurement: &Measurement,
    battery_charge: f32,
) {
    let [t0, t1] = ((measurement.temperature * 100.0) as u16).to_le_bytes();
    let [h0, h1] = ((measurement.humidity * 100.0) as u16).to_le_bytes();
    // pressure doesn't have to be scaled since it's already in Pascal
    let [p0, p1, p2, _] = ((measurement.pressure) as u32).to_le_bytes();
    let [b0, b1] = ((battery_charge * 1000.0) as u16).to_le_bytes();
    let _ = ble.cmd_set_le_advertising_data(
        create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceData16 {
                uuid: 0xFCD2,
                data: &[
                    0x40, 0x02, t0, t1, 0x03, h0, h1, 0x04, p0, p1, p2, 0x0C, b0, b1,
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
        error: None,
    })
}

fn sleep_wake_timer(lpwr: LPWR) -> ! {
    let mut rtc = Rtc::new(lpwr);
    let timer = TimerWakeupSource::new(Duration::from_secs(DEEP_SLEEP_SECONDS));
    defmt::debug!("sleeping!");
    rtc.sleep_deep(&[&timer]);
}

fn sleep_wake_lp_core(lpwr: LPWR) -> ! {
    let mut rtc = Rtc::new(lpwr);
    let waker = WakeFromLpCoreWakeupSource::new();
    defmt::debug!("sleeping!");
    rtc.sleep_deep(&[&waker]);
}

fn sent_ble_advertisments<'d>(
    wifi_controller: &EspWifiController<'d>,
    mut bt: BT,
    measurement: &Measurement,
    battery_charge: f32,
) {
    let delay = Delay::new();
    defmt::debug!("Setting up BLE connector");
    let connector = BleConnector::new(&wifi_controller, &mut bt);
    let now = || esp_hal::time::now().duration_since_epoch().to_millis();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);
    defmt::debug!("Finished setup of Bluetooth stack, initializing");
    if let Err(err) = ble.init() {
        return defmt::error!("Error initializing BTL stack {}", err);
    }
    let _ = ble.cmd_set_le_advertising_parameters();

    sent_ble_single_advertisment(&mut ble, &measurement, battery_charge);
    delay.delay_millis(100);

    if let Err(e) = ble.cmd_reset() {
        defmt::debug!("Error resetting BLE stack {:?}", e)
    }
}

fn start_lp_core(gpio06: GpioPin<6>, gpio07: GpioPin<7>, lp_i2c: LP_I2C0, lp_core: LP_CORE) {
    let lp_sda = LowPowerOutputOpenDrain::new(gpio06);
    let lp_scl = LowPowerOutputOpenDrain::new(gpio07);

    let lp_i2c = LpI2c::new(lp_i2c, lp_sda, lp_scl, 100.kHz());

    let mut lp_core = LpCore::new(lp_core);

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../esp32-c6-lp-climate/target/riscv32imac-unknown-none-elf/release/esp32-c6-lp-climate"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_i2c);
    defmt::debug!("Starting LP core");
}

fn read_lp_measurement() -> Measurement {
    let ptr = SHARED_MEMORY_ADDRESS as *mut MeasurmentBuffer;
    let buffer = unsafe { ptr.read_volatile() };
    let last_measurement = (buffer.next_index - 1 % 10) as usize;
    buffer.measurements[last_measurement]
}
