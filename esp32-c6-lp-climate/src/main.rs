
#![no_std]
#![no_main]

use bme280::i2c::BME280;
use esp_lp_hal::{i2c::LpI2c, prelude::*};
use panic_halt as _;


#[repr(C)]
struct Measurement {
    temperature: f32,
    pressure: f32,
    humidity: f32,
    cnt: u32,
}

const SHARED_MEMORY: u32 = 0x5000_2000;

#[entry]
fn main(i2c: LpI2c) -> ! {
    let _peripherals = esp32c6_lp::Peripherals::take().unwrap();
    let mut sensor = BME280::new_primary(i2c);
    let mut delay = esp_lp_hal::delay::Delay{};
    let ptr = SHARED_MEMORY as *mut Measurement;
    let mut cnt = 0u32;
    unsafe {
        ptr.write_volatile(Measurement {
            temperature: 0.0,
            pressure: 0.0,
            humidity: 0.0,
            cnt,
        })
    }
    cnt += 1;
    if let Err(e) = sensor.init(&mut delay) {
        let err_code = match e {
            bme280::Error::CompensationFailed => 1.0,
            bme280::Error::Bus(bus_error) => 200.0 + f32::from(bus_error as u8),
            bme280::Error::InvalidData => 3.0,
            bme280::Error::NoCalibrationData => 4.0,
            bme280::Error::UnsupportedChip => 5.0,
            bme280::Error::Delay => 6.0,
        };
        unsafe {
            ptr.write_volatile(Measurement {
                temperature: f32::NAN,
                pressure: f32::NAN,
                humidity: err_code,
                cnt,
            })
        }
    } else {
        unsafe {
            ptr.write_volatile(Measurement {
                temperature: 1.0,
                pressure: 1.0,
                humidity: 1.0,
                cnt,
            })
        }
    };
    cnt += 1;
    delay.delay_millis(3_000);
    loop {
        match sensor.measure(&mut delay) {
            Ok(m) => {
                let measurement = Measurement {
                    temperature: m.temperature,
                    pressure: m.pressure,
                    humidity: m.humidity,
                    cnt,
                };
                unsafe {
                    ptr.write_volatile(measurement)
                }
            },
            Err(e) => {
                let err_code = match e {
                    bme280::Error::CompensationFailed => 1.0,
                    bme280::Error::Bus(bus_error) => 200.0 + f32::from(bus_error as u8),
                    bme280::Error::InvalidData => 3.0,
                    bme280::Error::NoCalibrationData => 4.0,
                    bme280::Error::UnsupportedChip => 5.0,
                    bme280::Error::Delay => 6.0,
                };
                unsafe {
                    ptr.write_volatile(Measurement {
                        temperature: f32::NAN,
                        pressure: f32::NAN,
                        humidity: err_code,
                        cnt,
                    })
                }
            }
        }
        cnt += 1;
        delay.delay_millis(1_000);
    }
}