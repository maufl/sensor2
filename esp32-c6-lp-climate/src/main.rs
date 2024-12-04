#![no_std]
#![no_main]

use bme280::i2c::BME280;
use esp_lp_hal::{i2c::LpI2c, prelude::*, wake_hp_core};
use panic_halt as _;
use shared::{Measurement, MeasurmentBuffer, MeasurmentError, SHARED_MEMORY_ADDRESS};

#[derive(Debug, Copy, Clone)]
struct Delay;

impl embedded_hal::blocking::delay::DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        esp_lp_hal::delay::Delay {}.delay_ms(ms.into());
    }
}

fn convert_error(value: &bme280::Error<esp_lp_hal::i2c::Error>) -> MeasurmentError {
    match value {
        bme280::Error::CompensationFailed => MeasurmentError::CompensationFailed,
        bme280::Error::Bus(_) => MeasurmentError::BusError,
        bme280::Error::InvalidData => MeasurmentError::InvalidData,
        bme280::Error::NoCalibrationData => MeasurmentError::NoCalibrationData,
        bme280::Error::UnsupportedChip => MeasurmentError::UnsupportedChip,
    }
}

#[entry]
fn main(i2c: LpI2c) -> ! {
    let _peripherals = esp32c6_lp::Peripherals::take().unwrap();
    let mut sensor = BME280::new_primary(i2c, Delay {});
    let delay = esp_lp_hal::delay::Delay {};
    let buffer = unsafe {
        (SHARED_MEMORY_ADDRESS as *mut MeasurmentBuffer)
            .as_mut()
            .unwrap()
    };
    *buffer = MeasurmentBuffer {
        measurements: [Measurement {
            temperature: 0.0,
            pressure: 0.0,
            humidity: 0.0,
            error: None,
        }; 10],
        next_index: 0,
    };

    let initialization_error = sensor.init().err();

    loop {
        let current_measurement = &mut buffer.measurements[buffer.next_index as usize];
        *current_measurement = Measurement {
            temperature: 0.0,
            pressure: 0.0,
            humidity: 0.0,
            error: None,
        };
        if let Some(ref err) = initialization_error {
            current_measurement.error = Some(convert_error(err));
        } else {
            match sensor.measure() {
                Ok(m) => {
                    *current_measurement = Measurement {
                        temperature: m.temperature,
                        pressure: m.pressure,
                        humidity: m.humidity,
                        error: None,
                    }
                }
                Err(err) => current_measurement.error = Some(convert_error(&err)),
            }
        }
        buffer.next_index = buffer.next_index + 1 % 10;
        delay.delay_millis(6 * 1000);
        wake_hp_core();
    }
}
