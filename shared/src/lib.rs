
#![no_std]

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum MeasurmentError {
    CompensationFailed = 0x1,
    BusError = 0x2,
    InvalidData = 0x3,
    NoCalibrationData = 0x4,
    UnsupportedChip = 0x5,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Measurement {
    pub temperature: f32,
    pub pressure: f32,
    pub humidity: f32,
    pub error: Option<MeasurmentError>,
}

#[repr(C)]
pub struct MeasurmentBuffer {
    pub measurements: [Measurement; 10],
    pub next_index: u8,
}

pub const SHARED_MEMORY_ADDRESS: u32 = 0x5000_2000;