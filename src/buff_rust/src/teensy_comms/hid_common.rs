extern crate hidapi;

use hidapi::{HidApi, HidDevice};

pub static TEENSY_CYCLE_TIME_S: f64 = 0.001;
pub static TEENSY_CYCLE_TIME_MS: f64 = TEENSY_CYCLE_TIME_S * 1000.0;
pub static TEENSY_CYCLE_TIME_US: f64 = TEENSY_CYCLE_TIME_MS * 1000.0;

pub fn init_hid_device(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidDevice {
    let dev = hidapi.open(vid, pid).unwrap();
    dev.set_blocking_mode(false).unwrap();
    dev
}
