extern crate hidapi;

use crate::teensy_comms::hid_common::*;
use crate::utilities::buffers::*;
use hidapi::{HidApi, HidDevice};
use std::{
    sync::{mpsc::Receiver, Arc, RwLock},
    time::Instant,
};

pub struct HidWriter {
    pub shutdown: Arc<RwLock<bool>>,
    pub output: ByteBuffer,
    pub teensy: HidDevice,
}

impl HidWriter {
    pub fn new(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidWriter {
        let device = init_hid_device(hidapi, vid, pid); // not two teensys, just two instances

        HidWriter {
            // shutdown will be replaced by a variable shared between several HidWriters
            shutdown: Arc::new(RwLock::new(false)),
            output: ByteBuffer::new(64),
            teensy: device,
        }
    }

    /// Write the bytes from the output buffer to the teensy, then clear the buffer.
    /// Shutdown if the write fails.
    /// # Usage
    /// ```
    /// writer.output.puts(some_index, some_data);
    /// writer.write(); // writes some_data to the teensy
    /// ```
    pub fn write(&mut self) {
        // let t = Instant::now();
        match self.teensy.write(&self.output.data) {
            Ok(_) => {
                // println!("Write time {}", t.elapsed().as_micros());
                // self.output.print_data();
                self.output.reset();
            }
            _ => {
                *self.shutdown.write().unwrap() = true;
            }
        }
    }

    /// Creates a report from `id` and `data` and sends it to the teensy. Only use in testing.
    /// # Usage
    /// ```
    ///     writer.teensy = hidapi.open(vid, pid);
    ///     writer.send_report(report_id, data);
    /// ```
    pub fn send_report(&mut self, id: u8, data: Vec<u8>) {
        self.output.puts(0, vec![id]);
        self.output.puts(1, data);
        self.write();
    }

    /// Main function to spin and connect the teensy to ROS.
    /// Cycles through `reports` and sends a report to the teensy every millisecond.
    /// Only use in testing.
    pub fn spin(&mut self, reports: Vec<Vec<u8>>) {
        let mut report_request = 0;
        println!("HID-writer Live");

        while !*self.shutdown.read().unwrap() && self.output.timestamp.elapsed().as_millis() < 50 {
            let loopt = Instant::now();

            self.output.puts(0, reports[report_request].clone());
            self.write();
            report_request = (report_request + 1) % reports.len();
            if loopt.elapsed().as_micros() > TEENSY_CYCLE_TIME_US as u128 {
                println!("HID writer over cycled {}", loopt.elapsed().as_micros());
            }
            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }
        *self.shutdown.write().unwrap() = true;
    }

    /// Continually sends data from [HidROS] (or whatever owns the other end of `control_rx`) to the teensy.
    ///
    /// # Arguments
    /// * `shutdown` - The function stops when this is true.
    /// Used so that HidLayer threads, all running pipeline() at the same time, can be shutdown at the same time (by passing them the same variable)
    /// * `control_rx` - Receives the data from [HidROS].
    ///
    /// # Example
    /// See [`HidLayer::pipeline()`] source
    pub fn pipeline(&mut self, shutdown: Arc<RwLock<bool>>, control_rx: Receiver<Vec<u8>>) {
        self.shutdown = shutdown;

        // let mut status = RobotStatus::load_robot();

        // self.send_report(255, status.load_initializers()[0].clone());

        println!("HID-writer Live");

        while !*self.shutdown.read().unwrap() {
            self.output.puts(0, control_rx.recv().unwrap_or(vec![0]));
            self.write();
        }
    }
}
