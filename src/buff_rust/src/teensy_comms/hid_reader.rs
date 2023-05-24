extern crate hidapi;

use crate::teensy_comms::{data_structures::*, hid_common::*};
use crate::utilities::buffers::*;
use hidapi::{HidApi, HidDevice};
use std::{
    sync::{mpsc::Sender, Arc, RwLock},
    time::Instant,
};

/// Responsible for initializing [RobotStatus] and continuously
/// sending status reports
pub struct HidReader {
    pub shutdown: Arc<RwLock<bool>>,
    pub input: ByteBuffer,
    pub robot_status: RobotStatus,
    pub teensy: HidDevice,
    pub teensy_lifetime: f64,
    pub rust_lifetime: f64,
}

impl HidReader {
    pub fn new(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidReader {
        let device = init_hid_device(hidapi, vid, pid); // not two teensys, just two instances

        HidReader {
            shutdown: Arc::new(RwLock::new(false)),
            input: ByteBuffer::new(64),
            robot_status: RobotStatus::default(),
            teensy: device,
            teensy_lifetime: 0.0,
            rust_lifetime: 0.0,
        }
    }

    /// Read data into the input buffer and return how many bytes were read
    ///
    /// # Usage
    ///
    /// ```
    /// match reader.read() {
    ///     64 => {
    ///         // packet OK, do something
    ///     }
    ///     _ => {} // do nothing
    /// }
    /// ```
    pub fn read(&mut self) -> usize {
        match &self.teensy.read(&mut self.input.data) {
            Ok(value) => {
                // reset watchdog... woof
                self.input.timestamp = Instant::now();
                return *value;
            }
            _ => {
                *self.shutdown.write().unwrap() = true;
            }
        }
        return 0;
    }

    /// After requesting a report this can be used to wait for a reply
    ///
    /// # Panics
    ///
    /// This will panic if a reply is not received from the Teensy
    /// within `timeout` ms.
    ///
    /// # Usage
    ///
    /// ```
    /// // set writer.output.data[0] to an int [1-3] (255 for initializer)
    /// writer.write();
    /// reader.wait_for_report_reply(255, 10);
    /// ```
    pub fn wait_for_report_reply(&mut self, packet_id: u8, timeout: u128) {
        let mut loopt;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match self.read() {
                64 => {
                    if self.input.get(0) == 0 && self.input.get_float(60) == 0.0 {
                        continue;
                    } else if self.input.get_float(60) > TEENSY_CYCLE_TIME_US {
                        println!(
                            "Teensy cycle time is over the limit {}",
                            self.input.get_float(60)
                        );
                    }

                    self.teensy_lifetime = 0.0;
                    self.rust_lifetime = 0.0;

                    if self.input.get(0) == packet_id {
                        println!("Teenys report {} reply received", packet_id);
                        return;
                    }
                }
                _ => {}
            }

            // HID runs at 1 ms
            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }

        // If packet never arrives
        *self.shutdown.write().unwrap() = true;
        panic!("\tTimed out waiting for reply from Teensy");
    }

    /// Parse the stored HID packet into BuffBot Data Structures
    ///
    /// # Usage
    ///
    /// ```
    /// // HID packet waiting
    /// if reader.read() > 0 { // read returns the number of bytes (64 or bust)
    ///     reader.parse_report();
    /// }
    /// ```
    pub fn parse_report(&mut self) {
        // println!(
        //     "Lifetime relations {} - {} = {}",
        //     self.teensy_lifetime,
        //     self.rust_lifetime,
        //     self.teensy_lifetime - self.rust_lifetime
        // );

        // if self.input.get_float(60) - self.teensy_lifetime > TEENSY_CYCLE_TIME_S + 1e-2 {
        //     println!(
        //         "Teensy cycle time is over the limit {}",
        //         self.input.get_float(60) - self.teensy_lifetime
        //     );
        // }

        self.teensy_lifetime = self.input.get_float(60);
        // println!("Read packet: {} {}", self.input.get(0), self.teensy_lifetime as f64 / 1e6); // as seconds

        // match the report number to determine the structure
        if self.input.get(0) == SENSOR_REPORT_ID {
            if self.input.get(1) == READ_MODE {
                self.robot_status.update_sensor(
                    self.input.get(2) as usize,
                    self.input.get_floats(3, MAX_SENSOR_BUFFER_SIZE),
                    self.teensy_lifetime,
                );
            }
        }

        if self.input.get(0) == MOTOR_REPORT_ID {
            if self.input.get(1) == READ_MODE {
                self.robot_status.update_motor(
                    self.input.get(2) as usize,
                    self.input.get_floats(3, MAX_SENSOR_BUFFER_SIZE),
                    self.teensy_lifetime,
                );
            }
        }

        if self.input.get(0) == PROC_REPORT_ID {
            if self.input.get(1) == READ_INPUT_MODE {
                self.robot_status.update_proc_input(
                    self.input.get(2) as usize,
                    self.input.get_floats(3, MAX_PROCESS_IO),
                    self.teensy_lifetime,
                );
            }
            if self.input.get(1) == READ_STATE_MODE {
                self.robot_status.update_proc_state(
                    self.input.get(2) as usize,
                    self.input.get(3) as usize,
                    self.input.get_floats(4, MAX_PROCESS_STATES),
                    self.teensy_lifetime,
                );
            }
            if self.input.get(1) == READ_OUTPUT_MODE {
                self.robot_status.update_proc_output(
                    self.input.get(2) as usize,
                    self.input.get_floats(3, MAX_PROCESS_STATES),
                    self.teensy_lifetime,
                );
            }
        }
    }

    /// Main function to spin and connect the teensys
    /// input to ROS.
    ///
    /// # Usage
    /// ```
    /// use hidapi::HidApi;
    /// use buff_rust::teensy_comms::buff_hid::HidReader;
    ///
    /// let mut hidapi = HidApi::new().expect("Failed to create API instance");
    /// let mut reader = HidReader::new(&mut hidapi, vid, pid);
    /// reader.spin();       // runs until watchdog times out
    /// ```
    pub fn spin(&mut self) {
        let mut readt = Instant::now();

        while !*self.shutdown.read().unwrap() {
            let loopt = Instant::now();

            match self.read() {
                64 => {
                    self.parse_report();
                    readt = Instant::now();
                }

                _ => {
                    if readt.elapsed().as_millis() > 10 && readt.elapsed().as_millis() % 10 == 0 {
                        println!(
                            "HID Reader: No reply from Teensy for {}",
                            readt.elapsed().as_millis()
                        );
                    }
                }
            }

            self.rust_lifetime += TEENSY_CYCLE_TIME_S;
            if loopt.elapsed().as_micros() > TEENSY_CYCLE_TIME_US as u128 {
                println!("HID reader over cycled {}", loopt.elapsed().as_micros());
            }
            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }
    }

    /// Sends robot status report packet to [HidROS], waits for the reply packet,
    /// then calls [HidReader::spin] to begin parsing reports
    ///
    /// # Example
    ///
    /// see [HidLayer::pipeline()]
    pub fn pipeline(&mut self, shutdown: Arc<RwLock<bool>>, feedback_tx: Sender<RobotStatus>) {
        self.shutdown = shutdown;

        feedback_tx.send(self.robot_status.clone()).unwrap();
        println!("HID-reader Live");

        // wait for initializers reply
        self.wait_for_report_reply(255, 50);

        self.spin();
    }
}
