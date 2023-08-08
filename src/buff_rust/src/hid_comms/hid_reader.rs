extern crate hidapi;

use crate::hid_comms::hid_layer::*;
use crate::utilities::buffers::*;
use hidapi::HidDevice;
use std::time::Instant;

/// Responsible for initializing [RobotStatus] and continuously
/// sending status reports
pub struct HidReader {
    pub input: ByteBuffer,
    pub teensy: HidDevice,
    pub layer: HidLayer,
}

impl HidReader {
    pub fn new(layer: HidLayer) -> HidReader {
        HidReader {
            input: ByteBuffer::hid(),
            teensy: layer.device(),
            layer: layer,
        }
    }

    pub fn print(&self) {
        println!(
            "Reader Dump\n\trust time: {}\n\tteensy time: {}",
            self.layer.lifetime(),
            self.layer.mcu_lifetime(),
        );
        self.input.print();
    }

    pub fn buffer(&self) -> ByteBuffer {
        self.input.clone()
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
        // let t = Instant::now();
        match &self.teensy.read(&mut self.input.data) {
            Ok(value) => {
                // reset watchdog... woof
                // println!("Read time: {}", t.elapsed().as_micros());
                self.input.timestamp = Instant::now();
                self.layer.packet_read();
                return *value;
            }
            _ => {
                // *self.shutdown.write().unwrap() = true;
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
        let wait_timer = Instant::now();

        while wait_timer.elapsed().as_millis() < timeout {
            let loopt = Instant::now();

            match self.read() {
                64 => {
                    self.layer.reset_lifetime();

                    if self.input.get(0) == packet_id {
                        self.layer.initialize(true);
                        return;
                    }
                }
                _ => {}
            }

            // HID runs at 1 ms
            self.layer.loop_delay(loopt);
        }

        // If packet never arrives
        self.layer.shutdown();
        println!("HID Reader timed out waiting for reply from Teensy");
    }

    /// Main function to spin and connect the teensys
    /// input to ROS.
    ///
    /// # Usage
    /// ```
    /// use hidapi::HidApi;
    /// use buff_rust::hid_comms::buff_hid::HidReader;
    ///
    /// let mut hidapi = HidApi::new().expect("Failed to create API instance");
    /// let mut reader = HidReader::new(&mut hidapi, vid, pid);
    /// reader.spin();       // runs until watchdog times out
    /// ```
    pub fn spin(&mut self) {
        while !self.layer.is_shutdown() {
            let loopt = Instant::now();

            match self.read() {
                64 => {
                    if self.input.get(0) == 0 && self.input.get_float(60) == 0.0 {
                        continue;
                    } else if self.input.get(0) == 255 {
                        
                    } else if self.input.get_float(60) - self.layer.mcu_lifetime()
                        > TEENSY_CYCLE_TIME_US
                    {
                        println!(
                            "Teensy cycle time is over the limit {}",
                            self.input.get_float(60) - self.layer.mcu_lifetime()
                        );
                    }
                    self.layer.report_parser(&self.input);
                }

                _ => {
                    let timestamp = self.input.timestamp.elapsed();
                    if timestamp.as_secs() > MCU_NO_COMMS_TIMEOUT {
                        println!("HID Reader watchdog called for shutdown");
                        self.layer.shutdown();
                    } else if timestamp.as_millis() > MCU_NO_COMMS_RESET {
                        if !self.layer.is_connected() {
                            // watchdog... woof
                            // writing also failed, try to re-init
                            println!("HID Reader attempting to reconnect");
                            self.teensy = self.layer.device();
                        }
                    } else if timestamp.as_millis() > TEENSY_CYCLE_TIME_MS as u128
                        && timestamp.as_millis() % 10 == 0
                    {
                        println!(
                            "HID Reader: No reply from Teensy for {} ms",
                            timestamp.as_millis()
                        );
                    }
                }
            }

            // if loopt.elapsed().as_micros() > TEENSY_CYCLE_TIME_US as u128 {
            //     println!("HID Reader over cycled {}", loopt.elapsed().as_micros());
            // }
            self.layer.loop_delay(loopt);
        }
    }

    /// Sends robot status report packet to [HidROS], waits for the reply packet,
    /// then calls [HidReader::spin] to begin parsing reports
    ///
    /// # Example
    ///
    /// see [HidLayer::pipeline()]
    pub fn pipeline(&mut self) {
        println!("HID-reader Live");

        // wait for initializers reply
        self.wait_for_report_reply(255, 5000);

        self.spin();

        println!("HID-reader Shutdown");
    }
}
