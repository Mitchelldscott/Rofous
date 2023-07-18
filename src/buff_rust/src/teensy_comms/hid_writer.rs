extern crate hidapi;

use crate::teensy_comms::hid_layer::*;
use crate::utilities::buffers::*;
use hidapi::HidDevice;
use std::{sync::mpsc::Receiver, time::Instant};

pub struct HidWriter {
    writer_rx: Receiver<ByteBuffer>,
    output: ByteBuffer,
    teensy: HidDevice,
    layer: HidLayer,
}

impl HidWriter {
    pub fn new(layer: HidLayer, writer_rx: Receiver<ByteBuffer>) -> HidWriter {
        HidWriter {
            writer_rx: writer_rx,
            output: ByteBuffer::hid(),
            teensy: layer.device(),
            layer: layer,
        }
    }

    pub fn print(&self) {
        println!(
            "Writer Dump\n\ttimer: {} us\n\tpackets: {}",
            self.output.timestamp.elapsed().as_micros(),
            self.layer.get_packets_sent(),
        );
        self.output.print();
    }

    pub fn buffer(&self) -> ByteBuffer {
        self.output.clone()
    }

    pub fn silent_channel_default(&mut self) -> ByteBuffer {
        let mut report = ByteBuffer::hid();
        report.put_float(56, self.layer.get_packets_sent());
        report.put_float(60, self.layer.lifetime());

        report
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
                // *self.connected.unwrap().write() = true;
                self.output.reset();
                self.layer.packet_sent();
            }
            _ => {
                println!("HID Writer error");
                if self.output.timestamp.elapsed().as_millis() > MCU_NO_COMMS_RESET {
                    println!(
                        "HID Writer hasn't written for {} ms",
                        self.output.timestamp.elapsed().as_millis()
                    );
                    if self.layer.is_connected() {
                        println!("HID Writer disconnecting");
                        self.layer.disconnect();
                        self.teensy = self.layer.device();
                    }
                }
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

    /// Continually sends data from [HidROS] (or whatever owns the other end of `writer_rx`) to the teensy.
    ///
    /// # Arguments
    /// * `shutdown` - The function stops when this is true.
    /// Used so that HidLayer threads, all running pipeline() at the same time, can be shutdown at the same time (by passing them the same variable)
    /// * `writer_rx` - Receives the data from [HidROS].
    ///
    /// # Example
    /// See [`HidLayer::pipeline()`] source
    pub fn pipeline(&mut self) {
        println!("HID-writer Live");

        while !self.layer.is_shutdown() {
            let t = Instant::now();

            // let default_report = ;
            self.output = self
                .writer_rx
                .try_recv()
                .unwrap_or(self.silent_channel_default());
            // self.output.print();
            self.write();
            self.layer.delay(t);
        }

        println!("HID-writer Shutdown");
    }
}
