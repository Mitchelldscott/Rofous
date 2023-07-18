#![allow(unused_imports)]
extern crate hidapi;

use hidapi::{HidApi, HidDevice};

use crate::teensy_comms::{
    data_structures::*, hid_layer::*, hid_reader::*, hid_ros::*, hid_writer::*,
};
use crate::utilities::loaders::*;

use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    thread::{spawn, Builder},
    time::Instant,
};

#[allow(dead_code)]
const VERBOSITY: usize = 1;

#[cfg(test)]
pub mod robot_status_tests {
    use super::*;

    #[test]
    pub fn robot_status_load() {
        let rs = RobotStatus::new("penguin");
        rs.process_init_packets().iter().for_each(|packet| {
            packet.print();
        });

        rs.process_request_packets().iter().for_each(|packet| {
            packet.print();
        })
    }
}

#[cfg(test)]
pub mod teensy_comms_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn sim_control_pipeline(layer: HidLayer) {
        let initializers = layer.get_initializers();

        while !layer.is_connected() {}

        println!("HID-SIM Live");

        initializers.iter().for_each(|init| {
            let t = Instant::now();
            // init.print();
            layer.writer_tx(init.clone());
            layer.delay(t);
        });

        let mut current_report = 0;
        let reports = layer.get_requests();

        let t = Instant::now();

        while t.elapsed().as_secs() < 5 && !layer.is_shutdown() {
            let loopt = Instant::now();

            // println!("{:?}", reports[current_report]);
            if layer.is_connected() {
                layer.writer_tx(reports[current_report].clone());
            }
            current_report = (current_report + 1) % reports.len();

            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }

        layer.shutdown();
        println!("HID Control sim shutdown");
        layer.print();
    }

    #[test]
    pub fn hid_control_test() {
        /*
            Start an hid layer
        */
        let (layer, writer_rx) = HidLayer::new("penguin");
        layer.print();
        let sim_layer = layer.clone();
        let mut hidreader = HidReader::new(layer.clone());
        let mut hidwriter = HidWriter::new(layer, writer_rx);

        let hidreader_handle = Builder::new()
            .name("HID Reader".to_string())
            .spawn(move || {
                hidreader.pipeline();
            })
            .unwrap();

        let hidwriter_handle = Builder::new()
            .name("HID Writer".to_string())
            .spawn(move || {
                hidwriter.pipeline();
            })
            .unwrap();

        let pipeline_sim = Builder::new()
            .name("HID Control".to_string())
            .spawn(move || {
                sim_control_pipeline(sim_layer);
            })
            .unwrap();

        hidreader_handle.join().expect("HID Reader failed");
        pipeline_sim.join().expect("HID Sim failed");
        hidwriter_handle.join().expect("HID Writer failed");
    }
}
