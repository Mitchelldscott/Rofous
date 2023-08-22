/********************************************************************************
 *
 *      ____                     ____          __           __       _
 *     / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *    / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *   / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  )
 *  /_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/
 *        /____/
 *
 *
 *
 ********************************************************************************/

extern crate hidapi;

use crate::{
    hid_comms::{data_structures::*, hid_layer::*, hid_reader::*, hid_writer::*},
    utilities::data_structures::*,
};
use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
    },
    time::Instant,
};

pub static MCU_NO_COMMS_TIMEOUT_S: u64 = 10;
pub static MCU_NO_COMMS_RESET_MS: u128 = 10;
pub static MCU_RECONNECT_DELAY_US: f64 = 5.0 * 1E6;

pub static TEENSY_CYCLE_TIME_S: f64 = 0.0005;
pub static TEENSY_CYCLE_TIME_MS: f64 = TEENSY_CYCLE_TIME_S * 1E3;
pub static TEENSY_CYCLE_TIME_US: f64 = TEENSY_CYCLE_TIME_S * 1E6;
pub static TEENSY_CYCLE_TIME_ER: f64 = TEENSY_CYCLE_TIME_US + 50.0; // err threshold (before prints happen)

pub static TEENSY_DEFAULT_VID: u16 = 0x16C0;
pub static TEENSY_DEFAULT_PID: u16 = 0x0486;

pub struct HidInterface {
    pub layer: HidLayer,

    pub current_request: u8,

    // For sending reports to the writer
    pub writer_tx: Sender<ByteBuffer>,
    pub parser_rx: Receiver<ByteBuffer>,

    // For storing reply data
    pub robot_fw: RobotFirmware,
}

impl HidInterface {
    pub fn new(robot_name: &str) -> (HidInterface, HidReader, HidWriter) {
        let layer = HidLayer::new(TEENSY_DEFAULT_VID, TEENSY_DEFAULT_PID, TEENSY_CYCLE_TIME_US);
        let (writer_tx, writer_rx): (Sender<ByteBuffer>, Receiver<ByteBuffer>) = mpsc::channel();
        let (parser_tx, parser_rx): (Sender<ByteBuffer>, Receiver<ByteBuffer>) = mpsc::channel();

        (
            HidInterface {
                layer: layer.clone(),

                current_request: 0,

                writer_tx: writer_tx,
                parser_rx: parser_rx,

                robot_fw: RobotFirmware::new(robot_name),
            },
            HidReader::new(layer.clone(), parser_tx),
            HidWriter::new(layer, writer_rx),
        )
    }

    pub fn writer_tx(&self, report: ByteBuffer) {
        self.writer_tx.send(report).unwrap();
    }

    pub fn robot(&self) -> &RobotFirmware {
        &self.robot_fw
    }

    pub fn output(&self, index: usize) -> &Vec<f64> {
        &self.robot_fw.tasks[index].output
    }

    pub fn context(&self, index: usize) -> &Vec<f64> {
        &self.robot_fw.tasks[index].context
    }

    pub fn check_feedback(&mut self) {
        match self.parser_rx.try_recv() {
            Ok(report) => {
                self.robot_fw
                    .parse_request_feedback(report, &self.layer.mcu_stats);
            }
            _ => {}
        }
    }

    pub fn get_initializers(&self) -> Vec<ByteBuffer> {
        self.robot_fw.task_init_packets()
    }

    pub fn send_request(&mut self) {
        self.writer_tx(self.robot_fw.get_request(self.current_request));
        self.current_request = (self.current_request + 1) % (2 * self.robot_fw.tasks.len() as u8);
    }

    pub fn print(&self) {
        self.robot_fw.print();
    }

    pub fn pipeline(&mut self) {
        let initializers = self.get_initializers();

        while !self.layer.control_flags.is_connected() {}

        println!("[HID-Control]: Live");

        // Not as worried about efficiency here
        // Also it hard to know how many init packets
        // there will be... so K.I.S.S.
        initializers.iter().for_each(|init| {
            let t = Instant::now();
            self.writer_tx(init.clone());
            self.layer.delay(t);
        });

        while !self.layer.control_flags.is_shutdown() {
            let loopt = Instant::now();

            if self.layer.control_flags.is_connected() {
                self.send_request();
                self.check_feedback();
            }

            self.layer.delay(loopt);
            // if t.elapsed().as_micros() as f64 > TEENSY_CYCLE_TIME_US {
            //     println!("HID Control over cycled {} ms", (t.elapsed().as_micros() as f64) * 1E-3);
            // }
        }

        self.layer.control_flags.shutdown();
        println!("[HID-Control]: shutdown");
        self.layer.print();
    }
}
