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
    comms::{data_structures::*, hid_layer::*, hid_reader::*, hid_writer::*},
    utilities::data_structures::*,
};
use std::{sync::mpsc, time::Instant};

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
    pub writer_tx: crossbeam_channel::Sender<ByteBuffer>,
    pub parser_rx: mpsc::Receiver<ByteBuffer>,

    // For storing reply data
    pub robot_fw: RobotFirmware,
}

impl HidInterface {
    pub fn new() -> (HidInterface, HidReader, HidWriter) {
        let layer = HidLayer::new(TEENSY_DEFAULT_VID, TEENSY_DEFAULT_PID, TEENSY_CYCLE_TIME_US);
        let (writer_tx, writer_rx): (
            crossbeam_channel::Sender<ByteBuffer>,
            crossbeam_channel::Receiver<ByteBuffer>,
        ) = crossbeam_channel::bounded(100);
        let (parser_tx, parser_rx): (mpsc::Sender<ByteBuffer>, mpsc::Receiver<ByteBuffer>) =
            mpsc::channel();

        (
            HidInterface {
                layer: layer.clone(),

                current_request: 0,

                writer_tx: writer_tx.clone(),
                parser_rx: parser_rx,

                robot_fw: RobotFirmware::default(writer_tx),
            },
            HidReader::new(layer.clone(), parser_tx),
            HidWriter::new(layer, writer_rx),
        )
    }

    pub fn sim() -> HidInterface {
        let layer = HidLayer::new(TEENSY_DEFAULT_VID, TEENSY_DEFAULT_PID, TEENSY_CYCLE_TIME_US);
        let (writer_tx, _): (
            crossbeam_channel::Sender<ByteBuffer>,
            crossbeam_channel::Receiver<ByteBuffer>,
        ) = crossbeam_channel::bounded(100);
        let (_, parser_rx): (mpsc::Sender<ByteBuffer>, mpsc::Receiver<ByteBuffer>) =
            mpsc::channel();

        HidInterface {
            layer: layer.clone(),

            current_request: 0,

            writer_tx: writer_tx.clone(),
            parser_rx: parser_rx,

            robot_fw: RobotFirmware::default(writer_tx),
        }
    }

    pub fn writer_tx(&self, report: ByteBuffer) {
        self.writer_tx.send(report).unwrap();
    }

    pub fn robot(&self) -> &RobotFirmware {
        &self.robot_fw
    }

    pub fn output(&self, index: usize) -> &Vec<Vec<f64>> {
        &self.robot_fw.tasks[index].output
    }

    pub fn check_feedback(&mut self) {
        match self.parser_rx.try_recv() {
            Ok(report) => {
                self.robot_fw
                    .parse_hid_feedback(report, &self.layer.mcu_stats);
            }
            _ => {}
        }
    }

    pub fn get_initializers(&self) -> Vec<ByteBuffer> {
        self.robot_fw.task_init_packets()
    }

    pub fn send_control(&mut self, i: usize, data: Vec<f64>) {
        self.writer_tx(get_output_overwrite_latch(i as u8, data));
    }

    pub fn print(&self) {
        self.layer.print();
        self.robot_fw.print();
    }

    pub fn pipeline(&mut self, _unused_flag: bool) {
        let mut loop_count = 0;
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

        while !self.layer.control_flags.is_shutdown() && rosrust::is_ok() {
            let loopt = Instant::now();

            if self.layer.control_flags.is_connected() {
                // if loop_count % 100 == 0 {
                //     self.send_control(
                //         self.robot_fw.id_of("servo_motor"),
                //         vec![(loop_count % 256) as f64],
                //     );
                // }
                self.check_feedback();
            }

            loop_count += 1;
            if loop_count > 40000 {
                self.print();
                loop_count = 0;
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
