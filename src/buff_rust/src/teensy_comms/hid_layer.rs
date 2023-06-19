extern crate hidapi;

use crate::{teensy_comms::data_structures::*, utilities::buffers::ByteBuffer};
use hidapi::{HidApi, HidDevice};
use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    time::Instant,
};

pub static MCU_NO_COMMS_TIMEOUT: u64 = 5;
pub static MCU_NO_COMMS_RESET: u128 = 100;
pub static MCU_RECONNECT_DELAY: f64 = 5.0 * 1E6;

pub static MAX_FLOATS_PER_REPORT: usize = 12;

pub static TEENSY_CYCLE_TIME_S: f64 = 0.001;
pub static TEENSY_CYCLE_TIME_MS: f64 = TEENSY_CYCLE_TIME_S * 1000.0;
pub static TEENSY_CYCLE_TIME_US: f64 = TEENSY_CYCLE_TIME_MS * 1000.0;

pub static TEENSY_DEFAULT_VID: u16 = 0x16C0;
pub static TEENSY_DEFAULT_PID: u16 = 0x0486;

pub struct HidLayer {
    // Device info for initializing connection
    vid: u16,
    pid: u16,
    hidapi: HidApi,

    // Logic flags to cause events in other threads
    shutdown: Arc<RwLock<bool>>,
    connected: Arc<RwLock<bool>>,
    initialized: Arc<RwLock<bool>>,

    // Layer Statistics
    lifetime: Arc<RwLock<f64>>,
    mcu_lifetime: Arc<RwLock<f64>>,
    packets_sent: Arc<RwLock<f64>>,
    packets_read: Arc<RwLock<f64>>,

    // For sending reports to the writer
    writer_tx: Sender<ByteBuffer>,

    // For storing reply data
    robot_status: RobotStatus,
}

impl HidLayer {
    pub fn default() -> (HidLayer, Receiver<ByteBuffer>) {
        let (writer_tx, writer_rx): (Sender<ByteBuffer>, Receiver<ByteBuffer>) = mpsc::channel();

        (
            HidLayer {
                vid: TEENSY_DEFAULT_VID,
                pid: TEENSY_DEFAULT_PID,
                hidapi: HidApi::new().expect("Failed to create API instance"),

                shutdown: Arc::new(RwLock::new(false)),
                connected: Arc::new(RwLock::new(false)),
                initialized: Arc::new(RwLock::new(false)),

                lifetime: Arc::new(RwLock::new(0.0)),
                mcu_lifetime: Arc::new(RwLock::new(0.0)),
                packets_sent: Arc::new(RwLock::new(0.0)),
                packets_read: Arc::new(RwLock::new(0.0)),

                writer_tx: writer_tx,

                robot_status: RobotStatus::default(),
            },
            writer_rx,
        )
    }

    pub fn new(robot: &str) -> (HidLayer, Receiver<ByteBuffer>) {
        let (writer_tx, writer_rx): (Sender<ByteBuffer>, Receiver<ByteBuffer>) = mpsc::channel();

        (
            HidLayer {
                vid: TEENSY_DEFAULT_VID,
                pid: TEENSY_DEFAULT_PID,
                hidapi: HidApi::new().expect("Failed to create API instance"),

                shutdown: Arc::new(RwLock::new(false)),
                connected: Arc::new(RwLock::new(false)),
                initialized: Arc::new(RwLock::new(false)),

                lifetime: Arc::new(RwLock::new(0.0)),
                mcu_lifetime: Arc::new(RwLock::new(0.0)),
                packets_sent: Arc::new(RwLock::new(0.0)),
                packets_read: Arc::new(RwLock::new(0.0)),

                writer_tx: writer_tx,

                robot_status: RobotStatus::new(robot),
            },
            writer_rx,
        )
    }

    pub fn clone(&self) -> HidLayer {
        HidLayer {
            vid: TEENSY_DEFAULT_VID,
            pid: TEENSY_DEFAULT_PID,
            hidapi: HidApi::new().expect("Failed to create API instance"),

            shutdown: self.shutdown.clone(),
            connected: self.connected.clone(),
            initialized: self.initialized.clone(),

            lifetime: self.lifetime.clone(),
            mcu_lifetime: self.mcu_lifetime.clone(),
            packets_sent: self.packets_sent.clone(),
            packets_read: self.packets_read.clone(),

            writer_tx: self.writer_tx.clone(),

            robot_status: self.robot_status.clone(),
        }
    }

    pub fn device(&self) -> HidDevice {
        match self.hidapi.open(self.vid, self.pid) {
            Ok(dev) => {
                println!("New Device");

                self.connect();
                dev.set_blocking_mode(false).unwrap();
                dev
            }
            Err(_) => {
                if self.is_shutdown() {
                    panic!("Shutdown while searching for Teensy");
                }
                println!("recursing");
                let t = Instant::now();
                while self.delay(t) < MCU_RECONNECT_DELAY {}
                self.device()
            }
        }
    }

    pub fn writer_tx(&self, report: ByteBuffer) {
        self.writer_tx.send(report).unwrap();
    }

    pub fn delay(&self, time: Instant) -> f64 {
        let mut t = time.elapsed().as_micros() as f64;
        while t < TEENSY_CYCLE_TIME_US {
            t = time.elapsed().as_micros() as f64;
        }
        t
    }

    pub fn loop_delay(&self, time: Instant) {
        self.increment_lifetime(self.delay(time) * 1E-6);
    }

    pub fn is_shutdown(&self) -> bool {
        *self.shutdown.read().unwrap()
    }

    pub fn shutdown(&self) {
        *self.shutdown.write().unwrap() = true;
    }

    pub fn startup(&self) {
        *self.shutdown.write().unwrap() = false;
    }

    pub fn is_connected(&self) -> bool {
        *self.connected.read().unwrap()
    }

    pub fn connect(&self) {
        *self.connected.write().unwrap() = true;
    }

    pub fn disconnect(&self) {
        *self.connected.write().unwrap() = false;
    }

    pub fn is_initialized(&self) -> bool {
        *self.initialized.read().unwrap()
    }

    pub fn initialize(&self, status: bool) {
        *self.initialized.write().unwrap() = status;
    }

    pub fn packet_sent(&self) {
        *self.packets_sent.write().unwrap() += 1.0;
    }

    pub fn get_packets_sent(&self) -> f64 {
        *self.packets_sent.read().unwrap()
    }

    pub fn packet_read(&self) {
        *self.packets_read.write().unwrap() += 1.0;
    }

    pub fn get_packets_read(&self) -> f64 {
        *self.packets_read.read().unwrap()
    }

    pub fn reset_lifetime(&self) {
        *self.lifetime.write().unwrap() = 0.0;
        *self.mcu_lifetime.write().unwrap() = 0.0;
    }

    pub fn lifetime(&self) -> f64 {
        *self.lifetime.read().unwrap()
    }

    pub fn increment_lifetime(&self, t: f64) {
        *self.lifetime.write().unwrap() += t;
    }

    pub fn mcu_lifetime(&self) -> f64 {
        *self.mcu_lifetime.read().unwrap()
    }

    pub fn set_mcu_lifetime(&self, t: f64) {
        *self.mcu_lifetime.write().unwrap() = t;
    }

    pub fn robot(&self) -> RobotStatus {
        self.robot_status.clone()
    }

    pub fn report_parser(&mut self, report: &ByteBuffer) {
        self.set_mcu_lifetime(report.get_float(60));
        // println!("Read packet: {} {}", self.input.get(0), self.teensy_lifetime as f64 / 1e6); // as seconds

        // match the report number to determine the structure
        if report.get(0) == PROC_REPORT_ID {
            if report.get(1) == READ_STATE_MODE {
                self.robot_status.update_proc_state(
                    report.get(2) as usize,
                    report.get(3) as usize,
                    report.get_floats(4, MAX_FLOATS_PER_REPORT),
                    self.mcu_lifetime(),
                );
            }
            if report.get(1) == READ_OUTPUT_MODE {
                self.robot_status.update_proc_output(
                    report.get(2) as usize,
                    report.get_floats(3, MAX_FLOATS_PER_REPORT),
                    self.mcu_lifetime(),
                );
            }
        }
    }

    pub fn get_initializers(&self) -> Vec<ByteBuffer> {
        self.robot_status.process_init_packets()
    }

    pub fn get_requests(&self) -> Vec<ByteBuffer> {
        self.robot_status.process_request_packets()
    }

    pub fn print(&self) {
        println!("HIDLayer info");
        println!(
            "Shutdown: {}, Connected: {}, initialized: {}",
            self.is_shutdown(),
            self.is_connected(),
            self.is_initialized()
        );
        println!(
            "Packets sent: {}, Packets read: {}",
            self.get_packets_sent(),
            self.get_packets_read()
        );
        self.robot_status.print();
    }
}
