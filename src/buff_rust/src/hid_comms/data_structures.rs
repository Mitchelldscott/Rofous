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

use crate::utilities::{data_structures::*, loaders::*};
use std::sync::{Arc, RwLock};

/// helpful constants to use
pub static P: u8 = 0x50;
pub static W: u8 = 0x57;
pub static M: u8 = 0x4D;
pub static D: u8 = 0x44;
pub static I: u8 = 0x4C;
pub static C: u8 = 0x53;
pub static DELIM: u8 = 0x3A;

/// HID laws
pub static MAX_FLOAT_DATA: usize = 14;
pub static MAX_PARAMETERS: usize = 140;

/// first HID identifier
/// determines which report handler to use
/// # Usage
/// '''
///     packet.put(0, REPORT_ID);
/// '''
pub static INIT_REPORT_ID: u8 = 255; // Initialize
pub static TASK_REPORT_ID: u8 = 1; // Request

/// second HID identifier for initializing
/// specifies the report hanlder mode
/// # Usage
/// '''
///     packet.put(0, INIT_REPORT_ID);
///     packet.put(1, REPORT_MODE);
/// '''
pub static INIT_NODE_MODE: u8 = 1;
pub static SETUP_CONFIG_MODE: u8 = 2;

/// second HID identifier for requests
/// specifies the report hanlder mode
/// # Usage
/// '''
///     packet.put(0, TASK_REPORT_ID);
///     packet.put(1, REPORT_MODE);
/// '''
pub static READ_CONTEXT_MODE: u8 = 1;
pub static READ_OUTPUT_MODE: u8 = 2;
pub static WRITE_INPUT_MODE: u8 = 3;
pub static WRITE_CONTEXT_MODE: u8 = 4;
pub static WRITE_OUTPUT_MODE: u8 = 5;

#[derive(Clone)]
pub struct HidStats {
    lifetime: Arc<RwLock<f64>>,
    packets_sent: Arc<RwLock<f64>>,
    packets_read: Arc<RwLock<f64>>,
}

impl HidStats {
    pub fn new() -> HidStats {
        HidStats {
            lifetime: Arc::new(RwLock::new(0.0)),
            packets_sent: Arc::new(RwLock::new(0.0)),
            packets_read: Arc::new(RwLock::new(0.0)),
        }
    }

    pub fn lifetime(&self) -> f64 {
        *self.lifetime.read().unwrap()
    }

    pub fn set_lifetime(&self, t: f64) {
        *self.lifetime.write().unwrap() = t;
    }

    pub fn packets_sent(&self) -> f64 {
        *self.packets_sent.read().unwrap()
    }

    pub fn update_packets_sent(&self, n: f64) {
        *self.packets_sent.write().unwrap() += n;
    }

    pub fn set_packets_sent(&self, n: f64) {
        *self.packets_sent.write().unwrap() = n;
    }

    pub fn packets_read(&self) -> f64 {
        *self.packets_read.read().unwrap()
    }

    pub fn update_packets_read(&self, n: f64) {
        *self.packets_read.write().unwrap() += n;
    }

    pub fn set_packets_read(&self, n: f64) {
        *self.packets_read.write().unwrap() = n;
    }
}

#[derive(Clone)]
pub struct HidControlFlags {
    // Logic flags to cause events in other threads
    shutdown: Arc<RwLock<bool>>,
    connected: Arc<RwLock<bool>>,
    initialized: Arc<RwLock<bool>>,
}

impl HidControlFlags {
    pub fn new() -> HidControlFlags {
        HidControlFlags {
            shutdown: Arc::new(RwLock::new(false)),
            connected: Arc::new(RwLock::new(false)),
            initialized: Arc::new(RwLock::new(false)),
        }
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
}

#[derive(Clone)]
pub struct EmbeddedTask {
    pub name: String,
    pub driver: String,

    pub output: Vec<f64>,
    pub context: Vec<f64>,
    pub parameters: Vec<f64>,

    pub input_names: Vec<String>,
    pub output_names: Vec<String>,

    pub output_timestamp: f64,
    pub context_timestamp: f64,
}

impl EmbeddedTask {
    pub fn default() -> EmbeddedTask {
        EmbeddedTask {
            name: "UED".to_string(),
            driver: "UNKNOWN".to_string(),
            output: vec![],
            context: vec![],
            parameters: vec![],
            input_names: vec![],
            output_names: vec![],
            output_timestamp: 0.0,
            context_timestamp: 0.0,
        }
    }

    pub fn named(
        name: String,
        driver: String,
        input_names: Vec<String>,
        output_names: Vec<String>,
        config: Vec<f64>,
    ) -> EmbeddedTask {
        EmbeddedTask {
            name: name,
            driver: driver,
            output: vec![],
            context: vec![],
            parameters: config,
            input_names: input_names,
            output_names: output_names,
            output_timestamp: 0.0,
            context_timestamp: 0.0,
        }
    }

    pub fn set_config(&mut self, config: Vec<f64>) {
        if config.len() > MAX_PARAMETERS {
            panic!(
                "{:?} configuration exceeds maximum parameters ({})",
                self.name, MAX_PARAMETERS
            );
        }
        self.parameters = config;
    }

    pub fn driver(&self) -> Vec<u8> {
        self.driver.as_bytes().to_vec()
    }

    pub fn params(&self) -> Vec<u8> {
        self.parameters
            .iter()
            .map(|c| (*c as f32).to_be_bytes().to_vec())
            .flatten()
            .collect()
    }

    pub fn update_output(&mut self, length: usize, data: Vec<f64>, time: f64) {
        // println!("New output {:?}: {} {:?}", self.name, length, data);
        self.output = data[0..length].to_vec();
        self.output_timestamp = time;
    }

    pub fn update_context(&mut self, length: usize, data: Vec<f64>, time: f64) {
        // println!("New context {:?}: {} {:?}", self.name, length, data);
        self.context = data[0..length].to_vec();
        self.context_timestamp = time;
    }

    pub fn print(&self) {
        println!("{:?}: {:?} [{}]", self.name, self.driver, self.timestamp);
        println!("\toutput: [{}]\n\t\t{:?}", self.output_timestamp, self.output);
        println!("\tcontext: [{}]\n\t\t{:?}", self.context_timestamp, self.context);
        println!("\tparameters:\n\t\t{:?}", self.parameters);
    }
}

#[derive(Clone)]
pub struct RobotFirmware {
    pub tasks: Vec<EmbeddedTask>,
}

impl RobotFirmware {
    pub fn from_byu(byu: BuffYamlUtil) -> RobotFirmware {
        RobotFirmware {
            tasks: byu.load_tasks(),
        }
    }

    pub fn default() -> RobotFirmware {
        let byu = BuffYamlUtil::default();
        RobotFirmware::from_byu(byu)
    }

    pub fn new(robot_name: &str) -> RobotFirmware {
        if robot_name == "Default" {
            RobotFirmware::default()
        } else {
            let byu = BuffYamlUtil::new(robot_name);
            RobotFirmware::from_byu(byu)
        }
    }

    pub fn from_self() -> RobotFirmware {
        let byu = BuffYamlUtil::from_self();
        RobotFirmware::from_byu(byu)
    }

    pub fn find_ids(&self, names: &Vec<String>) -> Vec<u8> {
        let mut result = vec![];
        self.tasks.iter().enumerate().for_each(|(i, task)| {
            names.iter().for_each(|name| {
                task.output_names.iter().for_each(|n| {
                    if n == name {
                        result.push(i as u8);
                    }
                })
            })
        });

        result
    }

    pub fn get_task_initializers(
        &self,
        id: u8,
        driver: Vec<u8>,
        parameters: &Vec<f64>,
        input_names: &Vec<String>,
    ) -> Vec<ByteBuffer> {
        let mut reports = vec![];
        let input_ids = self.find_ids(input_names);

        // please clean this up, super not rusty

        let mut node_init = ByteBuffer::hid();
        node_init.puts(0, vec![INIT_REPORT_ID, INIT_NODE_MODE, id]);
        node_init.puts(3, driver);
        node_init.put(6, input_ids.len() as u8);
        node_init.puts(7, input_ids);
        reports.push(node_init);

        parameters
            .chunks(MAX_FLOAT_DATA)
            .enumerate()
            .for_each(|(i, chunk)| {
                let mut buffer = ByteBuffer::hid();
                buffer.puts(
                    0,
                    vec![
                        INIT_REPORT_ID,
                        SETUP_CONFIG_MODE,
                        id,
                        i as u8,
                        chunk.len() as u8,
                    ],
                );

                buffer.puts(
                    5,
                    chunk
                        .into_iter()
                        .map(|x| (*x as f32).to_le_bytes())
                        .flatten()
                        .collect(),
                );

                reports.push(buffer);
            });

        reports
    }

    pub fn task_init_packets(&self) -> Vec<ByteBuffer> {
        let mut results = vec![];
        self.tasks.iter().enumerate().for_each(|(i, task)| {
            results.append(&mut self.get_task_initializers(
                i as u8,
                task.driver(),
                &task.parameters,
                &task.input_names,
            ));
        });

        results
    }

    pub fn get_request(&self, i: u8) -> ByteBuffer {
        let mut buffer = ByteBuffer::hid();
        buffer.puts(0, vec![TASK_REPORT_ID, READ_CONTEXT_MODE + (i % 2), i / 2]);
        buffer
    }

    pub fn get_task_names(&self) -> Vec<&String> {
        self.tasks.iter().map(|task| &task.name).collect()
    }

    pub fn parse_request_feedback(&mut self, report: ByteBuffer, mcu_stats: &HidStats) {
        let rtype = report.get(0);
        let mode = report.get(1);
        let mcu_lifetime = report.get_float(60);
        let prev_mcu_lifetime = mcu_stats.lifetime();

        let lifetime_diff = mcu_lifetime - prev_mcu_lifetime;
        if lifetime_diff >= 0.00075 {
            println!("MCU Lifetime jump: {}", lifetime_diff);
        }

        if rtype == INIT_REPORT_ID {
            if mode == INIT_REPORT_ID {
                let prev_mcu_write_count = mcu_stats.packets_sent();

                let mcu_write_count = report.get_float(2);
                let packet_diff = mcu_write_count - prev_mcu_write_count;
                if packet_diff > 1.0 {
                    println!("MCU Packet write difference: {}", packet_diff);
                }

                mcu_stats.set_packets_sent(mcu_write_count);
                mcu_stats.set_packets_read(report.get_float(6));
            }
        } else if rtype == TASK_REPORT_ID {
            if mode == READ_CONTEXT_MODE {
                self.tasks[report.get(2) as usize].update_context(
                    report.get(3) as usize,
                    report.get_floats(4, MAX_FLOAT_DATA),
                    mcu_lifetime,
                );
            }

            if mode == READ_OUTPUT_MODE {
                self.tasks[report.get(2) as usize].update_output(
                    report.get(3) as usize,
                    report.get_floats(4, MAX_FLOAT_DATA),
                    mcu_lifetime,
                );
            }
            mcu_stats.update_packets_sent(1.0); // only works if we don't miss packets
            mcu_stats.update_packets_read(1.0);
        }

        mcu_stats.set_lifetime(mcu_lifetime);
    }

    pub fn print(&self) {
        self.tasks.iter().enumerate().for_each(|(i, task)| {
            println!("===== Task [{}] =====", i);
            task.print();
        });
    }
}
