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
use gnuplot::{Caption, Color, Figure};
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
pub static MAX_FLOAT_DATA: usize = 13;
pub static MAX_PARAMETERS: usize = 130;

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
pub static CONTEXT_MODE: u8 = 1;
pub static OUTPUT_MODE: u8 = 2;

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

    pub fn update_lifetime(&self, t: f64) {
        *self.lifetime.write().unwrap() += t;
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

    pub fn print(&self) {
        println!(
            "\t\tLifetime: {}\n\t\tPackets sent: {}\n\t\tPackets read: {}",
            self.lifetime(),
            self.packets_sent(),
            self.packets_read()
        );
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

    pub fn print(&self) {
        println!(
            "\tShutdown: {}\n\tConnected: {}\n\tInitialized: {}",
            self.is_shutdown(),
            self.is_connected(),
            self.is_initialized()
        );
    }
}

pub struct EmbeddedTask {
    pub name: String,
    pub driver: String,
    pub record: bool,
    pub display: bool,

    pub output: Vec<Vec<f64>>,
    pub context: Vec<Vec<f64>>,
    pub parameters: Vec<f64>,

    pub input_names: Vec<String>,
    pub output_names: Vec<String>,

    pub run_time: f64,
    pub output_timestamp: Vec<f64>,
    pub context_timestamp: Vec<f64>,

    pub output_csvu: CsvUtil,
    pub context_csvu: CsvUtil,

    pub fig: Option<Figure>,
}

impl EmbeddedTask {
    pub fn make_labels(vector_label: String, length: usize) -> Vec<String> {
        if length > 0 {
            (0..length)
                .map(|i| format!("{}[{}]", vector_label, i))
                .chain(["timestamp".to_string()])
                .collect()
        } else {
            vec![]
        }
    }

    pub fn named(
        name: String,
        driver: String,
        input_names: Vec<String>,
        output_names: Vec<String>,
        config: Vec<f64>,
        record: bool,
        display: bool,
    ) -> EmbeddedTask {
        let output_csvu = CsvUtil::new(format!("{}/output", name.clone()).as_str(), vec![]);
        let context_csvu = CsvUtil::new(format!("{}/context", name).as_str(), vec![]);

        let fig;
        match display {
            true => {
                fig = Some(Figure::new());
            }
            _ => {
                fig = None;
            }
        }

        EmbeddedTask {
            name: name.clone(),
            driver: driver,
            record: record,
            display: display,

            output: vec![],
            context: vec![],
            parameters: config,

            input_names: input_names,
            output_names: output_names,

            run_time: 0.0,
            output_timestamp: vec![0.0],
            context_timestamp: vec![0.0],

            output_csvu: output_csvu,
            context_csvu: context_csvu,

            fig: fig,
        }
    }

    pub fn default() -> EmbeddedTask {
        EmbeddedTask::named(
            "UNKNOWN".to_string(),
            "UET".to_string(),
            vec![],
            vec![],
            vec![],
            false,
            false,
        )
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

    pub fn update_output(&mut self, length: usize, data: Vec<f64>, time: f64, duration: f64) {
        // println!("New output {:?}: {} {:?}", self.name, length, data);
        self.run_time = duration;

        if length > 0 {
            self.output.push(data[0..length].to_vec());
            self.output_timestamp.push(time);
            let labels = EmbeddedTask::make_labels(
                self.output_names[0].clone(),
                self.output.last().unwrap().len(),
            );
            self.output_csvu.save(
                self.output.last().unwrap(),
                *self.output_timestamp.last().unwrap(),
                labels,
                self.record,
            );

            if self.output.len() > 200 {
                self.output.remove(0);
                self.output_timestamp.remove(0);
            }
        }
    }

    pub fn update_context(&mut self, length: usize, data: Vec<f64>, time: f64, duration: f64) {
        // println!("New context {:?}: {} {:?}", self.name, length, data);
        self.run_time = duration;

        if length > 0 {
            self.context.push(data[0..length].to_vec());
            self.context_timestamp.push(time);
            let labels =
                EmbeddedTask::make_labels(self.name.clone(), self.context.last().unwrap().len());
            self.context_csvu.save(
                self.context.last().unwrap(),
                *self.context_timestamp.last().unwrap(),
                labels,
                self.record,
            );

            if self.context.len() > 200 {
                self.context.remove(0);
                self.context_timestamp.remove(0);
            }
        }
    }

    pub fn print(&self) {
        println!(
            "{:?}: {:?} [runtime: {}ms]",
            self.name, self.driver, self.run_time
        );
        if self.output.len() > 0 {
            println!(
                "\toutput: [{}s]\n\t\t{:?}",
                self.output_timestamp.last().unwrap(),
                self.output.last().unwrap()
            );
        }
        if self.context.len() > 0 {
            println!(
                "\tcontext: [{}s]\n\t\t{:?}",
                self.context_timestamp.last().unwrap(),
                self.context.last().unwrap()
            );
        }
        println!("\tparameters:\n\t\t{:?}", self.parameters);
    }

    pub fn load(&mut self, run: u16, fc: u16) {
        self.output.clear();
        self.output_timestamp.clear();
        self.output_csvu.load_run(run, fc).iter().for_each(|c| {
            self.output.push(c[0..c.len() - 1].to_vec());
            self.output_timestamp.push(*c.last().unwrap());
        });

        self.context.clear();
        self.context_timestamp.clear();
        self.context_csvu.load_run(run, fc).iter().for_each(|c| {
            self.context.push(c[0..c.len() - 1].to_vec());
            self.context_timestamp.push(*c.last().unwrap());
        });
    }

    pub fn display(&mut self) {
        if self.display {
            if self.output.len() > 0 {
                let len = self.output[0].len();
                let mut iters: Vec<_> = self
                    .output
                    .clone()
                    .into_iter()
                    .map(|n| n.into_iter())
                    .collect();
                let transpose: Vec<Vec<f64>> = (0..len)
                    .map(|_| {
                        iters
                            .iter_mut()
                            .map(|n| n.next().unwrap())
                            .collect::<Vec<f64>>()
                    })
                    .collect();

                let fig = self.fig.as_mut().unwrap();
                fig.clear_axes();
                fig.axes2d().lines(
                    &self.output_timestamp,
                    &transpose[0],
                    &[
                        Caption(format!("{}/output0", self.name).as_str()),
                        Color("black"),
                    ],
                );
                fig.show_and_keep_running().unwrap();
            }
        }
    }
}

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

    pub fn id_of(&self, name: &str) -> usize {
        self.tasks
            .iter()
            .enumerate()
            .find(|(_, task)| name == task.name)
            .expect("Cannot find index of task")
            .0
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
        buffer.puts(0, vec![TASK_REPORT_ID, CONTEXT_MODE + (i % 2), i / 2]);
        buffer
    }

    pub fn get_context_overwrite_latch(&self, i: u8) -> ByteBuffer {
        if self.tasks[i as usize].context.len() > 0 {
            let mut buffer = ByteBuffer::hid();
            buffer.puts(
                0,
                vec![
                    TASK_REPORT_ID,
                    CONTEXT_MODE,
                    i,
                    1,
                    self.tasks[i as usize].context.len() as u8,
                ],
            );
            buffer
        } else {
            ByteBuffer::hid()
        }
    }

    pub fn get_output_overwrite_latch(&self, i: u8) -> ByteBuffer {
        if self.tasks[i as usize].output.len() > 0 {
            let mut buffer = ByteBuffer::hid();
            buffer.puts(
                0,
                vec![
                    TASK_REPORT_ID,
                    OUTPUT_MODE,
                    i,
                    1,
                    self.tasks[i as usize].output[0].len() as u8,
                ],
            );
            buffer
        } else {
            ByteBuffer::hid()
        }
    }

    pub fn get_task_names(&self) -> Vec<&String> {
        self.tasks.iter().map(|task| &task.name).collect()
    }

    pub fn parse_request_feedback(&mut self, report: ByteBuffer, mcu_stats: &HidStats) {
        let rid = report.get(0);
        let mode = report.get(1);
        let mcu_lifetime = report.get_float(60);
        let prev_mcu_lifetime = mcu_stats.lifetime();
        mcu_stats.set_lifetime(mcu_lifetime);

        let lifetime_diff = mcu_lifetime - prev_mcu_lifetime;
        if lifetime_diff >= 0.004 {
            println!("MCU Lifetime jump: {}", lifetime_diff);
        }

        if rid == INIT_REPORT_ID {
            if mode == INIT_REPORT_ID {
                let mcu_write_count = report.get_float(2);
                // let prev_mcu_write_count = mcu_stats.packets_sent();
                // let packet_diff = mcu_write_count - prev_mcu_write_count;
                // if packet_diff > 1.0 {
                //     println!("MCU Packet write difference: {}", packet_diff);
                // }

                mcu_stats.set_packets_sent(mcu_write_count);
                mcu_stats.set_packets_read(report.get_float(6));
            }
        } else if rid == TASK_REPORT_ID {
            if mode == CONTEXT_MODE {
                self.tasks[report.get(2) as usize].update_context(
                    report.get(3) as usize,
                    report.get_floats(4, MAX_FLOAT_DATA),
                    mcu_lifetime,
                    report.get_float(56),
                );
            }

            if mode == OUTPUT_MODE {
                self.tasks[report.get(2) as usize].update_output(
                    report.get(3) as usize,
                    report.get_floats(4, MAX_FLOAT_DATA),
                    mcu_lifetime,
                    report.get_float(56),
                );
            }

            mcu_stats.update_packets_sent(1.0); // only works if we don't miss packets
            mcu_stats.update_packets_read(1.0);
        }
    }

    pub fn print(&self) {
        println!("[Robot-Firmware]: ");
        self.tasks.iter().enumerate().for_each(|(i, task)| {
            println!("===== Task [{}] =====", i);
            task.print();
        });
    }

    pub fn load_run(&mut self, run: u16, fc: u16) {
        (0..self.tasks.len()).for_each(|i| {
            self.tasks[i].load(run, fc);
        });
    }

    pub fn display(&mut self) {
        (0..self.tasks.len()).for_each(|i| {
            self.tasks[i].display();
        });
    }

    pub fn display_run(&mut self, run: u16, fc: u16) {
        self.load_run(run, fc);
        self.display();
    }
}
