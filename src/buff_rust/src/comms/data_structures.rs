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
// use gnuplot::{Caption, Color, Figure};
use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    time::Instant,
};

/// helpful constants to use
pub static P: u8 = 0x50;
pub static W: u8 = 0x57;
pub static M: u8 = 0x4D;
pub static D: u8 = 0x44;
pub static I: u8 = 0x4C;
pub static C: u8 = 0x53;
pub static DELIM: u8 = 0x3A;

/// HID laws
pub static MAX_HID_FLOAT_DATA: usize = 13;
pub static MAX_TASK_PARAMETERS: usize = 130;

/// first HID identifier
/// determines which report handler to use
/// # Usage
/// '''
///     packet.put(0, REPORT_ID);
/// '''
pub static INIT_REPORT_ID: u8 = 255; // Initialize
pub static TASK_CONTROL_ID: u8 = 1; // Request

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
///     packet.put(0, TASK_CONTROL_ID);
///     packet.put(1, REPORT_MODE);
/// '''
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
    pub record: u16,
    pub publish: u16,
    pub latch: u16,

    pub output: Vec<Vec<f64>>,
    pub parameters: Vec<f64>,

    pub input_names: Vec<String>,
    pub output_names: Vec<String>,

    pub run_time: f64,
    pub timestamp: Vec<f64>,

    pub csvu: Option<CsvUtil>,
    pub record_timer: Instant,

    pub publisher: Option<rosrust::Publisher<std_msgs::Float64MultiArray>>,
    pub publish_timer: Instant,
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
        record: u16,
        publish: u16,
    ) -> EmbeddedTask {
        let publisher = match publish > 0 {
            true => Some(rosrust::publish(&name, 1).unwrap()),
            _ => None,
        };

        let csvu = match record > 0 {
            true => Some(CsvUtil::new(
                format!("{}/output", name.clone()).as_str(),
                vec![],
            )),
            _ => None,
        };

        EmbeddedTask {
            name: name.clone(),
            driver: driver,
            record: record,
            publish: publish,
            latch: 0,

            output: vec![],
            parameters: config,

            input_names: input_names,
            output_names: output_names,

            run_time: 0.0,
            timestamp: vec![0.0],

            csvu: csvu,
            record_timer: Instant::now(),

            publisher: publisher,
            publish_timer: Instant::now(),
        }
    }

    pub fn default() -> EmbeddedTask {
        EmbeddedTask::named(
            "UNKNOWN".to_string(),
            "UET".to_string(),
            vec![],
            vec![],
            vec![],
            0,
            0,
        )
    }

    pub fn set_config(&mut self, config: Vec<f64>) {
        if config.len() > MAX_TASK_PARAMETERS {
            panic!(
                "{:?} configuration exceeds maximum parameters ({})",
                self.name, MAX_TASK_PARAMETERS
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

    pub fn update_output(&mut self, latch: u8, data: Vec<f64>, time: f64, run_time: f64) {
        // println!("New output {:?}: {} {:?}", self.name, length, data);
        self.latch = latch as u16;
        self.run_time = run_time;

        if data.len() > 0 {
            self.output.push(data.to_vec());
            self.timestamp.push(time);

            if self.output.len() > 500 {
                self.output.remove(0);
                self.timestamp.remove(0);
            }

            match self.csvu {
                Some(_) => {
                    self.save();
                }
                _ => {}
            }

            match self.publisher {
                Some(_) => {
                    self.publish();
                }
                _ => {}
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
                self.timestamp.last().unwrap(),
                self.output.last().unwrap()
            );
        }
        println!("\tparameters:\n\t\t{:?}", self.parameters);
    }

    pub fn load(&mut self, run: u16, fc: u16) {
        self.output.clear();
        self.timestamp.clear();
        self.csvu = Some(CsvUtil::new(
            format!("{}/output", self.name.clone()).as_str(),
            vec![],
        ));
        self.csvu
            .as_mut()
            .unwrap()
            .load_run(run, fc)
            .iter()
            .for_each(|c| {
                self.output.push(c[0..c.len() - 1].to_vec());
                self.timestamp.push(*c.last().unwrap());
            });
    }

    pub fn save(&mut self) {
        if (self.record_timer.elapsed().as_millis() as f64) < 1E3 / (self.record as f64) {
            return;
        }
        self.record_timer = Instant::now();
        let csvu = self.csvu.as_mut().unwrap();
        if csvu.record_count == 0 {
            csvu.new_labels(EmbeddedTask::make_labels(
                self.output_names[0].clone(),
                self.output.last().unwrap().len(),
            ));
        }

        csvu.save(self.output.last().unwrap(), *self.timestamp.last().unwrap());
        self.record_timer = Instant::now();
    }

    pub fn publish(&mut self) {
        if (self.publish_timer.elapsed().as_millis() as f64) < 1E3 / (self.publish as f64) {
            return;
        }
        self.publish_timer = Instant::now();
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.output.last().unwrap().to_vec();
        msg.data.push(*self.timestamp.last().unwrap());
        self.publisher.as_ref().unwrap().send(msg).unwrap();
    }

    // pub fn display(&mut self) {
    //     if (self.display_timer.elapsed().as_millis() as f64) * 1E-3 < 1.0 / (self.record as f64) {
    //         return;
    //     }

    //     if self.output.len() > 0 {
    //         let len = self.output[0].len();
    //         let mut iters: Vec<_> = self
    //             .output
    //             .clone()
    //             .into_iter()
    //             .map(|n| n.into_iter())
    //             .collect();
    //         let transpose: Vec<Vec<f64>> = (0..len)
    //             .map(|_| {
    //                 iters
    //                     .iter_mut()
    //                     .map(|n| n.next().unwrap())
    //                     .collect::<Vec<f64>>()
    //             })
    //             .collect();

    //         let fig = self.fig.as_mut().unwrap();
    //         fig.clear_axes();
    //         fig.axes2d().lines(
    //             &self.timestamp,
    //             &transpose[0],
    //             &[
    //                 Caption(format!("{}/output0", self.name).as_str()),
    //                 Color("black"),
    //             ],
    //         );
    //         fig.show_and_keep_running().unwrap();
    //         self.display_timer = Instant::now();
    //     }
    // }
}

pub fn get_output_overwrite_latch(i: u8, data: Vec<f64>) -> ByteBuffer {
    let mut buffer = ByteBuffer::hid();
    buffer.puts(0, vec![TASK_CONTROL_ID, i, 1, data.len() as u8]);
    buffer.put_floats(4, data);
    buffer
}

pub struct RobotFirmware {
    pub tasks: Vec<EmbeddedTask>,
    pub task_subs: Vec<rosrust::Subscriber>,
}

impl RobotFirmware {
    pub fn from_byu(
        byu: BuffYamlUtil,
        writer_tx: crossbeam_channel::Sender<ByteBuffer>,
    ) -> RobotFirmware {
        rosrust::init("dyse_comms");

        let tasks = byu.load_tasks();

        let task_subs = (0..tasks.len())
            .map(|i| {
                let writer_clone = writer_tx.clone();
                rosrust::subscribe(
                    &format!("{}_ctrl", tasks[i].name),
                    1,
                    move |msg: std_msgs::Float64MultiArray| {
                        writer_clone
                            .send(get_output_overwrite_latch(i as u8, msg.data))
                            .unwrap();
                    },
                )
                .unwrap()
            })
            .collect();

        RobotFirmware {
            tasks: tasks,
            task_subs: task_subs,
        }
    }

    pub fn default(writer_tx: crossbeam_channel::Sender<ByteBuffer>) -> RobotFirmware {
        let byu = BuffYamlUtil::default();
        RobotFirmware::from_byu(byu, writer_tx)
    }

    pub fn new(
        robot_name: &str,
        writer_tx: crossbeam_channel::Sender<ByteBuffer>,
    ) -> RobotFirmware {
        let byu = BuffYamlUtil::new(robot_name);
        RobotFirmware::from_byu(byu, writer_tx)
    }

    pub fn from_self(writer_tx: crossbeam_channel::Sender<ByteBuffer>) -> RobotFirmware {
        let byu = BuffYamlUtil::from_self();
        RobotFirmware::from_byu(byu, writer_tx)
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
            .chunks(MAX_HID_FLOAT_DATA)
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

    pub fn get_task_names(&self) -> Vec<&String> {
        self.tasks.iter().map(|task| &task.name).collect()
    }

    pub fn parse_hid_feedback(&mut self, report: ByteBuffer, mcu_stats: &HidStats) {
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
        } else if rid == TASK_CONTROL_ID && self.tasks.len() > report.get(1) as usize {
            self.tasks[report.get(1) as usize].update_output(
                report.get(2),
                report.get_floats(4, report.get(3) as usize),
                mcu_lifetime,
                report.get_float(56),
            );

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

    // pub fn display(&mut self) {
    //     (0..self.tasks.len()).for_each(|i| {
    //         self.tasks[i].display();
    //     });
    // }

    // pub fn display_run(&mut self, run: u16, fc: u16) {
    //     self.load_run(run, fc);
    //     self.display();
    // }
}
