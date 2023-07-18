use crate::utilities::{buffers::ByteBuffer, data_structures::*, loaders::*};
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
pub static MAX_PROCESS_IO: usize = 14;
pub static MAX_CONFIG_SIZE: usize = 10;
pub static MAX_PROCESS_CONTEXT: usize = 10;
pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;

/// first HID identifier
/// determines which report handler to use
/// # Usage
/// '''
///     packet.put(0, REPORT_ID);
/// '''
pub static INIT_REPORT_ID: u8 = 255; // Initialize
pub static PROC_REPORT_ID: u8 = 1; // Request

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
///     packet.put(0, PROC_REPORT_ID);
///     packet.put(1, REPORT_MODE);
/// '''
pub static READ_CONTEXT_MODE: u8 = 1;
pub static READ_OUTPUT_MODE: u8 = 2;
pub static WRITE_INPUT_MODE: u8 = 3;
pub static WRITE_CONTEXT_MODE: u8 = 4;
pub static WRITE_OUTPUT_MODE: u8 = 5;

#[derive(Clone)]
pub struct RobotStatus {
    pub processes: Vec<Arc<RwLock<EmbeddedProcess>>>,
}

impl RobotStatus {
    pub fn from_byu(byu: BuffYamlUtil) -> RobotStatus {
        let processes = byu
            .load_processes()
            .into_iter()
            .map(|x| Arc::new(RwLock::new(x)))
            .collect::<Vec<Arc<RwLock<EmbeddedProcess>>>>();

        RobotStatus {
            processes: processes,
        }
    }

    pub fn default() -> RobotStatus {
        let byu = BuffYamlUtil::default();
        RobotStatus::from_byu(byu)
    }

    pub fn new(robot_name: &str) -> RobotStatus {
        let byu = BuffYamlUtil::new(robot_name);
        RobotStatus::from_byu(byu)
    }

    pub fn from_self() -> RobotStatus {
        let byu = BuffYamlUtil::from_self();
        RobotStatus::from_byu(byu)
    }

    pub fn clone(&self) -> RobotStatus {
        RobotStatus {
            processes: self.processes.clone(),
        }
    }

    pub fn find_ids(&self, names: &Vec<String>) -> Vec<u8> {
        let mut result = vec![];
        self.processes.iter().enumerate().for_each(|(i, proc)| {
            names.iter().for_each(|name| {
                proc.read().unwrap().output_names().iter().for_each(|n| {
                    if n == name {
                        result.push(i as u8);
                    }
                })
            })
        });

        result
    }

    pub fn process_init_packets(&self) -> Vec<ByteBuffer> {
        let mut results = vec![];
        self.processes.iter().enumerate().for_each(|(i, process)| {
            let input_ids = self.find_ids(&process.read().unwrap().input_names());
            results.append(&mut process.read().unwrap().get_initializers(
                INIT_REPORT_ID,
                INIT_NODE_MODE,
                i as u8,
                input_ids,
            ));
        });

        results
    }

    pub fn process_request_packets(&self) -> Vec<ByteBuffer> {
        let mut results = vec![];
        (0..self.processes.len()).for_each(|i| {
            let mut out_buffer = ByteBuffer::hid();
            out_buffer.puts(0, vec![PROC_REPORT_ID, READ_OUTPUT_MODE, i as u8]);
            results.push(out_buffer);

            let mut context_buffer = ByteBuffer::hid();
            context_buffer.puts(0, vec![PROC_REPORT_ID, READ_CONTEXT_MODE, i as u8]);
            results.push(context_buffer);
        });

        results
    }

    pub fn update_proc_output(
        &mut self,
        index: usize,
        length: usize,
        data: Vec<f64>,
        timestamp: f64,
    ) {
        self.processes[index]
            .write()
            .unwrap()
            .update_output(length, data, timestamp);
    }

    pub fn update_proc_context(
        &mut self,
        index: usize,
        length: usize,
        data: Vec<f64>,
        timestamp: f64,
    ) {
        self.processes[index]
            .write()
            .unwrap()
            .update_context(length, data, timestamp);
    }

    pub fn get_process_names(&self) -> Vec<String> {
        self.processes
            .iter()
            .map(|proc| proc.read().unwrap().name())
            .collect()
    }

    pub fn print(&self) {
        self.processes.iter().enumerate().for_each(|(i, proc)| {
            println!("Processs {}", i);
            proc.read().unwrap().print();
        });
    }
}
