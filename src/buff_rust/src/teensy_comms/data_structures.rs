use crate::utilities::{data_structures::*, loaders::*};
use std::sync::{Arc, RwLock};

pub static MAX_PROCESS_IO: usize = 14;
pub static MAX_CONFIG_SIZE: usize = 10;
pub static MAX_PROCESS_STATES: usize = 10;
pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;
pub static P: u8 = 0x50;
pub static W: u8 = 0x57;
pub static M: u8 = 0x4D;
pub static D: u8 = 0x44;
pub static I: u8 = 0x4C;
pub static C: u8 = 0x53;
pub static DELIM: u8 = 0x3A;

// first HID value
pub static INIT_REPORT_ID: u8 = 255;
pub static PROC_REPORT_ID: u8 = 1;
pub static MOTOR_REPORT_ID: u8 = 2;
pub static SENSOR_REPORT_ID: u8 = 3;

// second HID value
pub static PROC_INIT_MODE: u8 = 0; // only use with INIT_REPORT_ID
pub static MOTOR_INIT_MODE: u8 = 1;
pub static SENSOR_INIT_MODE: u8 = 2;

pub static READ_MODE: u8 = 0; // only use with MOTOR/SENSOR_REPORT_ID
pub static WRITE_MODE: u8 = 1;

pub static READ_INPUT_MODE: u8 = 0; // only use with PROC_REPORT_ID
pub static READ_STATE_MODE: u8 = 1; // only use with PROC_REPORT_ID
pub static READ_OUTPUT_MODE: u8 = 2;
pub static WRITE_INPUT_MODE: u8 = 3;
pub static WRITE_STATE_MODE: u8 = 4;
pub static WRITE_OUTPUT_MODE: u8 = 5;

//

pub static INIT_MEM_MODE: u8 = 0; // only use with INIT_REPORT_ID
pub static INIT_DRIVER_MODE: u8 = 1;
pub static INIT_CONNECT_MODE: u8 = 2;

pub struct RobotStatus {
    pub motors: Vec<Arc<RwLock<EmbeddedDevice>>>,
    pub sensors: Vec<Arc<RwLock<EmbeddedDevice>>>,
    pub processes: Vec<Arc<RwLock<EmbeddedProcess>>>,
}

impl RobotStatus {
    pub fn from_byu(byu: BuffYamlUtil) -> RobotStatus {
        let motors = byu
            .load_motors()
            .into_iter()
            .map(|x| Arc::new(RwLock::new(x)))
            .collect::<Vec<Arc<RwLock<EmbeddedDevice>>>>();
        let sensors = byu
            .load_sensors()
            .into_iter()
            .map(|x| Arc::new(RwLock::new(x)))
            .collect::<Vec<Arc<RwLock<EmbeddedDevice>>>>();
        let processes = byu
            .load_processes()
            .into_iter()
            .map(|x| Arc::new(RwLock::new(x)))
            .collect::<Vec<Arc<RwLock<EmbeddedProcess>>>>();

        RobotStatus {
            motors: motors,
            sensors: sensors,
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
            motors: self.motors.clone(),
            sensors: self.sensors.clone(),
            processes: self.processes.clone(),
        }
    }

    pub fn system_init_packet_compiler(
        mode: u8,
        length: usize,
        mut mem: Vec<u8>,
        mut drivers: Vec<u8>,
    ) -> (Vec<u8>, Vec<u8>) {
        let mut mem_packet = vec![INIT_REPORT_ID, mode, INIT_MEM_MODE, length as u8];
        mem_packet.append(&mut mem);

        let mut driver_packet = vec![INIT_REPORT_ID, mode, INIT_DRIVER_MODE];
        driver_packet.append(&mut drivers);

        (mem_packet, driver_packet)
    }

    pub fn find_item(&self, name: &String) -> Vec<u8> {
        let mut result = vec![];

        match self
            .sensors
            .iter()
            .rposition(|sensor| sensor.read().unwrap().name() == *name)
            .unwrap_or(255)
        {
            255 => {}
            i => {
                result.push(D);
                result.push(i as u8);
            }
        }

        match self
            .processes
            .iter()
            .rposition(|proc| proc.read().unwrap().name() == *name)
            .unwrap_or(255)
        {
            255 => {}
            i => {
                result.push(P);
                result.push(i as u8);
            }
        }

        match self
            .motors
            .iter()
            .rposition(|motor| motor.read().unwrap().name() == *name)
            .unwrap_or(255)
        {
            255 => {}
            i => {
                result.push(M);
                result.push(i as u8);
            }
        }

        result
    }

    pub fn compile_connections(&self, index: usize) -> Vec<u8> {
        let mut results: Vec<u8> = self.processes[index]
            .read()
            .unwrap()
            .input_names()
            .iter()
            .map(|input| self.find_item(input))
            .flatten()
            .collect();

        results.push(DELIM);

        results.append(
            &mut self.processes[index]
                .read()
                .unwrap()
                .output_names()
                .iter()
                .map(|output| self.find_item(output))
                .flatten()
                .collect(),
        );

        results
    }

    pub fn connections(&mut self) -> Vec<Vec<u8>> {
        (0..self.processes.len())
            .map(|i| {
                [INIT_REPORT_ID, PROC_INIT_MODE, INIT_CONNECT_MODE]
                    .into_iter()
                    .chain(self.compile_connections(i).into_iter())
                    .collect()
            })
            .collect()
    }

    pub fn system_init_packets(&mut self) -> Vec<Vec<u8>> {
        let (proc_mem_init, proc_drv_init) = RobotStatus::system_init_packet_compiler(
            PROC_INIT_MODE,
            self.processes.len(),
            self.processes
                .iter()
                .map(|proc| proc.read().unwrap().n_outputs() as u8)
                .collect(),
            self.processes
                .iter()
                .map(|proc| proc.read().unwrap().driver())
                .flatten()
                .collect(),
        );
        let (motor_mem_init, motor_drv_init) = RobotStatus::system_init_packet_compiler(
            MOTOR_INIT_MODE,
            self.motors.len(),
            self.motors
                .iter()
                .map(|motor| motor.read().unwrap().shape() as u8)
                .collect(),
            self.motors
                .iter()
                .map(|motor| motor.read().unwrap().driver())
                .flatten()
                .collect(),
        );
        let (sensor_mem_init, sensor_drv_init) = RobotStatus::system_init_packet_compiler(
            SENSOR_INIT_MODE,
            self.sensors.len(),
            self.sensors
                .iter()
                .map(|sensor| sensor.read().unwrap().shape() as u8)
                .collect(),
            self.sensors
                .iter()
                .map(|sensor| sensor.read().unwrap().driver())
                .flatten()
                .collect(),
        );

        vec![
            proc_mem_init,
            proc_drv_init,
            motor_mem_init,
            motor_drv_init,
            sensor_mem_init,
            sensor_drv_init,
        ]
    }

    pub fn motor_init_packets(&mut self) -> Vec<Vec<u8>> {
        let mut results = vec![];
        self.motors.iter().enumerate().for_each(|(i, motor)| {
            results.append(&mut motor.read().unwrap().get_initializers(
                MOTOR_REPORT_ID,
                INIT_DRIVER_MODE,
                i as u8,
            ))
        });

        results
    }

    pub fn sensor_init_packets(&mut self) -> Vec<Vec<u8>> {
        let mut results = vec![];
        self.sensors.iter().enumerate().for_each(|(i, sensor)| {
            results.append(&mut sensor.read().unwrap().get_initializers(
                SENSOR_REPORT_ID,
                INIT_DRIVER_MODE,
                i as u8,
            ))
        });

        results
    }

    pub fn process_init_packets(&mut self) -> Vec<Vec<u8>> {
        let mut results = vec![];
        self.processes.iter().enumerate().for_each(|(i, process)| {
            results.append(&mut process.read().unwrap().get_initializers(
                PROC_REPORT_ID,
                INIT_DRIVER_MODE,
                i as u8,
            ))
        });

        results
    }

    pub fn get_initializers(&mut self) -> Vec<Vec<u8>> {
        let mut initializers = self.system_init_packets();
        initializers.append(&mut self.connections());
        initializers.append(&mut self.process_init_packets());
        initializers.append(&mut self.motor_init_packets());
        initializers.append(&mut self.sensor_init_packets());

        initializers
    }

    pub fn process_request_packets(&mut self) -> Vec<Vec<u8>> {
        (0..self.processes.len())
            .map(|i| vec![PROC_REPORT_ID, READ_OUTPUT_MODE, i as u8])
            .chain(
                (0..self.processes.len()).map(|i| vec![PROC_REPORT_ID, READ_STATE_MODE, i as u8]),
            )
            .collect()
    }

    pub fn get_reports(&mut self) -> Vec<Vec<u8>> {
        let mut reports = self.process_request_packets();
        reports.append(&mut self.sensor_init_packets());

        reports
    }

    pub fn update_motor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.motors[index]
            .write()
            .unwrap()
            .update(feedback, timestamp);
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.sensors[index]
            .write()
            .unwrap()
            .update(feedback, timestamp);
    }

    pub fn update_proc_input(&mut self, index: usize, data: Vec<f64>, timestamp: f64) {
        self.processes[index]
            .write()
            .unwrap()
            .update_input(data, timestamp);
    }

    pub fn update_proc_output(&mut self, index: usize, data: Vec<f64>, timestamp: f64) {
        self.processes[index]
            .write()
            .unwrap()
            .update_output(data, timestamp);
    }

    pub fn update_proc_state(
        &mut self,
        index: usize,
        length: usize,
        data: Vec<f64>,
        timestamp: f64,
    ) {
        self.processes[index]
            .write()
            .unwrap()
            .update_states(length, data, timestamp);
    }

    pub fn get_motor_names(&self) -> Vec<String> {
        self.motors
            .iter()
            .map(|motor| motor.read().unwrap().name())
            .collect()
    }

    pub fn get_sensor_names(&self) -> Vec<String> {
        self.sensors
            .iter()
            .map(|sensor| sensor.read().unwrap().name())
            .collect()
    }

    pub fn get_process_names(&self) -> Vec<String> {
        self.processes
            .iter()
            .map(|proc| proc.read().unwrap().name())
            .collect()
    }
}
