use crate::utilities::{data_structures::*, loaders::*};

pub static MAX_PROCESS_IO: usize = 14;
pub static MAX_CONFIG_SIZE: usize = 10;
pub static MAX_PROCESS_STATES: usize = 10;
pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;
pub static P: u8 = 0x50;
pub static W: u8 = 0x57;
pub static M: u8 = 0x4D;


// first HID value
pub static INIT_REPORT_ID: u8 = 255;
pub static PROC_REPORT_ID: u8 = 1;
pub static MOTOR_REPORT_ID: u8 = 2;
pub static SENSOR_REPORT_ID: u8 = 3;

// second HID value
pub static PROC_INIT_SWITCH_MODE: u8 = 0; // pairs with init ID report always
pub static MOTOR_INIT_SWITCH_MODE: u8 = 1;
pub static SENSOR_INIT_SWITCH_MODE: u8 = 2;

pub static MOTOR_READ_SWITCH_MODE: u8 = 0;
pub static MOTOR_WRITE_SWITCH_MODE: u8 = 1;
pub static SENORS_READ_SWITCH_MODE: u8 = 0;

pub static PROC_READ_INPUT_SWITCH_MODE: u8 = 0;
pub static PROC_READ_OUTPUT_SWITCH_MODE: u8 = 1;
pub static PROC_WRITE_INPUT_SWITCH_MODE: u8 = 2;
pub static PROC_WRITE_OUTPUT_SWITCH_MODE: u8 = 3;

pub struct RobotStatus {
    pub motors: Vec<EmbeddedDevice>,
    pub sensors: Vec<EmbeddedDevice>,
    pub processes: Vec<EmbeddedProcess>,
}

impl RobotStatus {
    pub fn default() -> RobotStatus {
        RobotStatus {
            motors: vec![],
            sensors: vec![],
            processes: vec![],
        }
    }

    pub fn from_byu(byu: BuffYamlUtil) -> RobotStatus {
        let motors = byu.load_motors();
        let sensors = byu.load_sensors();
        let processes = byu.load_processes();

        RobotStatus {
            motors: motors,
            sensors: sensors,
            processes: processes,
        }
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

    pub fn system_init_packets(&mut self) -> Vec<Vec<u8>> {
        // set the number of processes and the length of each's output
        let mut proc_mem = vec![INIT_REPORT_ID, PROC_INIT_SWITCH_MODE, self.processes.len() as u8];
        self.processes.iter().for_each(|proc| {
            proc_mem.push(proc.n_outputs() as u8);
        });

        let mut motor_mem = vec![INIT_REPORT_ID, MOTOR_INIT_SWITCH_MODE, self.motors.len() as u8];
        self.motors.iter().for_each(|motor| {
            motor_mem.push(motor.shape() as u8);
        });

        let mut sensor_mem = vec![INIT_REPORT_ID, SENSOR_INIT_SWITCH_MODE, self.sensors.len() as u8];
        self.sensors.iter().for_each(|sensor| {
            sensor_mem.push(sensor.shape() as u8);
        });

        vec![proc_mem, motor_mem, sensor_mem]
    }

    pub fn motor_init_packets(&mut self) -> Vec<Vec<u8>> {
        self.motors
            .iter()
            .enumerate()
            .map(|(i, motor)| {
                vec![MOTOR_REPORT_ID, i as u8]
                    .into_iter()
                    .chain(motor.config())
                    .collect()
            })
            .collect()
    }

    pub fn sensor_init_packets(&mut self) -> Vec<Vec<u8>> {
        self.sensors
            .iter()
            .enumerate()
            .map(|(i, sensor)| {
                vec![SENSOR_REPORT_ID, i as u8]
                    .into_iter()
                    .chain(sensor.config())
                    .collect()
            })
            .collect()
    }

    pub fn process_init_packets(&mut self) -> Vec<Vec<u8>> {
        self.processes
            .iter()
            .enumerate()
            .map(|(i, proc)| {
                vec![PROC_REPORT_ID, PROC_INIT_SWITCH_MODE, i as u8]
                    .into_iter()
                    .chain(proc.config())
                    .collect()
            })
            .collect()
    }

    pub fn get_initializers(&mut self) -> Vec<Vec<u8>> {
        let mut initializers = self.system_init_packets();
        initializers.append(&mut self.process_init_packets());
        initializers.append(&mut self.motor_init_packets());
        initializers.append(&mut self.sensor_init_packets());

        initializers
    }

    pub fn process_request_packets(&mut self) -> Vec<Vec<u8>> {
        (0..self.processes.len())
            .map(|i| vec![PROC_REPORT_ID, PROC_READ_OUTPUT_SWITCH_MODE, i as u8])
            .collect()
    }

    pub fn get_reports(&mut self) -> Vec<Vec<u8>> {
        self.process_request_packets()
            .into_iter()
            .chain(self.sensor_init_packets().into_iter())
            .collect()
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.sensors[index]
            .update(feedback, timestamp);
    }

    pub fn update_proc_io(&mut self, index: usize, data: Vec<f64>, timestamp: f64) {
        self.processes[index]
            .update_io(data, timestamp);
    }

    pub fn update_proc_state(&mut self, index: usize, length: usize, data: Vec<f64>, timestamp: f64) {
        self.processes[index]
            .update_states(length, data, timestamp);
    }

    pub fn get_sensor_names(&self) -> Vec<String> {
        self.sensors
            .iter()
            .map(|sensor| sensor.name())
            .collect()
    }

    pub fn get_process_names(&self) -> Vec<String> {
        self.processes
            .iter()
            .map(|proc| proc.name())
            .collect()
    }
}
