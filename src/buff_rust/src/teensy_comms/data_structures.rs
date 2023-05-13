use crate::utilities::loaders::*;
use std::sync::{Arc, RwLock};

pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;
pub static MAX_PROCESS_IO: usize = 14;
pub static MAX_PROCESS_STATES: usize = 10;
pub static MAX_CONFIG_SIZE: usize = 10;

// first HID value
pub static INITIALIZER_REPORT_ID: u8 = 255;
pub static PROCESS_REPORT_ID: u8 = 1;
pub static CONTROLLER_REPORT_ID: u8 = 2;
pub static SENSOR_REPORT_ID: u8 = 3;

// second HID value
pub static EST_INIT_SWITCH_MODE: u8 = 0;
pub static CONTROLLER_INIT_SWITCH_MODE: u8 = 1;
pub static KINEMATICS_INIT_SWITCH_MODE: u8 = 2;
pub static CONTROLLER_REQUEST_SWITCH_MODE: u8 = 1;

pub static EST_IO_SWITCH_MODE: u8 = 0;
pub static EST_STATE_SWITCH_MODE: u8 = 1;

// third HID value
pub static CONTROLLER_REPORT_REQUEST: u8 = 0;
pub static VEL_ESTIMATE_REPORT_REQUEST: u8 = 1;
pub static POS_ESTIMATE_REPORT_REQUEST: u8 = 2;
pub static MANAGER_REPORT_REQUEST: u8 = 3;

pub struct EmbeddedDevice {
    name: String,
    timestamp: f64,
    data: Vec<f64>,
    config: Vec<u8>,
}

impl EmbeddedDevice {
    pub fn default() -> EmbeddedDevice {
        EmbeddedDevice {
            name: "UED".to_string(),
            timestamp: 0.0,
            data: vec![],
            config: vec![],
        }
    }

    pub fn anonymous(id: u8, data_size: usize, config: Vec<u8>) -> EmbeddedDevice {
        EmbeddedDevice {
            name: format!("embedded_device_{}", id),
            timestamp: 0.0,
            data: vec![0.0; data_size],
            config: config,
        }
    }

    pub fn named(name: String, data_size: usize, config: Vec<u8>) -> EmbeddedDevice {
        EmbeddedDevice {
            name: name,
            timestamp: 0.0,
            data: vec![0.0; data_size],
            config: config,
        }
    }

    pub fn update(&mut self, data: Vec<f64>, time: f64) {
        self.data = data[0..self.data.len()].to_vec();
        self.timestamp = time;
    }

    pub fn print(&self) {
        println!(
            "{}\n\tconfig:\t{:?}\n\tData:\t{}\n\t\t{:?}",
            self.name, self.config, self.timestamp, self.data
        );
    }

    pub fn as_string_vec(&self) -> Vec<String> {
        vec![self.timestamp.to_string()]
            .into_iter()
            .chain(
                self.data
                    .iter()
                    .map(|x| x.to_string())
                    .collect::<Vec<String>>(),
            )
            .collect()
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    pub fn data(&self) -> Vec<f64> {
        self.data.clone()
    }

    pub fn config(&self) -> Vec<u8> {
        self.config.clone()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp.clone()
    }
}

pub struct EmbeddedProcess {
    name: String,
    config: Vec<f64>,
    inputs: Vec<f64>,
    states: Vec<f64>,
    outputs: Vec<f64>,
    timestamp: f64,
}

impl EmbeddedProcess {
    pub fn default() -> EmbeddedProcess {
        EmbeddedProcess {
            name: "UED".to_string(),
            config: vec![],
            inputs: vec![],
            states: vec![],
            outputs: vec![],
            timestamp: 0.0,
        }
    }

    pub fn anonymous(id: u8, dimensions: Vec<u8>) -> EmbeddedProcess {
        EmbeddedProcess {
            name: format!("embedded_controller_{}", id),
            config: vec![],
            inputs: vec![0.0; dimensions[0] as usize],
            states: vec![0.0; dimensions[1] as usize],
            outputs: vec![0.0; dimensions[2] as usize],
            timestamp: 0.0,
        }
    }

    pub fn set_config(&mut self, data: Vec<f64>) {
        if data.len() > MAX_CONFIG_SIZE {
            panic!("No process config can exceed 15 values");
        }
        self.config = data;
    }

    pub fn new(name: String, dimensions: Vec<u8>, data: Vec<f64>) -> EmbeddedProcess {
        // if (dimensions[0] + dimensions[2]) as usize > MAX_PROCESS_IO {
        //     panic!(
        //         "Invalid input/output dimension for process {} {:?}",
        //         name, dimensions
        //     );
        // }

        if dimensions[1] as usize > MAX_PROCESS_STATES {
            panic!(
                "Invalid state dimension for process {} {:?}",
                name, dimensions
            );
        }

        EmbeddedProcess {
            name: name,
            config: data,
            inputs: vec![0.0; dimensions[0] as usize],
            states: vec![0.0; dimensions[1] as usize],
            outputs: vec![0.0; dimensions[2] as usize],
            timestamp: 0.0,
        }
    }

    pub fn config(&self) -> Vec<u8> {
        self.config
            .iter()
            .map(|c| (*c as f32).to_be_bytes().to_vec())
            .flatten()
            .collect()
    }

    pub fn update_io(&mut self, data: Vec<f64>, time: f64) {
        self.inputs = data[0..self.inputs.len()].to_vec();
        self.outputs = data[self.inputs.len()..self.inputs.len() + self.outputs.len()].to_vec();
        self.timestamp = time;
    }

    pub fn update_states(&mut self, data: Vec<f64>, time: f64) {
        self.states = data[0..self.states.len()].to_vec();
        self.timestamp = time;
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    pub fn input(&self) -> Vec<f64> {
        self.inputs.clone()
    }

    pub fn state(&self) -> Vec<f64> {
        self.states.clone()
    }

    pub fn output(&self) -> Vec<f64> {
        self.outputs.clone()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp
    }
}

pub struct RobotStatus {
    pub sensors: Vec<Arc<RwLock<EmbeddedDevice>>>,
    pub estimators: Vec<Arc<RwLock<EmbeddedProcess>>>,
}

impl RobotStatus {
    pub fn default() -> RobotStatus {
        RobotStatus {
            sensors: vec![],
            estimators: vec![],
        }
    }

    pub fn load_sensors(byu: &BuffYamlUtil) -> Vec<Arc<RwLock<EmbeddedDevice>>> {
        let sensor_index = byu.load_string_list("sensor_index");
        let sensor_buffers = byu.load_u8_list("sensor_buffers");
        let sensor_configs = byu.load_float_matrix("sensor_config");

        assert!(
            sensor_index.len() == sensor_buffers.len(),
            "Number of Sensors and Sensor buffers should match"
        );
        assert!(
            sensor_index.len() == sensor_configs.len(),
            "Number of Sensors and Sensor configs should match"
        );

        let sensor_bytes: Vec<Vec<u8>> = sensor_configs
            .into_iter()
            .map(|config| {
                config
                    .iter()
                    .map(|c| (*c as f32).to_be_bytes().to_vec())
                    .flatten()
                    .collect()
            })
            .collect();

        sensor_index
            .into_iter()
            .zip(sensor_buffers.iter())
            .zip(sensor_bytes.iter())
            .map(|((name, buffer), config)| {
                Arc::new(RwLock::new(EmbeddedDevice::named(
                    name,
                    *buffer as usize,
                    config.to_vec(),
                )))
            })
            .collect()
    }

    pub fn load_estimators(byu: &BuffYamlUtil) -> Vec<Arc<RwLock<EmbeddedProcess>>> {
        let estimator_index = byu.load_string_list("estimator_index");
        let estimator_configs = byu.load_float_matrix("estimator_configs");
        let estimator_dimensions = byu.load_integer_matrix("estimator_dimensions");

        assert!(
            estimator_index.len() == estimator_configs.len(),
            "Number of Estimators and Estimator configs should match"
        );
        assert!(
            estimator_index.len() == estimator_dimensions.len(),
            "Number of Estimators and Estimator dimensions should match"
        );

        estimator_index
            .into_iter()
            .zip(estimator_dimensions.into_iter())
            .zip(estimator_configs.into_iter())
            .map(|((name, dimension), config)| {
                Arc::new(RwLock::new(EmbeddedProcess::new(
                    name,
                    dimension,
                    config.to_vec(),
                )))
            })
            .collect()
    }

    pub fn from_byu(byu: BuffYamlUtil) -> RobotStatus {
        let sensors = RobotStatus::load_sensors(&byu);
        let estimators = RobotStatus::load_estimators(&byu);

        RobotStatus {
            sensors: sensors,
            estimators: estimators,
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
            sensors: self.sensors.iter().map(|sensor| sensor.clone()).collect(),
            estimators: self.estimators.iter().map(|est| est.clone()).collect(),
        }
    }

    pub fn sensor_init_packets(&mut self) -> Vec<Vec<u8>> {
        self.sensors
            .iter()
            .enumerate()
            .map(|(i, sensor)| {
                vec![SENSOR_REPORT_ID, i as u8]
                    .into_iter()
                    .chain(sensor.read().unwrap().config())
                    .collect()
            })
            .collect()
    }

    pub fn estimator_init_packets(&mut self) -> Vec<Vec<u8>> {
        self.estimators
            .iter()
            .enumerate()
            .map(|(i, est)| {
                vec![INITIALIZER_REPORT_ID, EST_INIT_SWITCH_MODE, i as u8]
                    .into_iter()
                    .chain(est.read().unwrap().config())
                    .collect()
            })
            .collect()
    }

    pub fn load_initializers(&mut self) -> Vec<Vec<u8>> {
        let mut initializers = vec![];
        self.estimator_init_packets()
            .iter()
            .for_each(|packet| initializers.push(packet.to_vec()));

        self.sensor_init_packets()
            .iter()
            .for_each(|packet| initializers.push(packet.to_vec()));

        initializers
    }

    pub fn estimator_request_packets(&mut self) -> Vec<Vec<u8>> {
        (0..self.estimators.len())
            .map(|i| vec![PROCESS_REPORT_ID, EST_STATE_SWITCH_MODE, i as u8])
            .collect()
    }

    pub fn get_reports(&mut self) -> Vec<Vec<u8>> {
        self.estimator_request_packets()
            .into_iter()
            .chain(self.sensor_init_packets().into_iter())
            .collect()
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.sensors[index]
            .write()
            .unwrap()
            .update(feedback, timestamp);
    }

    pub fn update_est_io(&mut self, index: usize, data: Vec<f64>, timestamp: f64) {
        self.estimators[index]
            .write()
            .unwrap()
            .update_io(data, timestamp);
    }

    pub fn update_est_state(&mut self, index: usize, data: Vec<f64>, timestamp: f64) {
        self.estimators[index]
            .write()
            .unwrap()
            .update_states(data, timestamp);
    }

    pub fn get_sensor_names(&self) -> Vec<String> {
        self.sensors
            .iter()
            .map(|sensor| sensor.read().unwrap().name())
            .collect()
    }

    pub fn get_estimator_names(&self) -> Vec<String> {
        self.estimators
            .iter()
            .map(|est| est.read().unwrap().name())
            .collect()
    }
}
