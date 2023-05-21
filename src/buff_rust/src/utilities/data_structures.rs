pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;
pub static MAX_PROCESS_STATES: usize = 10;
pub static MAX_CONFIG_SIZE: usize = 10;
pub static MAX_PROCESS_IO: usize = 14;

#[derive(Clone)]
pub struct EmbeddedDevice {
    name: String,
    driver: String,
    timestamp: f64,
    data: Vec<f64>,
    config: Vec<u8>,
}

impl EmbeddedDevice {
    pub fn default() -> EmbeddedDevice {
        EmbeddedDevice {
            name: "UED".to_string(),
            driver: "UNKNOWN".to_string(),
            timestamp: 0.0,
            data: vec![],
            config: vec![],
        }
    }

    pub fn anonymous(id: u8, data_size: usize, config: Vec<u8>, driver: &str) -> EmbeddedDevice {
        EmbeddedDevice {
            name: format!("embedded_device_{}", id),
            driver: driver.to_string(),
            timestamp: 0.0,
            data: vec![0.0; data_size],
            config: config,
        }
    }

    pub fn named(
        name: String,
        driver: String,
        data_size: usize,
        config: Vec<u8>,
    ) -> EmbeddedDevice {
        EmbeddedDevice {
            name: name,
            driver: driver,
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

    pub fn shape(&self) -> usize {
        self.data.len()
    }

    pub fn config(&self) -> Vec<u8> {
        self.config.clone()
    }

    pub fn driver(&self) -> Vec<u8> {
        self.driver.as_bytes().to_vec()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp.clone()
    }
}

#[derive(Clone)]
pub struct EmbeddedProcess {
    name: String,
    driver: String,
    config: Vec<f64>,
    inputs: Vec<f64>,
    states: Vec<f64>,
    outputs: Vec<f64>,
    input_shapes: Vec<usize>,
    state_shape: usize,
    output_shapes: Vec<usize>,
    timestamp: f64,
}

impl EmbeddedProcess {
    pub fn default() -> EmbeddedProcess {
        EmbeddedProcess {
            name: "UED".to_string(),
            driver: "UNKNOWN".to_string(),
            config: vec![],
            inputs: vec![],
            states: vec![],
            outputs: vec![],
            input_shapes: vec![0],
            state_shape: 0,
            output_shapes: vec![0],
            timestamp: 0.0,
        }
    }

    pub fn set_config(&mut self, config: Vec<f64>) {
        if config.len() > MAX_CONFIG_SIZE {
            panic!("No process config can exceed 15 values");
        }
        self.config = config;
    }

    pub fn named(
        name: String,
        driver: String,
        input_shapes: Vec<usize>,
        output_shapes: Vec<usize>,
        config: Vec<f64>,
    ) -> EmbeddedProcess {
        EmbeddedProcess {
            name: name,
            driver: driver,
            config: config,
            inputs: vec![0.0; input_shapes.iter().sum()],
            states: vec![0.0],
            outputs: vec![0.0; output_shapes.iter().sum()],
            input_shapes: input_shapes,
            state_shape: 0,
            output_shapes: output_shapes,
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
        let n_inputs: usize = self.input_shapes.iter().sum();
        let n_outputs: usize = self.output_shapes.iter().sum();
        self.inputs = data[0..n_inputs].to_vec();
        self.outputs = data[n_inputs..n_inputs + n_outputs].to_vec();
        self.timestamp = time;
    }

    pub fn update_states(&mut self, length: usize, data: Vec<f64>, time: f64) {
        self.state_shape = length;
        self.states = data[0..length].to_vec();
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

    pub fn n_inputs(&self) -> usize {
        self.input_shapes.iter().sum()
    }

    pub fn n_outputs(&self) -> usize {
        self.output_shapes.iter().sum()
    }

    pub fn driver(&self) -> Vec<u8> {
        self.driver.as_bytes().to_vec()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp
    }
}
