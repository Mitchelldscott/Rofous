use crate::utilities::buffers::ByteBuffer;

pub static MAX_PROCESS_IO: usize = 10;
pub static MAX_CONFIG_SIZE: usize = 40;
pub static MAX_PROCESS_STATES: usize = 10;
pub static MAX_SENSOR_BUFFER_SIZE: usize = 10;

#[derive(Clone)]
pub struct EmbeddedProcess {
    name: String,
    driver: String,

    config: Vec<f64>,
    states: Vec<f64>,
    inputs: Vec<f64>,
    outputs: Vec<f64>,

    state_shape: usize,
    input_shapes: Vec<usize>,
    output_shapes: Vec<usize>,

    input_names: Vec<String>,
    output_names: Vec<String>,

    timestamp: f64,
}

impl EmbeddedProcess {
    pub fn default() -> EmbeddedProcess {
        EmbeddedProcess {
            name: "UED".to_string(),
            driver: "UNKNOWN".to_string(),
            config: vec![],
            states: vec![],
            inputs: vec![],
            outputs: vec![],
            state_shape: 0,
            input_shapes: vec![0],
            output_shapes: vec![0],
            input_names: vec![],
            output_names: vec![],
            timestamp: 0.0,
        }
    }

    pub fn set_config(&mut self, config: Vec<f64>) {
        if config.len() > MAX_CONFIG_SIZE {
            panic!("No process config can exceed {} values", MAX_CONFIG_SIZE);
        }
        self.config = config;
    }

    pub fn named(
        name: String,
        driver: String,
        input_shapes: Vec<usize>,
        output_shapes: Vec<usize>,
        input_names: Vec<String>,
        output_names: Vec<String>,
        config: Vec<f64>,
    ) -> EmbeddedProcess {
        EmbeddedProcess {
            name: name,
            driver: driver,
            config: config,
            states: vec![0.0],
            inputs: vec![0.0; input_shapes.iter().sum()],
            outputs: vec![0.0; output_shapes.iter().sum()],
            state_shape: 0,
            input_shapes: input_shapes,
            output_shapes: output_shapes,
            input_names: input_names,
            output_names: output_names,
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

    pub fn update_input(&mut self, data: Vec<f64>, time: f64) {
        let n_inputs: usize = self.input_shapes.iter().sum();
        self.inputs = data[0..n_inputs].to_vec();
        self.timestamp = time;
    }

    pub fn update_output(&mut self, data: Vec<f64>, time: f64) {
        let n_outputs: usize = self.output_shapes.iter().sum();
        self.outputs = data[0..n_outputs].to_vec();
        self.timestamp = time;
    }

    pub fn update_states(&mut self, length: usize, data: Vec<f64>, time: f64) {
        self.state_shape = length;
        self.states = data[0..length].to_vec();
        self.timestamp = time;
    }

    pub fn get_initializers(
        &self,
        report_id: u8,
        report_mode: u8,
        id: u8,
        input_ids: Vec<u8>,
    ) -> Vec<ByteBuffer> {
        let mut reports = vec![];
        let mut node_init = ByteBuffer::hid();
        node_init.puts(0, vec![report_id, report_mode, id]);
        node_init.puts(3, self.driver());
        node_init.put(6, self.config_shape() as u8);
        node_init.put(7, input_ids.len() as u8);
        node_init.puts(8, input_ids);
        reports.push(node_init);

        self.config
            .chunks(MAX_PROCESS_IO)
            .enumerate()
            .for_each(|(i, chunk)| {
                let mut buffer = ByteBuffer::hid();
                buffer.puts(
                    0,
                    vec![report_id, report_mode + 1, id, i as u8, chunk.len() as u8],
                );

                buffer.puts(
                    5,
                    chunk
                        .into_iter()
                        .map(|x| (*x as f32).to_be_bytes())
                        .flatten()
                        .collect(),
                );

                reports.push(buffer);
            });

        reports
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

    pub fn input_shape(&self) -> Vec<usize> {
        self.input_shapes.clone()
    }

    pub fn output_shape(&self) -> Vec<usize> {
        self.output_shapes.clone()
    }

    pub fn config_shape(&self) -> usize {
        self.config.len()
    }

    pub fn n_inputs(&self) -> usize {
        self.input_shapes.iter().sum()
    }

    pub fn n_outputs(&self) -> usize {
        self.output_shapes.iter().sum()
    }

    pub fn input_names(&self) -> Vec<String> {
        self.input_names.clone()
    }

    pub fn output_names(&self) -> Vec<String> {
        self.output_names.clone()
    }

    pub fn driver(&self) -> Vec<u8> {
        self.driver.as_bytes().to_vec()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp
    }

    pub fn print(&self) {
        println!("{:?}: {:?} [{}]", self.name, self.driver, self.timestamp);
        println!("config:\n\t{:?}", self.config);
        println!("state: [{}]\n\t{:?}", self.state_shape, self.states);
        println!("output: {:?}\n\t{:?}", self.output_shapes, self.outputs);
    }
}
