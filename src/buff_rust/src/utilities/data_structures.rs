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
    context: Vec<f64>,
    inputs: Vec<f64>,
    outputs: Vec<f64>,

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
            context: vec![],
            inputs: vec![],
            outputs: vec![],
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
        input_names: Vec<String>,
        output_names: Vec<String>,
        config: Vec<f64>,
    ) -> EmbeddedProcess {
        EmbeddedProcess {
            name: name,
            driver: driver,
            config: config,
            context: vec![],
            inputs: vec![],
            outputs: vec![],
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

    pub fn update_output(&mut self, length: usize, data: Vec<f64>, time: f64) {
        // println!("New output {:?}: {} {:?}", self.name, length, data);
        self.outputs = data[0..length].to_vec();
        self.timestamp = time;
    }

    pub fn update_context(&mut self, length: usize, data: Vec<f64>, time: f64) {
        // println!("New context {:?}: {} {:?}", self.name, length, data);
        self.context = data[0..length].to_vec();
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

    pub fn context(&self) -> Vec<f64> {
        self.context.clone()
    }

    pub fn output(&self) -> Vec<f64> {
        self.outputs.clone()
    }

    pub fn output_shape(&self) -> usize {
        self.outputs.len()
    }

    pub fn config_shape(&self) -> usize {
        self.config.len()
    }

    pub fn n_inputs(&self) -> usize {
        self.input_names.len()
    }

    pub fn n_outputs(&self) -> usize {
        self.output_names.len()
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
        println!("state:\n\t{:?}", self.context);
        println!("output:\n\t{:?}", self.outputs);
    }
}
