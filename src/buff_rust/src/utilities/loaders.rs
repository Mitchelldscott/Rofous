extern crate yaml_rust;

use crate::utilities::data_structures::*;
use std::{env, fs};
use yaml_rust::{yaml::Yaml, YamlLoader};

// Define a struct that can clean up the way we load things
// from ros param server

pub struct BuffYamlUtil {
    pub yaml_path: String,
    pub node_data: Yaml,
    pub process_data: Yaml,
}

impl BuffYamlUtil {
    pub fn read_yaml_as_string(yaml_path: &str) -> String {
        fs::read_to_string(yaml_path).expect(format!("No config in {}", yaml_path).as_str())
    }

    pub fn new(bot_name: &str) -> BuffYamlUtil {
        let robot_name = bot_name;
        let project_root = env::var("PROJECT_ROOT").expect("Project root not set");

        let yaml_path = format!("{}/buffpy/data/robots/{}", project_root, robot_name);

        let node_string = BuffYamlUtil::read_yaml_as_string(
            format!(
                "{}/buffpy/data/robots/{}/nodes.yaml",
                project_root, robot_name
            )
            .as_str(),
        );

        let process_string = BuffYamlUtil::read_yaml_as_string(
            format!(
                "{}/buffpy/data/robots/{}/process.yaml",
                project_root, robot_name
            )
            .as_str(),
        );

        BuffYamlUtil {
            yaml_path: yaml_path,
            node_data: YamlLoader::load_from_str(node_string.as_str()).unwrap()[0].clone(),
            process_data: YamlLoader::load_from_str(process_string.as_str()).unwrap()[0].clone(),
        }
    }

    pub fn from_self() -> BuffYamlUtil {
        let project_root = env::var("PROJECT_ROOT").expect("Project root not set");
        let self_path = format!("{}/buffpy/data/robots/self.txt", project_root);
        let robot_name = fs::read_to_string(self_path).unwrap();
        BuffYamlUtil::new(robot_name.as_str())
    }

    pub fn default() -> BuffYamlUtil {
        let robot_name =
            env::var("ROBOT_NAME").expect("Not sure which robot to load, set ROBOT_NAME and retry");

        BuffYamlUtil::new(&robot_name.as_str())
    }

    pub fn load_string(&self, item: &str) -> String {
        self.node_data[item].as_str().unwrap().to_string()
    }

    pub fn load_u16(&self, item: &str) -> u16 {
        self.node_data[item].as_i64().unwrap() as u16
    }

    pub fn load_u128(&self, item: &str) -> u128 {
        self.node_data[item].as_i64().unwrap() as u128
    }

    pub fn load_string_list(&self, item: &str) -> Vec<String> {
        self.node_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| x.as_str().unwrap().to_string())
            .collect()
    }

    pub fn load_u8_list(&self, item: &str) -> Vec<u8> {
        self.node_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| x.as_i64().unwrap() as u8)
            .collect()
    }

    pub fn load_integer_matrix(&self, item: &str) -> Vec<Vec<u8>> {
        self.node_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| {
                x.as_vec()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_i64().unwrap() as u8)
                    .collect()
            })
            .collect()
    }

    pub fn load_float_matrix(&self, item: &str) -> Vec<Vec<f64>> {
        self.node_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| {
                x.as_vec()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_f64().unwrap())
                    .collect()
            })
            .collect()
    }

    pub fn parse_processes(data: &Yaml) -> Vec<EmbeddedProcess> {
        data.as_hash()
            .unwrap()
            .iter()
            .map(|(key, value)| {
                // println!("new process: {:?}", key.as_str().unwrap());
                let name = key.as_str().unwrap();
                let mut inputs = vec![];
                let mut outputs = vec![];
                let mut config = vec![];
                let mut driver = "UNKNOWN".to_string();

                value.as_vec().unwrap().iter().for_each(|item| {
                    item.as_hash().unwrap().iter().for_each(|(k, v)| {
                        match k.as_str().unwrap() {
                            "inputs" => {
                                inputs = v
                                    .as_vec()
                                    .unwrap()
                                    .to_vec()
                                    .iter()
                                    .map(|name| name.as_str().unwrap().to_string())
                                    .collect();
                                // println!("\tinputs: {:?}", v.as_vec().unwrap());
                            }
                            "outputs" => {
                                outputs = v
                                    .as_vec()
                                    .unwrap()
                                    .to_vec()
                                    .iter()
                                    .map(|name| name.as_str().unwrap().to_string())
                                    .collect();
                                // println!("\toutputs: {:?}", v.as_vec().unwrap());
                            }
                            "config" => {
                                v.as_vec().unwrap().iter().for_each(|x| match x {
                                    Yaml::Real(_) => config.push(x.as_f64().unwrap()),
                                    Yaml::Array(a) => {
                                        a.iter().for_each(|n| config.push(n.as_f64().unwrap()))
                                    }
                                    _ => {}
                                });
                                // println!("\tconfig: {:?}", config);
                            }
                            // "input_shapes" => {
                            //     input_shapes = v
                            //         .as_vec()
                            //         .unwrap()
                            //         .iter()
                            //         .map(|x| x.as_i64().unwrap() as usize)
                            //         .collect();
                            // }
                            // "output_shapes" => {
                            //     output_shapes = v
                            //         .as_vec()
                            //         .unwrap()
                            //         .iter()
                            //         .map(|x| x.as_i64().unwrap() as usize)
                            //         .collect();
                            // }
                            "driver" => {
                                driver = v.as_str().unwrap().to_string();
                                // println!("\ttype: {:?}", v.as_str().unwrap());
                            }
                            _ => {}
                        }
                    });
                });
                EmbeddedProcess::named(name.to_string(), driver, inputs, outputs, config)
            })
            .collect()
    }

    // pub fn load_sensors(&self) -> Vec<EmbeddedDevice> {
    //     BuffYamlUtil::parse_device_items(&self.sensor_data)
    // }

    // pub fn load_motors(&self) -> Vec<EmbeddedDevice> {
    //     BuffYamlUtil::parse_device_items(&self.motor_data)
    // }

    pub fn load_processes(&self) -> Vec<EmbeddedProcess> {
        BuffYamlUtil::parse_processes(&self.process_data)
    }
}
