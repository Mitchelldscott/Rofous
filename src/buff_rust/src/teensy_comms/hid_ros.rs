use crate::teensy_comms::{data_structures::*, hid_common::*};
use rosrust_msg::std_msgs;
use std::{
    sync::{
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    time::Instant,
};

pub struct HidROS {
    pub shutdown: Arc<RwLock<bool>>,
    pub robot_status: RobotStatus,
    pub control_flag: Arc<RwLock<i32>>,

    pub sensor_pubs: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
    pub proc_state_pubs: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
    pub proc_output_pubs: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
}

impl HidROS {
    /// Handles publishing data to ROS from HID
    ///
    /// # Usage
    /// ```
    /// use buff_rust::teensy_comms::buff_hid::HidROS;
    ///
    /// let hidros = HidROS::new();
    /// ```
    pub fn new() -> HidROS {
        let robot_status = RobotStatus::from_self();
        let sensor_names = robot_status.get_sensor_names();
        let proc_names = robot_status.get_process_names();

        env_logger::init();
        rosrust::init("buffpy_hid");

        let sensor_pubs = sensor_names
            .iter()
            .map(|name| rosrust::publish(name, 1).unwrap())
            .collect();

        let proc_state_pubs = proc_names
            .iter()
            .map(|name| rosrust::publish(format!("{}_state", name).as_str(), 1).unwrap())
            .collect();

        let proc_output_pubs = proc_names
            .iter()
            .map(|name| rosrust::publish(format!("{}_output", name).as_str(), 1).unwrap())
            .collect();

        HidROS {
            shutdown: Arc::new(RwLock::new(false)),
            robot_status: robot_status,
            control_flag: Arc::new(RwLock::new(0)),

            sensor_pubs: sensor_pubs,
            proc_state_pubs: proc_state_pubs,
            proc_output_pubs: proc_output_pubs,
        }
    }

    /// Create a new HidROS from an existing RobotStatus
    ///
    /// # Usage
    ///
    /// ```
    /// use buff_rust::teensy_comms::buff_hid::HidROS;
    ///
    /// let hidros = HidROS::from_robot_status(shutdown, robotstatus);
    /// ```
    pub fn from_robot_status(shutdown: Arc<RwLock<bool>>, robot_status: RobotStatus) -> HidROS {
        env_logger::init();
        rosrust::init("buffpy_hid");

        let mut hidros = HidROS::new();
        hidros.shutdown = shutdown;
        hidros.robot_status = robot_status;

        hidros
    }

    /// Publish sensor data to ROS
    pub fn publish_sensors(&self) {
        self.sensor_pubs
            .iter()
            .enumerate()
            .for_each(|(i, sensor_pub)| {
                let mut msg = std_msgs::Float64MultiArray::default();
                msg.data = self.robot_status.sensors[i].read().unwrap().data();
                msg.data
                    .push(self.robot_status.sensors[i].read().unwrap().timestamp());
                sensor_pub.send(msg).unwrap();
            });
    }

    /// Publish sensor data to ROS
    pub fn publish_process_states(&self) {
        self.proc_state_pubs
            .iter()
            .enumerate()
            .for_each(|(i, proc_state_pub)| {
                let mut msg = std_msgs::Float64MultiArray::default();
                msg.data = self.robot_status.processes[i].read().unwrap().state();
                msg.data
                    .push(self.robot_status.processes[i].read().unwrap().timestamp());
                proc_state_pub.send(msg).unwrap();
            });
    }

    pub fn publish_process_outputs(&self) {
        self.proc_output_pubs
            .iter()
            .enumerate()
            .for_each(|(i, proc_out_pub)| {
                let mut msg = std_msgs::Float64MultiArray::default();
                msg.data = self.robot_status.processes[i].read().unwrap().output();
                msg.data
                    .push(self.robot_status.processes[i].read().unwrap().timestamp());
                proc_out_pub.send(msg).unwrap();
            });
    }

    /// Start to continually publish motor and sensor data
    pub fn spin(&mut self) {
        println!("HID-ROS Live");

        while rosrust::is_ok() {
            let loopt = Instant::now();

            self.publish_sensors();

            if loopt.elapsed().as_millis() > 5 {
                println!("HID ROS over cycled {}", loopt.elapsed().as_micros());
            }
            while loopt.elapsed().as_millis() < 5 {}
        }

        // buffpy RUN relies on ros to shutdown
        *self.shutdown.write().unwrap() = true;
    }

    /// Begin publishing motor, controller, and sensor data to ROS and
    /// sending control reports to [HidWriter]
    ///
    /// # Example
    /// See [HidLayer::pipeline()]
    pub fn pipeline(
        &mut self,
        shutdown: Arc<RwLock<bool>>,
        control_tx: Sender<Vec<u8>>,
        feedback_rx: Receiver<RobotStatus>,
    ) {
        self.shutdown = shutdown;

        self.robot_status = feedback_rx.recv().unwrap_or(RobotStatus::default());

        let initializers = self.robot_status.get_initializers();

        initializers.iter().for_each(|init| {
            control_tx.send(init.clone()).unwrap();
        });

        println!("HID-ROS Live");

        let mut current_report = 0;
        let reports = self.robot_status.get_reports();

        let mut publish_timer = Instant::now();
        let mut pub_switch = 0;

        while rosrust::is_ok() && !*self.shutdown.read().unwrap() {
            let loopt = Instant::now();

            // don't publish every cycle
            if publish_timer.elapsed().as_millis() > (reports.len() / 2) as u128 {
                publish_timer = Instant::now();
                match pub_switch {
                    0 => {
                        self.publish_sensors();
                        pub_switch += 1;
                    }
                    1 => {
                        self.publish_process_states();
                        pub_switch += 1;
                    }
                    2 => {
                        self.publish_process_outputs();
                        pub_switch = 0;
                    }
                    _ => {}
                }
            }

            control_tx.send(reports[current_report].clone()).unwrap();
            current_report = (current_report + 1) % reports.len();

            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }

        // buffpy RUN relies on ros to shutdown
        *self.shutdown.write().unwrap() = true;
    }
}
