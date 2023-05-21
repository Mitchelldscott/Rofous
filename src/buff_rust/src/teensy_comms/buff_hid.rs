// extern crate hidapi;

// use crate::teensy_comms::data_structures::*;
// use crate::utilities::{buffers::*, loaders::*};
// use hidapi::{HidApi, HidDevice};
// use rosrust_msg::std_msgs;
// use std::{
//     sync::{
//         mpsc,
//         mpsc::{Receiver, Sender},
//         Arc, RwLock,
//     },
//     thread::spawn,
//     time::Instant,
// };

// static TEENSY_CYCLE_TIME_S: f64 = 0.001;
// static TEENSY_CYCLE_TIME_MS: f64 = TEENSY_CYCLE_TIME_S * 1000.0;
// static TEENSY_CYCLE_TIME_US: f64 = TEENSY_CYCLE_TIME_MS * 1000.0;

// pub fn init_hid_device(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidDevice {
//     let dev = hidapi.open(vid, pid).unwrap();
//     dev.set_blocking_mode(false).unwrap();
//     dev
// }

// pub struct HidWriter {
//     pub shutdown: Arc<RwLock<bool>>,
//     pub output: ByteBuffer,
//     pub teensy: HidDevice,
// }

// impl HidWriter {
//     pub fn new(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidWriter {
//         let device = init_hid_device(hidapi, vid, pid); // not two teensys, just two instances

//         HidWriter {
//             // shutdown will be replaced by a variable shared between several HidWriters
//             shutdown: Arc::new(RwLock::new(false)),
//             output: ByteBuffer::new(64),
//             teensy: device,
//         }
//     }

//     /// Write the bytes from the output buffer to the teensy, then clear the buffer.
//     /// Shutdown if the write fails.
//     /// # Usage
//     /// ```
//     /// writer.output.puts(some_index, some_data);
//     /// writer.write(); // writes some_data to the teensy
//     /// ```
//     pub fn write(&mut self) {
//         // let t = Instant::now();
//         match self.teensy.write(&self.output.data) {
//             Ok(_) => {
//                 // println!("Write time {}", t.elapsed().as_micros());
//                 // self.output.print_data();
//                 self.output.reset();
//             }
//             _ => {
//                 *self.shutdown.write().unwrap() = true;
//             }
//         }
//     }

//     /// Creates a report from `id` and `data` and sends it to the teensy. Only use in testing.
//     /// # Usage
//     /// ```
//     ///     writer.teensy = hidapi.open(vid, pid);
//     ///     writer.send_report(report_id, data);
//     /// ```
//     pub fn send_report(&mut self, id: u8, data: Vec<u8>) {
//         self.output.puts(0, vec![id]);
//         self.output.puts(1, data);
//         self.write();
//     }

//     /// Main function to spin and connect the teensy to ROS.
//     /// Cycles through `reports` and sends a report to the teensy every millisecond.
//     /// Only use in testing.
//     pub fn spin(&mut self, reports: Vec<Vec<u8>>) {
//         let mut report_request = 0;
//         println!("HID-writer Live");

//         while !*self.shutdown.read().unwrap() && self.output.timestamp.elapsed().as_millis() < 50 {
//             let loopt = Instant::now();

//             self.output.puts(0, reports[report_request].clone());
//             self.write();
//             report_request = (report_request + 1) % reports.len();
//             if loopt.elapsed().as_micros() > TEENSY_CYCLE_TIME_US as u128 {
//                 println!("HID writer over cycled {}", loopt.elapsed().as_micros());
//             }
//             while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
//         }
//         *self.shutdown.write().unwrap() = true;
//     }
//     /// Continually sends data from [HidROS] (via `control_rx`) to the teensy.
//     ///
//     /// # Arguments
//     /// * `shutdown` - The function stops when this is true.
//     /// Used so that HidLayer threads, all running pipeline() at the same time, can be shutdown at the same time (by passing them the same variable)
//     /// * `control_rx` - Receives the data from [HidROS].
//     ///
//     /// # Example
//     /// See [`HidLayer::pipeline()`] source
//     pub fn pipeline(&mut self, shutdown: Arc<RwLock<bool>>, control_rx: Receiver<Vec<u8>>) {
//         self.shutdown = shutdown;

//         // let mut status = RobotStatus::load_robot();

//         // self.send_report(255, status.load_initializers()[0].clone());

//         println!("HID-writer Live");

//         while !*self.shutdown.read().unwrap() {
//             self.output.puts(0, control_rx.recv().unwrap_or(vec![0]));
//             self.write();
//         }
//     }
// }

// /// Responsible for initializing [RobotStatus] and continuously
// /// sending status reports
// pub struct HidReader {
//     pub shutdown: Arc<RwLock<bool>>,
//     pub input: ByteBuffer,
//     pub robot_status: RobotStatus,
//     pub teensy: HidDevice,
//     pub teensy_lifetime: f64,
//     pub rust_lifetime: f64,
// }

// impl HidReader {
//     pub fn new(hidapi: &mut HidApi, vid: u16, pid: u16) -> HidReader {
//         let device = init_hid_device(hidapi, vid, pid); // not two teensys, just two instances

//         HidReader {
//             shutdown: Arc::new(RwLock::new(false)),
//             input: ByteBuffer::new(64),
//             robot_status: RobotStatus::from_self(),
//             teensy: device,
//             teensy_lifetime: 0.0,
//             rust_lifetime: 0.0,
//         }
//     }

//     /// Read data into the input buffer and return how many bytes were read
//     ///
//     /// # Usage
//     ///
//     /// ```
//     /// match reader.read() {
//     ///     64 => {
//     ///         // packet OK, do something
//     ///     }
//     ///     _ => {} // do nothing
//     /// }
//     /// ```
//     pub fn read(&mut self) -> usize {
//         match &self.teensy.read(&mut self.input.data) {
//             Ok(value) => {
//                 // reset watchdog... woof
//                 self.input.timestamp = Instant::now();
//                 return *value;
//             }
//             _ => {
//                 *self.shutdown.write().unwrap() = true;
//             }
//         }
//         return 0;
//     }

//     /// After requesting a report this can be used to wait for a reply
//     ///
//     /// # Panics
//     ///
//     /// This will panic if a reply is not received from the Teensy
//     /// within `timeout` ms.
//     ///
//     /// # Usage
//     ///
//     /// ```
//     /// // set writer.output.data[0] to an int [1-3] (255 for initializer)
//     /// writer.write();
//     /// reader.wait_for_report_reply(255, 10);
//     /// ```
//     pub fn wait_for_report_reply(&mut self, packet_id: u8, timeout: u128) {
//         let mut loopt;
//         let t = Instant::now();

//         while t.elapsed().as_millis() < timeout {
//             loopt = Instant::now();

//             match self.read() {
//                 64 => {
//                     if self.input.get(0) == 0 && self.input.get_float(60) == 0.0 {
//                         continue;
//                     } else if self.input.get_float(60) > TEENSY_CYCLE_TIME_US {
//                         println!(
//                             "Teensy cycle time is over the limit {}",
//                             self.input.get_float(60)
//                         );
//                     }

//                     self.teensy_lifetime = 0.0;
//                     self.rust_lifetime = 0.0;

//                     if self.input.get(0) == packet_id {
//                         println!("Teenys report {} reply received", packet_id);
//                         return;
//                     }
//                 }
//                 _ => {}
//             }

//             // HID runs at 1 ms
//             while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
//         }

//         // If packet never arrives
//         *self.shutdown.write().unwrap() = true;
//         panic!("\tTimed out waiting for reply from Teensy");
//     }

//     /// Parse the stored HID packet into BuffBot Data Structures
//     ///
//     /// # Usage
//     ///
//     /// ```
//     /// // HID packet waiting
//     /// if reader.read() > 0 { // read returns the number of bytes (64 or bust)
//     ///     reader.parse_report();
//     /// }
//     /// ```
//     pub fn parse_report(&mut self) {
//         // println!(
//         //     "Lifetime relations {} - {} = {}",
//         //     self.teensy_lifetime,
//         //     self.rust_lifetime,
//         //     self.teensy_lifetime - self.rust_lifetime
//         // );

//         // if self.input.get_float(60) - self.teensy_lifetime > TEENSY_CYCLE_TIME_S + 1e-2 {
//         //     println!(
//         //         "Teensy cycle time is over the limit {}",
//         //         self.input.get_float(60) - self.teensy_lifetime
//         //     );
//         // }

//         self.teensy_lifetime = self.input.get_float(60);

//         // match the report number to determine the structure
//         if self.input.get(0) == PROCESS_REPORT_ID {
//             if self.input.get(1) == EST_IO_SWITCH_MODE {
//                 self.robot_status.update_proc_io(
//                     self.input.get(2) as usize,
//                     self.input.get_floats(3, MAX_PROCESS_IO),
//                     self.teensy_lifetime,
//                 );
//             }
//             if self.input.get(1) == EST_STATE_SWITCH_MODE {
//                 self.robot_status.update_proc_state(
//                     self.input.get(2) as usize,
//                     self.input.get_floats(3, MAX_PROCESS_STATES),
//                     self.teensy_lifetime,
//                 );
//             }
//         }

//         if self.input.get(0) == CONTROLLER_REPORT_ID {}

//         if self.input.get(0) == SENSOR_REPORT_ID {
//             self.robot_status.update_sensor(
//                 self.input.get(1) as usize,
//                 self.input.get_floats(2, MAX_SENSOR_BUFFER_SIZE),
//                 self.teensy_lifetime,
//             );
//         }
//         // self.robot_status.motors[4].read().unwrap().print();
//     }

//     /// Main function to spin and connect the teensys
//     /// input to ROS.
//     ///
//     /// # Usage
//     /// ```
//     /// use hidapi::HidApi;
//     /// use buff_rust::teensy_comms::buff_hid::HidReader;
//     ///
//     /// let mut hidapi = HidApi::new().expect("Failed to create API instance");
//     /// let mut reader = HidReader::new(&mut hidapi, vid, pid);
//     /// reader.spin();       // runs until watchdog times out
//     /// ```
//     pub fn spin(&mut self) {
//         let mut readt = Instant::now();

//         while !*self.shutdown.read().unwrap() {
//             let loopt = Instant::now();

//             match self.read() {
//                 64 => {
//                     self.parse_report();
//                     readt = Instant::now();
//                 }

//                 _ => {
//                     if readt.elapsed().as_millis() > 10 && readt.elapsed().as_millis() % 10 == 0 {
//                         println!(
//                             "HID Reader: No reply from Teensy for {}",
//                             readt.elapsed().as_millis()
//                         );
//                     }
//                 }
//             }

//             self.rust_lifetime += TEENSY_CYCLE_TIME_S;
//             if loopt.elapsed().as_micros() > TEENSY_CYCLE_TIME_US as u128 {
//                 println!("HID reader over cycled {}", loopt.elapsed().as_micros());
//             }
//             while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
//         }
//     }

//     /// Sends robot status report packet to [HidROS], waits for the reply packet,
//     /// then calls [HidReader::spin] to begin parsing reports
//     ///
//     /// # Example
//     ///
//     /// see [HidLayer::pipeline()]
//     pub fn pipeline(&mut self, shutdown: Arc<RwLock<bool>>, feedback_tx: Sender<RobotStatus>) {
//         self.shutdown = shutdown;

//         feedback_tx.send(self.robot_status.clone()).unwrap();
//         println!("HID-reader Live");

//         // wait for initializers reply
//         self.wait_for_report_reply(255, 50);

//         self.spin();
//     }
// }

// pub struct HidROS {
//     pub shutdown: Arc<RwLock<bool>>,
//     pub robot_status: RobotStatus,
//     pub control_flag: Arc<RwLock<i32>>,

//     pub sensor_publishers: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
//     pub est_state_publishers: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
// }

// impl HidROS {
//     /// Handles publishing data to ROS from HID
//     ///
//     /// # Usage
//     /// ```
//     /// use buff_rust::teensy_comms::buff_hid::HidROS;
//     ///
//     /// let hidros = HidROS::new();
//     /// ```
//     pub fn new() -> HidROS {
//         let robot_status = RobotStatus::from_self();
//         let sensor_names = robot_status.get_sensor_names();
//         let est_names = robot_status.get_process_names();

//         env_logger::init();
//         rosrust::init("buffpy_hid");

//         let sensor_publishers = sensor_names
//             .iter()
//             .map(|name| rosrust::publish(name, 1).unwrap())
//             .collect();

//         let est_state_publishers = est_names
//             .iter()
//             .map(|name| rosrust::publish(name, 1).unwrap())
//             .collect();

//         HidROS {
//             shutdown: Arc::new(RwLock::new(false)),
//             robot_status: robot_status,
//             control_flag: Arc::new(RwLock::new(0)),

//             sensor_publishers: sensor_publishers,
//             est_state_publishers: est_state_publishers,
//         }
//     }

//     /// Create a new HidROS from an existing RobotStatus
//     ///
//     /// # Usage
//     ///
//     /// ```
//     /// use buff_rust::teensy_comms::buff_hid::HidROS;
//     ///
//     /// let hidros = HidROS::from_robot_status(shutdown, robotstatus);
//     /// ```
//     pub fn from_robot_status(shutdown: Arc<RwLock<bool>>, robot_status: RobotStatus) -> HidROS {
//         env_logger::init();
//         rosrust::init("buffpy_hid");

//         let mut hidros = HidROS::new();
//         hidros.shutdown = shutdown;
//         hidros.robot_status = robot_status;

//         hidros
//     }

//     /// Publish sensor data to ROS
//     pub fn publish_sensors(&self) {
//         self.sensor_publishers
//             .iter()
//             .enumerate()
//             .for_each(|(i, sensor_pub)| {
//                 let mut msg = std_msgs::Float64MultiArray::default();
//                 msg.data = self.robot_status.sensors[i].read().unwrap().data();
//                 msg.data
//                     .push(self.robot_status.sensors[i].read().unwrap().timestamp());
//                 sensor_pub.send(msg).unwrap();
//             });
//     }

//     /// Publish sensor data to ROS
//     pub fn publish_processes(&self) {
//         self.est_state_publishers
//             .iter()
//             .enumerate()
//             .for_each(|(i, est_pub)| {
//                 let mut msg = std_msgs::Float64MultiArray::default();
//                 msg.data = self.robot_status.estimators[i].read().unwrap().state();
//                 msg.data
//                     .push(self.robot_status.estimators[i].read().unwrap().timestamp());
//                 est_pub.send(msg).unwrap();
//             });
//     }

//     /// Start to continually publish motor and sensor data
//     pub fn spin(&mut self) {
//         println!("HID-ROS Live");

//         while rosrust::is_ok() {
//             let loopt = Instant::now();

//             self.publish_sensors();

//             if loopt.elapsed().as_millis() > 5 {
//                 println!("HID ROS over cycled {}", loopt.elapsed().as_micros());
//             }
//             while loopt.elapsed().as_millis() < 5 {}
//         }

//         // buffpy RUN relies on ros to shutdown
//         *self.shutdown.write().unwrap() = true;
//     }

//     /// Begin publishing motor, controller, and sensor data to ROS and
//     /// sending control reports to [HidWriter]
//     ///
//     /// # Example
//     /// See [HidLayer::pipeline()]
//     pub fn pipeline(
//         &mut self,
//         shutdown: Arc<RwLock<bool>>,
//         control_tx: Sender<Vec<u8>>,
//         feedback_rx: Receiver<RobotStatus>,
//     ) {
//         self.shutdown = shutdown;

//         self.robot_status = feedback_rx.recv().unwrap_or(RobotStatus::from_self());

//         let initializers = self.robot_status.load_initializers();

//         initializers.iter().for_each(|init| {
//             control_tx.send(init.clone()).unwrap();
//         });

//         println!("HID-ROS Live");

//         let mut current_report = 0;
//         let reports = self.robot_status.get_reports();

//         let mut publish_timer = Instant::now();
//         let mut pub_switch = 0;

//         while rosrust::is_ok() && !*self.shutdown.read().unwrap() {
//             let loopt = Instant::now();

//             // don't publish every cycle
//             if publish_timer.elapsed().as_millis() > (reports.len() / 2) as u128 {
//                 publish_timer = Instant::now();
//                 match pub_switch {
//                     0 => {
//                         self.publish_sensors();
//                         pub_switch += 1;
//                     }
//                     1 => {
//                         self.publish_processes();
//                         pub_switch = 0;
//                     }
//                     _ => {}
//                 }
//             }

//             control_tx.send(reports[current_report].clone()).unwrap();
//             current_report = (current_report + 1) % reports.len();

//             while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
//         }

//         // buffpy RUN relies on ros to shutdown
//         *self.shutdown.write().unwrap() = true;
//     }
// }

// /// An HID Comms layer for teensys
// /// Usage:
// /// '''
// ///     HidLayer::spin();
// /// '''
// pub struct HidLayer;

// impl HidLayer {
//     /// careful hard to shutdown...

//     pub fn rospipeline() {
//         let byu = BuffYamlUtil::from_self();

//         let vid = byu.load_u16("teensy_vid");
//         let pid = byu.load_u16("teensy_pid");

//         let shutdown = Arc::new(RwLock::new(false));
//         let shdn1 = shutdown.clone();
//         let shdn2 = shutdown.clone();

//         let mut hidapi = HidApi::new().expect("Failed to create API instance");
//         let mut hidreader = HidReader::new(&mut hidapi, vid, pid);
//         let mut hidwriter = HidWriter::new(&mut hidapi, vid, pid);
//         let mut hidros = HidROS::new();

//         // create the rust channels
//         let (feedback_tx, feedback_rx): (Sender<RobotStatus>, Receiver<RobotStatus>) =
//             mpsc::channel();
//         let (control_tx, control_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();

//         let hidwriter_handle = spawn(move || {
//             hidwriter.pipeline(shutdown, control_rx);
//         });

//         let hidros_handle = spawn(move || {
//             hidros.pipeline(shdn2, control_tx, feedback_rx);
//         });

//         let hidreader_handle = spawn(move || {
//             hidreader.pipeline(shdn1, feedback_tx);
//         });

//         hidreader_handle.join().expect("HID Reader failed");
//         hidros_handle.join().expect("HID ROS failed");
//         hidwriter_handle.join().expect("HID Writer failed");
//     }

//     pub fn pipeline() {
//         let byu = BuffYamlUtil::from_self();

//         let vid = byu.load_u16("teensy_vid");
//         let pid = byu.load_u16("teensy_pid");

//         let shutdown = Arc::new(RwLock::new(false));
//         let shdn1 = shutdown.clone();
//         let shdn2 = shutdown.clone();

//         let mut hidapi = HidApi::new().expect("Failed to create API instance");
//         let mut hidreader = HidReader::new(&mut hidapi, vid, pid);
//         let mut hidwriter = HidWriter::new(&mut hidapi, vid, pid);
//         let mut hidros = HidROS::new();

//         // create the rust channels
//         let (feedback_tx, feedback_rx): (Sender<RobotStatus>, Receiver<RobotStatus>) =
//             mpsc::channel();
//         let (control_tx, control_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();

//         let hidwriter_handle = spawn(move || {
//             hidwriter.pipeline(shutdown, control_rx);
//         });

//         let hidros_handle = spawn(move || {
//             hidros.pipeline(shdn2, control_tx, feedback_rx);
//         });

//         let hidreader_handle = spawn(move || {
//             hidreader.pipeline(shdn1, feedback_tx);
//         });

//         hidreader_handle.join().expect("HID Reader failed");
//         hidros_handle.join().expect("HID ROS failed");
//         hidwriter_handle.join().expect("HID Writer failed");
//     }
// }
