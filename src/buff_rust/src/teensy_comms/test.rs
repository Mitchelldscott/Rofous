#![allow(unused_imports)]
extern crate hidapi;

use hidapi::{HidApi, HidDevice};

use crate::teensy_comms::{
    data_structures::*, hid_common::*, hid_reader::*, hid_ros::*, hid_writer::*,
};
use crate::utilities::loaders::*;

use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    thread::spawn,
    time::Instant,
};

#[allow(dead_code)]
const VERBOSITY: usize = 1;

#[cfg(test)]
pub mod robot_status_tests {
    use super::*;

    #[test]
    pub fn robot_status_load() {
        let mut rs = RobotStatus::new("penguin");
        rs.get_initializers().iter().for_each(|packet| {
            println!("{:02X?}", packet);
        });

        rs.get_reports().iter().for_each(|packet| {
            println!("{:02X?}", packet);
        })
    }
}

#[cfg(test)]
pub mod teensy_comms_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn write_with_debug(writer: &mut HidWriter, sys_time: Instant) {
        let m = writer.output.get(0);
        let t = Instant::now();
        writer.write();
        writer.output.print_data();

        println!(
            "\t[{}] Writer success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            m,
            t.elapsed().as_micros()
        );
    }

    pub fn read_with_debug(reader: &mut HidReader, sys_time: Instant) -> usize {
        let n = reader.read();
        reader.input.print_data();
        println!(
            "\t[{}] Reader success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            reader.input.get(0),
            reader.input.get_i32(60)
        );
        return n;
    }

    pub fn validate_input_packet(reader: &mut HidReader, sys_time: Instant) -> u8 {
        let mode = reader.input.get(0);
        let timer = reader.input.get_i32(60);

        if mode == 255 && VERBOSITY > 0 {
            println!(
                "\t[{}] Config Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else if mode == 0 && VERBOSITY > 0 {
            println!(
                "\t[{}] Idle Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else if VERBOSITY > 0 {
            println!(
                "\t[{}] Data Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        }

        if timer == 0 && VERBOSITY > 0 {
            println!("\tTeensy does not recognize the connection!");
        }

        return mode;
    }

    pub fn watch_for_packet(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut loopt;
        let mode = 0;
        let teensy_f = 1.0 / 600000000.0;
        let t = Instant::now();
        let mut teensy_elapsed_time;

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match reader.read() {
                64 => {
                    teensy_elapsed_time =
                        teensy_f * (reader.input.get_i32(60) as f64 - teensy_cyccnt);
                    teensy_cyccnt = reader.input.get_i32(60) as f64;

                    if reader.input.get(0) == 0 && teensy_cyccnt == 0.0 {
                        continue;
                    } else if teensy_elapsed_time > 0.001 {
                        if VERBOSITY > 0 {
                            println!(
                                "\tTeensy cycle time is over the limit {}",
                                teensy_elapsed_time
                            );
                        }
                    }

                    if reader.input.get(0) == packet_id {
                        if VERBOSITY > 1 {
                            println!("\tTeenys report {} reply received", packet_id);
                        }
                        return teensy_cyccnt;
                    }
                }
                0 => {
                    if VERBOSITY > 0 {
                        println!("\tNo Packet available");
                    }
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            // writer.write();
            while loopt.elapsed().as_micros() < 1000 {}
        }

        assert_eq!(
            packet_id,
            mode,
            "\t[{}] Requested Teensy Packet not found\n\n",
            t.elapsed().as_millis()
        );

        return teensy_cyccnt;
    }

    pub fn watch_for_packet_data(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .gets(index, n)
                .into_iter()
                .map(|x| x as f64)
                .sum::<f64>();
        }

        assert!(
            sum >= 0.0,
            "\t[{}] Requested Teensy Packet not found\n\n",
            t.elapsed().as_millis()
        );
        assert!(
            sum > 0.0,
            "\t[{}] Requested Teensy Data Empty\n\n",
            t.elapsed().as_millis()
        );
        return teensy_cyccnt;
    }

    pub fn watch_for_report(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        report: Vec<u8>,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .data
                .iter()
                .zip(report.iter())
                .map(|(x, r)| (*x - *r) as f64)
                .sum::<f64>();
        }

        assert!(
            sum >= 0.0,
            "\t[{}] Requested Teensy Packet does not match\n\n",
            t.elapsed().as_millis()
        );

        return teensy_cyccnt;
    }

    pub fn watch_for_no_packet_data(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .gets(index, n)
                .into_iter()
                .map(|x| x as f64)
                .sum::<f64>();
        }

        assert!(
            sum == 0.0,
            "\t[{}] Requested Teensy Packet was empty\n\n",
            t.elapsed().as_millis()
        );
        assert!(
            sum != 0.0,
            "\t[{}] Requested  Packet was not empty\n\n",
            t.elapsed().as_millis()
        );

        return teensy_cyccnt;
    }

    pub fn packet_request_test(
        reader: &mut HidReader,
        writer: &mut HidWriter,
        packet_id: u8,
        data: Vec<u8>,
        teensy_cyccnt: f64,
    ) -> f64 {
        if VERBOSITY > 1 {
            println!("Testing packet request {}:...", packet_id);
        }
        writer.send_report(packet_id, data);
        return watch_for_packet(reader, packet_id, 5, teensy_cyccnt);
    }

    pub fn initializer_test(reader: &mut HidReader, writer: &mut HidWriter) {
        let mut robot_status = RobotStatus::new("penguin");
        let initializers = robot_status.get_initializers();
        let mut teensy_cyccnt = 0.0;
        println!("\nTesting initializers:...");
        initializers.into_iter().enumerate().for_each(|(i, init)| {
            println!("Init packet: {}, length {}", i, init.len());
            writer.send_report(INIT_REPORT_ID, init[1..].to_vec());
            teensy_cyccnt =
                watch_for_report(reader, INIT_REPORT_ID, 5, init[..3].to_vec(), teensy_cyccnt);
        });
    }

    pub fn imu_connection_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting imu access:...\n");
        writer.send_report(SENSOR_REPORT_ID, vec![0]);
        watch_for_packet_data(reader, SENSOR_REPORT_ID, 5, 2, 36, 0.0);
        println!("IMU data: {:?}", reader.input.get_floats(2, 9));
    }

    pub fn latency_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting latency alignment:...\n");
        let t = Instant::now();
        // mock request for process 0 status
        let mut teensy_cyccnt = 0.0;
        while t.elapsed().as_millis() < 900 {
            teensy_cyccnt = packet_request_test(
                reader,
                writer,
                PROC_REPORT_ID,
                vec![READ_STATE_MODE, 0],
                teensy_cyccnt,
            )
        }
    }

    #[test]
    pub fn read_write_test() {
        /*
            Attempt to connect and read/write from teensy
        */
        let byu = BuffYamlUtil::new("penguin");

        let vid = byu.load_u16("teensy_vid");
        let pid = byu.load_u16("teensy_pid");

        let mut hidapi = HidApi::new().expect("Failed to create API instance");
        let mut reader = HidReader::new(&mut hidapi, vid, pid);
        let mut writer = HidWriter::new(&mut hidapi, vid, pid);

        initializer_test(&mut reader, &mut writer);

        // latency_test(&mut reader, &mut writer);
        // imu_connection_test(&mut reader, &mut writer);
        // dr16_connection_test(&mut reader, &mut writer);
        // motor_feedback_test(&mut reader, &mut writer);
        // can_control_test(&mut reader, &mut writer);
        // gimbal_input_control_test(&mut reader, &mut writer);
    }

    pub fn sim_control_pipeline(
        shutdown: Arc<RwLock<bool>>,
        control_tx: Sender<Vec<u8>>,
        feedback_rx: Receiver<RobotStatus>,
    ) {
        let mut robot_status = feedback_rx.recv().unwrap_or(RobotStatus::default());

        let initializers = robot_status.get_initializers();

        initializers.iter().for_each(|init| {
            control_tx.send(init.clone()).unwrap();
        });

        println!("HID-SIM Live");

        let mut current_report = 0;
        let reports = robot_status.get_reports();

        let mut publish_timer = Instant::now();
        let mut pub_switch = 0;

        let t = Instant::now();

        while t.elapsed().as_secs() < 5 {
            let loopt = Instant::now();

            control_tx.send(reports[current_report].clone()).unwrap();
            current_report = (current_report + 1) % reports.len();

            while loopt.elapsed().as_micros() < TEENSY_CYCLE_TIME_US as u128 {}
        }

        // buffpy RUN relies on ros to shutdown
        *shutdown.write().unwrap() = true;
    }

    #[test]
    pub fn hid_test() {
        /*
            Start an hid layer
        */
        let byu = BuffYamlUtil::new("penguin");

        let vid = byu.load_u16("teensy_vid");
        let pid = byu.load_u16("teensy_pid");

        let shutdown = Arc::new(RwLock::new(false));
        let shdn1 = shutdown.clone();
        let shdn2 = shutdown.clone();

        let mut hidapi = HidApi::new().expect("Failed to create API instance");
        let mut hidreader = HidReader::new(&mut hidapi, vid, pid);
        let mut hidwriter = HidWriter::new(&mut hidapi, vid, pid);

        // create the rust channels
        let (feedback_tx, feedback_rx): (Sender<RobotStatus>, Receiver<RobotStatus>) =
            mpsc::channel();
        let (control_tx, control_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();

        let hidwriter_handle = spawn(move || {
            hidwriter.pipeline(shutdown, control_rx);
        });

        let pipeline_sim = spawn(move || {
            sim_control_pipeline(shdn2, control_tx, feedback_rx);
        });

        let hidreader_handle = spawn(move || {
            hidreader.pipeline(shdn1, feedback_tx);
        });

        hidreader_handle.join().expect("HID Reader failed");
        pipeline_sim.join().expect("HID Sim failed");
        hidwriter_handle.join().expect("HID Writer failed");
    }
}
