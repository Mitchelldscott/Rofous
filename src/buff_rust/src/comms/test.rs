/********************************************************************************
 *
 *      ____                     ____          __           __       _
 *     / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *    / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *   / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  )
 *  /_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/
 *        /____/
 *
 *
 *
 ********************************************************************************/

#![allow(unused_imports)]
extern crate hidapi;

use hidapi::{HidApi, HidDevice};

use crate::{
    comms::{data_structures::*, hid_interface::*, hid_layer::*, hid_reader::*, hid_writer::*, socks::*},
    utilities::{data_structures::*, loaders::*},
};

use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    thread::{spawn, Builder},
    time::Instant,
};

#[allow(dead_code)]
const VERBOSITY: usize = 1;
pub static TEST_DURATION: u64 = 30;

#[cfg(test)]
pub mod robot_fw {
    use super::*;

    #[test]
    pub fn robot_fw_load() {
        let rs = RobotFirmware::new("penguin");

        rs.print();

        rs.task_init_packets().iter().for_each(|packet| {
            packet.print();
        });

        rs.print();
    }
}

///
/// Test the hid functionality on the Teensy
/// Only demonstrates the ability to maintain a connection
/// This is usually paried with firmware/examples/hid/live_test.cpp
/// Dump packets to the Teensy at 1ms, each packet contains a counter
/// and timestamp.
#[cfg(test)]
pub mod dead_read_write {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn hid_read_write_spawner() {
        /*
            Start an hid layer
        */
        // let (layer, writer_rx) = HidLayer::new("penguin");
        // let sim_layer = layer.clone();
        // let mut hidreader = HidReader::new(layer.clone());
        // let mut hidwriter = HidWriter::new(layer, writer_rx);

        let (mut interface, mut reader, mut writer) = HidInterface::new("penguin");

        interface.layer.print();

        let reader_handle = Builder::new()
            .name("HID Reader".to_string())
            .spawn(move || {
                reader.pipeline();
            })
            .unwrap();

        let writer_handle = Builder::new()
            .name("HID Writer".to_string())
            .spawn(move || {
                writer.pipeline();
            })
            .unwrap();

        let t = Instant::now();
        while t.elapsed().as_secs() < TEST_DURATION && !interface.layer.control_flags.is_shutdown()
        {
            let loopt = Instant::now();
            interface.check_feedback();
            interface.layer.delay(loopt);
        }
        interface.layer.control_flags.shutdown();

        reader_handle.join().expect("[HID-Reader]: failed");
        writer_handle.join().expect("[HID-Writer]: failed");
        interface.layer.print();
    }
}

#[cfg(test)]
pub mod dead_comms {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn sim_interface(mut interface: HidInterface) {
        while !interface.layer.control_flags.is_connected() {}

        println!("[HID-Control]: Live");

        let t = Instant::now();

        while t.elapsed().as_secs() < TEST_DURATION && !interface.layer.control_flags.is_shutdown()
        {
            let loopt = Instant::now();

            if interface.layer.control_flags.is_connected() {
                let mut buffer = ByteBuffer::hid();
                buffer.puts(0, vec![255, 255]);
                buffer.put_float(2, interface.layer.pc_stats.packets_sent());
                interface.writer_tx(buffer);
                interface.check_feedback();
            }

            interface.layer.delay(loopt);
            // if interface.delay(t) > TEENSY_CYCLE_TIME_US {
            //     println!("HID Control over cycled {}", t.elapsed().as_micros());
            // }
        }

        interface.layer.control_flags.shutdown();
        println!("[HID-Control]: shutdown");
        interface.layer.print();
    }

    #[test]
    pub fn hid_spawner() {
        /*
            Start an hid layer
        */
        // let (layer, writer_rx) = HidLayer::new("penguin");
        // let sim_layer = layer.clone();
        // let mut hidreader = HidReader::new(layer.clone());
        // let mut hidwriter = HidWriter::new(layer, writer_rx);

        let (interface, mut reader, mut writer) = HidInterface::new("penguin");

        interface.layer.print();

        let reader_handle = Builder::new()
            .name("HID Reader".to_string())
            .spawn(move || {
                reader.pipeline();
            })
            .unwrap();

        let writer_handle = Builder::new()
            .name("HID Writer".to_string())
            .spawn(move || {
                writer.pipeline();
            })
            .unwrap();

        let interface_sim = Builder::new()
            .name("HID Control".to_string())
            .spawn(move || {
                sim_interface(interface);
            })
            .unwrap();

        reader_handle.join().expect("HID Reader failed");
        interface_sim.join().expect("HID Control failed");
        writer_handle.join().expect("HID Writer failed");
    }
}

#[cfg(test)]
pub mod live_comms {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn sim_interface(mut interface: HidInterface) {
        let initializers = interface.get_initializers();

        while !interface.layer.control_flags.is_connected() {}

        println!("[HID-Control]: Live");

        // Not as worried about efficiency here
        // Also it hard to know how many init packets
        // there will be... so K.I.S.S.
        initializers.iter().for_each(|init| {
            let t = Instant::now();
            interface.writer_tx(init.clone());
            interface.layer.delay(t);
        });

        let lifetime = Instant::now();
        let mut t = Instant::now();

        while lifetime.elapsed().as_secs() < TEST_DURATION
            && !interface.layer.control_flags.is_shutdown()
        {
            let loopt = Instant::now();

            if interface.layer.control_flags.is_connected() {
                interface.check_feedback();
                if t.elapsed().as_secs() >= 5 {
                    interface.send_control(0, vec![0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                    interface.layer.print();
                    interface.print();
                    t = Instant::now();
                }
            }

            interface.layer.delay(loopt);
            // if t.elapsed().as_micros() as f64 > TEENSY_CYCLE_TIME_US {
            //     println!("HID Control over cycled {} ms", (t.elapsed().as_micros() as f64) * 1E-3);
            // }
        }

        interface.layer.control_flags.shutdown();
        println!("[HID-Control]: shutdown");
        interface.layer.print();
    }

    #[test]
    pub fn hid_spawner() {
        /*
            Start an hid layer
        */

        let (interface, mut reader, mut writer) = HidInterface::new("penguin");

        interface.layer.print();

        let reader_handle = Builder::new()
            .name("HID Reader".to_string())
            .spawn(move || {
                reader.pipeline();
            })
            .unwrap();

        let writer_handle = Builder::new()
            .name("HID Writer".to_string())
            .spawn(move || {
                writer.pipeline();
            })
            .unwrap();

        let interface_sim = Builder::new()
            .name("HID Control".to_string())
            .spawn(move || {
                sim_interface(interface);
            })
            .unwrap();

        reader_handle.join().expect("[HID-Reader]: failed");
        interface_sim.join().expect("[HID-Control]: failed");
        writer_handle.join().expect("[HID-Writer]: failed");
    }
}

#[cfg(test)]
pub mod live_record {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn sim_interface(mut interface: HidInterface) {
        let initializers = interface.get_initializers();

        while !interface.layer.control_flags.is_connected() {}

        println!("[HID-Control]: Live");

        // Not as worried about efficiency here
        // Also it hard to know how many init packets
        // there will be... so K.I.S.S.
        initializers.iter().for_each(|init| {
            let t = Instant::now();
            interface.writer_tx(init.clone());
            interface.layer.delay(t);
        });

        let lifetime = Instant::now();

        while lifetime.elapsed().as_secs() < TEST_DURATION
            && !interface.layer.control_flags.is_shutdown()
        {
            let loopt = Instant::now();

            if interface.layer.control_flags.is_connected() {
                // interface.send_request();
                interface.check_feedback();
            }

            interface.layer.delay(loopt);
            // if t.elapsed().as_micros() as f64 > TEENSY_CYCLE_TIME_US {
            //     println!("HID Control over cycled {} ms", (t.elapsed().as_micros() as f64) * 1E-3);
            // }
        }

        interface.layer.control_flags.shutdown();
        println!("[HID-Control]: shutdown");
        interface.layer.print();
    }

    #[test]
    pub fn hid_spawner() {
        /*
            Start an hid layer
        */

        let (interface, mut reader, mut writer) = HidInterface::new("penguin");

        interface.layer.print();

        let reader_handle = Builder::new()
            .name("HID Reader".to_string())
            .spawn(move || {
                reader.pipeline();
            })
            .unwrap();

        let writer_handle = Builder::new()
            .name("HID Writer".to_string())
            .spawn(move || {
                writer.pipeline();
            })
            .unwrap();

        let interface_sim = Builder::new()
            .name("HID Control".to_string())
            .spawn(move || {
                sim_interface(interface);
            })
            .unwrap();

        reader_handle.join().expect("[HID-Reader]: failed");
        interface_sim.join().expect("[HID-Control]: failed");
        writer_handle.join().expect("[HID-Writer]: failed");
    }
}

#[cfg(test)]
pub mod socks_beta_again {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;
    const SEND_PACKET_SIZE: usize = 1024;
    ///
    /// Test the byte buffer
    ///


    #[test]
    pub fn sock_server2() {
        sock_server_gen2();
    }


    #[test]
    pub fn sock_test1() {
        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 1;
        sock_base(1, 25, data);
    }

    #[test]
    pub fn sock_test6() {
        let mut data = vec![13; SEND_PACKET_SIZE];
        data[1] = 6;
        sock_base(6, 5000, data);
    }
}


#[cfg(test)]
pub mod socks_beta_actual {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;
    ///
    /// Test the byte buffer
    ///


    #[test]
    pub fn sock_server4() {
        SockServer::core();
    }


    #[test]
    pub fn sock_test1() {
        let rate = 500.0;
        let sock = SockClient::request_subscribe(1, rate, vec![2]);

        let t = Instant::now();
        while t.elapsed().as_secs() < 5 && !sock.is_shutdown() {
            let lt = Instant::now();
            while lt.elapsed().as_micros() as f64 * 1E-3 < (1.0 / (rate * 2.0)) * 1000.0 {}
        }
        println!("Server writes and reads: {:?} {}", sock.receive_buffer.read().unwrap().get_floats(2, 2), t.elapsed().as_micros() as f64 * 1E-6);
        println!("Sock1 reader finished");
    }

    #[test]
    pub fn sock_test2() {
        let rate = 500.0;

        let t = Instant::now();
        while t.elapsed().as_secs() < 5 {
            let lt = Instant::now();
            SockClient::request_publish(2, rate, vec![3]);
            while lt.elapsed().as_micros() as f64 * 1E-3 < (1.0 / (rate * 2.0)) * 1000.0 {}
        }
        println!("Sock2 reader finished");
    }
}
