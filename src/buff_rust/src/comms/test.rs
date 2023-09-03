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
#![allow(unused_macros)]
extern crate hidapi;

use hidapi::{HidApi, HidDevice};

use crate::{
    comms::{
        data_structures::*, hid_interface::*, hid_layer::*, hid_reader::*, hid_writer::*, socks::*,
    },
    utilities::{data_structures::*, loaders::*},
};

use std::{
    env,
    net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket},
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    thread::{spawn, Builder},
    time::{Duration, Instant},
};

use more_asserts::assert_le;

macro_rules! sock_uri {
    () => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 0) }};
    ($port:expr) => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), $port) }};
    ($port:expr, $ip1:expr, $ip2:expr, $ip3:expr, $ip4:expr) => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new($ip1, $ip2, $ip3, $ip4)), $port) }};
}

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

pub fn packet_sum(buffer: [u8; UDP_PACKET_SIZE]) -> u32 {
    buffer.iter().map(|x| *x as u32).sum()
}

#[cfg(test)]
pub mod u_socks {
    use super::*;
    pub const UDP_PACKET_SIZE: usize = 1024;

    #[test]
    pub fn init_read() {
        let mut t = Instant::now();
        let socket = UdpSocket::bind(env::var("DYSE_CORE_URI").unwrap())
            .expect("Couldn't bind to socket 1313");
        socket.set_read_timeout(Some(Duration::new(0, 1))).unwrap();
        assert_le!(t.elapsed().as_micros(), 120, "Core bind time us");

        t = Instant::now();
        let mut buf = [0; UDP_PACKET_SIZE];
        let (size, src_addr) = socket.recv_from(&mut buf).expect("Didn't receive data");
        assert_le!(t.elapsed().as_micros(), 123, "Core read time us");

        assert_eq!(
            src_addr,
            SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 7878)
        );
        assert_eq!(size, UDP_PACKET_SIZE, "Packet was incorrect size");
        assert_eq!(
            packet_sum(buf),
            UDP_PACKET_SIZE as u32,
            "Buffer read had incorrect sum"
        );
    }

    #[test]
    pub fn init_write() {
        let mut t = Instant::now();
        let socket = UdpSocket::bind("127.0.0.1:7878").expect("Couldn't bind to socket 7878");
        assert_le!(t.elapsed().as_micros(), 100, "Client bind time us");

        t = Instant::now();
        socket.connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        socket.set_write_timeout(Some(Duration::new(0, 1))).unwrap();
        assert_le!(t.elapsed().as_micros(), 100, "Client connect time us");

        t = Instant::now();
        let mut buf = [1; UDP_PACKET_SIZE];
        let size = socket.send(&mut buf).expect("Didn't send data");
        assert_le!(t.elapsed().as_micros(), 100, "Client write time us");

        assert_eq!(size, UDP_PACKET_SIZE, "Sent the wrong packet size");
    }

    #[test]
    pub fn init_read_loop() {
        let millis = 500;
        let lifetime = Instant::now();
        while lifetime.elapsed().as_micros() < 250 as u128 {}

        let mut lt = Instant::now();
        let socket = UdpSocket::bind(env::var("DYSE_CORE_URI").unwrap())
            .expect("Couldn't bind to socket 1313");
        socket.set_read_timeout(Some(Duration::new(0, 1))).unwrap();
        assert_le!(lt.elapsed().as_micros(), 100, "Core bind time us");

        assert_eq!(
            socket.local_addr().unwrap(),
            SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 1313),
            "Didn't bind to requested IP"
        );

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {
            lt = Instant::now();
            let mut buf = [0; UDP_PACKET_SIZE];
            let (size, src_addr) = match socket.recv_from(&mut buf) {
                Ok((size, src_addr)) => (size, src_addr),
                Err(e) => {
                    println!("Read {:?}", e);
                    (
                        0,
                        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 1313),
                    )
                }
            };

            assert_le!(lt.elapsed().as_micros(), 500, "Client read time us");

            assert_eq!(
                src_addr,
                SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 7878)
            );
            assert_eq!(size, UDP_PACKET_SIZE, "Packet size was incorrect size");
            assert_eq!(
                packet_sum(buf),
                UDP_PACKET_SIZE as u32,
                "Buffer read had incorrect sum"
            );

            while t.elapsed().as_millis() < millis {}
        }
        println!("R Finished {}s", t.elapsed().as_micros() as f64 * 1E-6);
    }

    #[test]
    pub fn init_write_loop() {
        let millis = 500;
        let lifetime = Instant::now();
        while lifetime.elapsed().as_micros() < 250 as u128 {}

        let mut lt = Instant::now();
        let socket = UdpSocket::bind("127.0.0.1:7878").expect("Couldn't bind to socket 7878");
        socket.connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        socket.set_write_timeout(Some(Duration::new(0, 1))).unwrap();
        assert_le!(lt.elapsed().as_micros(), 100, "Client bind time us");

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {
            lt = Instant::now();
            let mut buf = [1; UDP_PACKET_SIZE];
            let size = match socket.send(&mut buf) {
                Ok(size) => size,
                _ => 0, // broken connection, session died
            };
            assert_le!(lt.elapsed().as_micros(), 500, "Client send time us");

            match size {
                0 => break,
                _ => {}
            }

            assert_eq!(size, UDP_PACKET_SIZE, "Sent wrong packet size");

            while t.elapsed().as_millis() < millis {}
        }
        println!("W Finished {}s", t.elapsed().as_micros() as f64 * 1E-6);
    }

    #[test]
    pub fn init_read_loop_obj() {
        let millis = 500;
        let lifetime = Instant::now();
        while lifetime.elapsed().as_secs() < 0 as u64 {}

        let mut lt = Instant::now();
        let mut sock = Sockage::core("loop_reader");
        assert_le!(lt.elapsed().as_micros(), 100, "Core bind time us");

        assert_eq!(
            sock.socket.local_addr().unwrap(),
            SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 1313),
            "Didn't bind to requested IP"
        );

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {
            lt = Instant::now();
            let mut buffer = [0; UDP_PACKET_SIZE];
            sock.recv(&mut buffer);

            assert_le!(lt.elapsed().as_micros(), 500, "Client read time us");
            assert_eq!(
                packet_sum(buffer),
                UDP_PACKET_SIZE as u32,
                "Buffer read had incorrect sum"
            );

            while t.elapsed().as_millis() < millis {}
        }

        println!(
            "Core Finished {}s",
            lifetime.elapsed().as_micros() as f64 * 1E-6
        );
    }

    #[test]
    pub fn init_write_loop_obj() {
        let millis = 500;
        let lifetime = Instant::now();
        while lifetime.elapsed().as_secs() < 0 as u64 {}

        let mut lt = Instant::now();
        let mut sock = Sockage::new("loop_writer", sock_uri!(7878), millis);
        assert_le!(lt.elapsed().as_micros(), 100, "Client bind time us");

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {
            lt = Instant::now();
            let buffer = [1; UDP_PACKET_SIZE];
            sock.send_to(buffer, sock_uri!(1313));
            assert_le!(lt.elapsed().as_micros(), 500, "Client send time us");

            while t.elapsed().as_millis() < millis {}
        }
        println!(
            "Client Finished {}s",
            lifetime.elapsed().as_micros() as f64 * 1E-6
        );
    }
}

#[cfg(test)]
pub mod low_socks {
    use super::*;

    #[test]
    pub fn core() {
        let millis = 10;
        let lifetime = Instant::now();
        while lifetime.elapsed().as_secs() < 0 as u64 {}

        let mut known_names: Vec<String> = vec![];
        let mut known_addrs: Vec<SocketAddr> = vec![];

        let mut sock = Sockage::core("low_sock");

        assert_eq!(
            sock.socket.local_addr().unwrap(),
            SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 1313),
            "Core Didn't bind to requested IP"
        );

        let t = Instant::now();
        while t.elapsed().as_secs() < 4 {
            let mut buffer = [0; UDP_PACKET_SIZE];
            let src = sock.recv(&mut buffer);

            sock.shutdown(false);

            match src.port() > 0 {
                true => {
                    let name = parse_string(0, &mut buffer);
                    match known_addrs.iter().find(|a| **a == src) {
                        Some(_) => {}
                        _ => {
                            sock.log(format!("New Request from {}", name));
                            known_addrs.push(src);
                            known_names.push(name);
                        }
                    }
                    sock.send_to(sock.stamp_packet(), src);
                }

                _ => {}
            }

            while t.elapsed().as_millis() < millis {}
        }

        sock.log(format!("Ports: {:?}", known_addrs));
        sock.log(format!("Names: {:?}", known_names));

        sock.log(format!(
            "Finished {}s",
            lifetime.elapsed().as_micros() as f64 * 1E-6
        ));
    }

    pub fn simple_node(name: &str, rate: f64) {
        let millis = (1000.0 / rate) as u128;
        let mut lifetime = Instant::now();
        while lifetime.elapsed().as_millis() < 1 as u128 {}

        let mut server_replies = 0;
        let mut sock = Sockage::client(name, millis);

        sock.log("Active");

        lifetime = Instant::now();
        let mut t = Instant::now();
        while lifetime.elapsed().as_secs() < 2 {
            sock.send_to(sock.stamp_packet(), sock_uri!(1313));

            let mut buffer = [0; UDP_PACKET_SIZE];
            let src = sock.recv(&mut buffer);

            match src.port() {
                1313 => {
                    let name = parse_string(0, &mut buffer);
                    sock.log(format!("Request from {}", name));
                    server_replies += 1;
                }

                _ => {}
            }

            while t.elapsed().as_millis() < sock.millis_rate {}
            t = Instant::now();
        }

        sock.log(format!(
            "Finished {}s",
            lifetime.elapsed().as_micros() as f64 * 1E-6
        ));
        assert_le!(1, server_replies, "Didn't receive Reply from server");
    }

    #[test]
    pub fn node1() {
        simple_node("sock_node1", 2.0);
    }

    #[test]
    pub fn node2() {
        simple_node("sock_node2", 4.0);
    }
}

#[cfg(test)]
pub mod mid_socks {
    use super::*;

    #[test]
    pub fn core() {
        let millis = 250;

        let mut sock = Sockage::core("low_sock");

        assert_eq!(
            sock.socket.local_addr().unwrap(),
            SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 1313),
            "Core Didn't bind to requested IP"
        );

        let t = Instant::now();
        while t.elapsed().as_secs() < 4 {
            let mut buffer = [0; UDP_PACKET_SIZE];
            let src = sock.recv(&mut buffer);
            let (sender, mode, names) = read_all_names(&buffer);
            sock.shutdown(false);

            match (src.port() > 0, names.len() > 0) {
                (true, true) => {       // sock requesting to refresh registry

                    sock.discover_sock(sender, &src);
                    let buffer = sock.addrs_packet(0, sock.adress_of_names(names));
                    sock.send_to(buffer, src);
                    // sock.log_heavy();
                    
                }
                (true, false) => {      // sock requesting item
                    match mode {
                        true => {
                            sock.discover_sock(sender, &src);
                            sock.send_to(sock.stamp_packet(), src);
                        }
                        false => {},
                    }
                },
                _ => {},
            }

            while t.elapsed().as_millis() < millis {}
        }

        sock.log_heavy();
        sock.log("Finished");
    }

    pub fn simple_node(name: &str, rate: f64) {
        let millis = (1000.0 / rate) as u128;
        let mut lifetime = Instant::now();
        while lifetime.elapsed().as_millis() < 1 as u128 {}

        let mut server_replies = 0;
        let mut sock = Sockage::client(name, millis);

        sock.log("Active");

        // sock.names.push("tall_socks".to_string());

        lifetime = Instant::now();
        let mut t = Instant::now();
        while lifetime.elapsed().as_secs() < 2 {
            sock.send_to(sock.names_packet(1), sock_uri!(1313));

            let mut buffer = [0; UDP_PACKET_SIZE];
            let src = sock.recv(&mut buffer);
            let (sender, mode, addrs) = read_all_addrs(&buffer);

            match src.port() > 0 {
                true => {
                    match mode {
                        true => {},
                        false => {
                            sock.send_to(sock.data_packet(vec![1.0,2.0,3.0,4.0,5.0]), src)
                        }
                    }
                    sock.log(format!("{} {} {:?}", sender, mode, addrs));
                    server_replies += 1;
                }

                _ => {}
            }

            while t.elapsed().as_millis() < sock.millis_rate {}
            t = Instant::now();
        }

        sock.log("Finished");
        assert_le!(1, server_replies, "Didn't receive Reply from server");
    }

    #[test]
    pub fn node1() {
        simple_node("sock_node1", 2.0);
    }

    // #[test]
    // pub fn node2() {
    //     simple_node("sock_node2", 4.0);
    // }

    #[test]
    pub fn node3() {
        let name = "sock_node3";
        let rate = 2.0;
        // let mode = 0;
        let millis = (1000.0 / rate) as u128;
        let mut lifetime = Instant::now();
        while lifetime.elapsed().as_millis() < 1 as u128 {}

        let mut server_replies = 0;
        let mut sock = Sockage::client(name, millis);

        sock.log("Active");

        sock.names.push("sock_node1".to_string());
        // sock.names.push("sock_node2".to_string());

        lifetime = Instant::now();
        let mut t = Instant::now();
        while lifetime.elapsed().as_secs() < 2 {
            sock.send_to(sock.names_packet(1), sock_uri!(1313));

            let mut buffer = [0; UDP_PACKET_SIZE];
            let src = sock.recv(&mut buffer);
            let (sender, _mode, addrs) = read_all_addrs(&buffer);

            match src.port() {
                1313 => {
                    // sock.log(format!("{} {:?}", sender, addrs));

                    (0..sock.names.len()).for_each(|i| {
                        sock.discover_sock(sock.names[i].to_string(), &addrs[i])
                    });
                    server_replies += 1;
                    (0..sock.addrs.len()).for_each(|i| {
                        sock.send_to(sock.stamp_packet(), addrs[i]);
                    })
                }

                _ => {
                    if src.port() > 0 {
                        sock.log(format!("Received data from {}", sender));
                    }
                }
            }

            while t.elapsed().as_millis() < sock.millis_rate {}
            t = Instant::now();
        }

        sock.log_heavy();
        assert_le!(1, server_replies, "Didn't receive Reply from server");
    }
}