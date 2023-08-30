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
use crate::utilities::{data_structures::*, loaders::*, socks::*};
use rand::Rng;
use std::env;
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    thread::{spawn, Builder},
    time::{Duration, Instant},
};

#[cfg(test)]
pub mod byu_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn load_yaml() {
        /*
            Use the penguin yaml file to test some loading functions
        */

        let byu = BuffYamlUtil::new("penguin");

        assert_eq!(byu.load_string("robot_type"), "demo");

        byu.load_tasks();
    }
}

#[cfg(test)]
pub mod buffer_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    ///
    /// Test the byte buffer
    ///
    #[test]
    pub fn basic_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: u8 = rng.gen();
        let i: usize = rng.gen_range(0..63);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.put(i, n1);
        assert_eq!(
            buffer.get(i),
            n1,
            "[{}] Failed get check {} != {}",
            i,
            n1,
            buffer.get(i)
        );
    }

    #[test]
    pub fn intermediate_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: Vec<u8> = vec![rng.gen(); 10];
        let i: usize = rng.gen_range(0..53);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.puts(i, n1.clone());

        assert_eq!(
            buffer.get(i),
            n1[0],
            "[{}] Failed get check {} != {}",
            i,
            n1[0],
            buffer.get(i)
        );
        assert_eq!(
            buffer.get(i + 1),
            n1[1],
            "[{}] Failed get check {} != {}",
            i + 1,
            n1[1],
            buffer.get(i + 1)
        );
        assert_eq!(
            buffer.get(i + 2),
            n1[2],
            "[{}] Failed get check {} != {}",
            i + 2,
            n1[2],
            buffer.get(i + 2)
        );
        assert_eq!(
            buffer.get(i + 3),
            n1[3],
            "[{}] Failed get check {} != {}",
            i + 3,
            n1[3],
            buffer.get(i + 3)
        );
    }

    #[test]
    pub fn get_float_byte_buffer() {
        let n1: Vec<u8> = vec![0x40, 0x49, 0xf, 0xdb];

        let mut buffer = ByteBuffer::new(64);
        buffer.puts(2, n1.clone());
        buffer.print();

        assert_eq!(
            buffer.get_float(2),
            3.1415927410125732,
            "Failed Float check {} != {}",
            3.1415927410125732,
            buffer.get_float(2)
        );
    }
}

#[cfg(test)]
pub mod socks_beta_0 {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;
    const SEND_PACKET_SIZE: usize = 1024;
    ///
    /// Test the byte buffer
    ///

    #[test]
    pub fn socet_server() {
        sock_server_gen1();
    }

    #[test]
    pub fn sock_test1() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 50 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
        println!("Test {} alive", 1);

        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 2;
        stream.write_all(&data).unwrap();
        stream.flush().unwrap();

        let mut buffer = [255; SEND_PACKET_SIZE];
        stream.read(&mut buffer).expect("Sock error");
        println!("Server reply: to test1 {}", buffer[1]);
    }

    #[test]
    pub fn sock_test2() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 100 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
        println!("Test {} alive", 2);

        let data = vec![1; SEND_PACKET_SIZE];
        stream.write_all(&data).unwrap();
        stream.flush().unwrap();

        let mut buffer = [0; SEND_PACKET_SIZE];
        let size = stream.read(&mut buffer).expect("Sock error");
        let message = String::from_utf8_lossy(&buffer[..size]);
        println!("Server says: {} to test2", message);
    }

    #[test]
    pub fn sock_test3() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 200 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
        println!("Test {} alive", 3);

        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 3;
        stream.write_all(&data).unwrap();
        stream.flush().unwrap();

        let mut buffer = [0; SEND_PACKET_SIZE];
        stream.read(&mut buffer).expect("Sock error");
        println!("Server reply: to test3 {}", buffer[1]);
    }

    #[test]
    pub fn sock_test4() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 300 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        println!("Test {} alive", 4);

        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 4;

        let t = Instant::now();
        let mut ctr = 1;
        loop {
            let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
            stream.write_all(&data).unwrap();
            stream.flush().unwrap();

            let mut buffer = [0; SEND_PACKET_SIZE];
            stream.read(&mut buffer).expect("Sock error");
            println!("Server reply: to test4 {} {} [{}]", buffer[1], ctr, t.elapsed().as_micros() as f64 * 1E-3);

            match ctr >= 100 {
                true => {
                    return;
                }
                _ => {}
            }
            ctr += 1;
        }
    }

    #[test]
    pub fn sock_test5() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 300 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        println!("Test {} alive", 5);

        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 5;

        let t = Instant::now();
        let mut ctr = 1;
        loop {
            let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
            stream.write_all(&data).unwrap();
            stream.flush().unwrap();

            let mut buffer = [0; SEND_PACKET_SIZE];
            stream.read(&mut buffer).expect("Sock error");
            println!("Server reply: to test5 {} {} [{}]", buffer[1], ctr, t.elapsed().as_micros() as f64 * 1E-3);

            match ctr >= 100 {
                true => {
                    return;
                }
                _ => {}
            }
            ctr += 1;
        }
    }

    #[test]
    pub fn sock_test6() {

        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 1000 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        println!("Test {} alive", 6);

        let data = vec![13; SEND_PACKET_SIZE];

        let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
        stream.write_all(&data).unwrap();
        stream.flush().unwrap();

        let mut buffer = [0; SEND_PACKET_SIZE];
        stream.read(&mut buffer).expect("Sock error");
        println!("Server says: {} to test{} [{}ms]", buffer[1], 6, t.elapsed().as_micros() as f64 * 1E-3);
    }

    #[test]
    pub fn sock_test7() {
        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= 400 {
                true => {
                    break;
                }
                _ => {}
            }
        }

        println!("Test {} alive", 7);

        let mut data = vec![255; SEND_PACKET_SIZE];
        data[1] = 5;

        let t = Instant::now();
        let mut ctr = 1;
        loop {
            let mut stream = TcpStream::connect("127.0.0.1:1313").unwrap();
            stream.write_all(&data).unwrap();
            stream.flush().unwrap();

            let mut buffer = [0; SEND_PACKET_SIZE];
            stream.read(&mut buffer).expect("Sock error");
            println!("Server reply: to test7 {} {} [{}]", buffer[1], ctr, t.elapsed().as_micros() as f64 * 1E-3);

            match ctr >= 100 {
                true => {
                    return;
                }
                _ => {}
            }
            ctr += 1;
        }
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
    pub fn socet_server2() {
        sock_server_gen2();
    }


    pub fn sock_call_back() {
        
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
        sock_base(6, 5000, data);    }
}
