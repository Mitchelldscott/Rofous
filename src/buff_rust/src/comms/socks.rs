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

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket},
    sync::{
        // mpsc,
        // mpsc::{Receiver, Sender},
        Arc,
        RwLock,
    },
    // thread::{Builder, JoinHandle},
    time::{Duration, Instant},
};

pub const UDP_PACKET_SIZE: usize = 1024;
type UdpPacket = [u8; UDP_PACKET_SIZE];

macro_rules! sock_uri {
    () => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 0) }};
    ($port:expr) => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), $port) }};
    ($ip1:expr, $ip2:expr, $ip3:expr, $ip4:expr, $port:expr) => {{ SocketAddr::new(IpAddr::V4(Ipv4Addr::new($ip1, $ip2, $ip3, $ip4)), $port) }};
}

pub fn dump_string(idx: usize, data: &str, buffer: &mut UdpPacket) -> usize {
    let data = data.as_bytes();
    let data_len = data.len();
    buffer[idx] = data_len as u8;
    buffer[idx+1..data_len+idx+1].copy_from_slice(&data);
    data_len+idx+1
}

pub fn parse_string(idx: usize, buffer: &UdpPacket) -> String {
    let str_len = buffer[idx] as usize;
    match String::from_utf8(buffer[idx+1..str_len+idx+1].to_vec()) {
        Ok(s) => s,
        Err(_) => String::new(),
    }
}

pub fn sender_name(buffer: &UdpPacket) -> (usize, String) {
    ((buffer[0]+1) as usize, parse_string(0, buffer))
}

pub fn read_all_names(buffer: &UdpPacket) -> (String, u8, Vec<String>) {
    let (mut next_idx, sender_name) = sender_name(buffer);
    let mut names = vec![];

    let mode = buffer[next_idx];
    let n_names = buffer[next_idx+1] as usize;
    next_idx += 2;
    
    (0..n_names).for_each(|_| {
        let name = parse_string(next_idx, buffer);
        next_idx += (buffer[next_idx]+1) as usize;
        names.push(name);
    });

    (sender_name, mode, names)
}

pub fn dump_all_names(mut idx: usize, buffer: &mut UdpPacket, names: &Vec<String>) {
    buffer[idx] = names.len() as u8;
    idx += 1;
    names.iter().for_each(|name| {
        idx = dump_string(idx, name, buffer);
    })
}

pub fn dump_addr(idx: usize, buffer: &mut UdpPacket, addr: &SocketAddr) -> usize {
    let mut octets = match addr.ip() {
        IpAddr::V4(ip) => ip.octets().to_vec(),
        _ => {vec![0, 0, 0, 0]}
    };
    octets.append(&mut addr.port().to_be_bytes().to_vec());
    buffer[idx..idx+6].copy_from_slice(&octets);
    idx + 6
}

pub fn parse_addr(idx: usize, buffer: &UdpPacket) -> SocketAddr {
    sock_uri!(buffer[idx], buffer[idx+1], buffer[idx+2], buffer[idx+3], u16::from_be_bytes([buffer[idx+4], buffer[idx+5]]))
}

pub fn read_all_addrs(buffer: &UdpPacket) -> (String, u8, Vec<SocketAddr>) {
    let (mut next_idx, sender_name) = sender_name(buffer);
    let mut addrs = vec![];

    let mode = buffer[next_idx];
    let n_addrs = buffer[next_idx+1] as usize;
    next_idx += 2;
    
    (0..n_addrs).for_each(|_| {
        let addr = parse_addr(next_idx, buffer);
        next_idx += 6;
        addrs.push(addr);
    });

    (sender_name, mode, addrs)
}

pub fn dump_all_addrs(mut idx: usize, buffer: &mut UdpPacket, ips: &Vec<SocketAddr>) {
    buffer[idx] = ips.len() as u8;
    idx += 1;
    ips.iter().for_each(|ip| {
        idx = dump_addr(idx, buffer, ip);
    });
}

pub struct Sockage {
    pub name: String,
    pub lifetime: Instant,
    pub millis_rate: u128,
    pub send_count: u128,
    pub recv_count: u128,

    pub socket: UdpSocket,

    pub shutdown: Arc<RwLock<bool>>,

    pub addrs: Vec<SocketAddr>,
    pub names: Vec<String>,
}

impl Sockage {
    pub fn new(name: &str, addr: SocketAddr, rate: u128) -> Sockage {
        Sockage {
            name: name.to_string(),
            lifetime: Instant::now(),
            millis_rate: rate,
            send_count: 0,
            recv_count: 0,

            socket: Sockage::new_socket(addr, rate as u32),

            shutdown: Arc::new(RwLock::new(false)),

            addrs: vec![],
            names: vec![],
        }
    }

    pub fn new_socket(addr: SocketAddr, timeout: u32) -> UdpSocket {
        let socket = UdpSocket::bind(addr).expect("Couldn't bind to socket");
        socket
            .set_write_timeout(Some(Duration::new(0, timeout)))
            .unwrap();
        socket
            .set_read_timeout(Some(Duration::new(0, timeout)))
            .unwrap();
        socket
    }

    pub fn core(name: &str) -> Sockage {
        Sockage::new(name, sock_uri!(1313), 1)
    }

    pub fn client(name: &str, rate: u128) -> Sockage {
        Sockage::new(name, sock_uri!(0,0,0,0,0), rate)
    }

    pub fn clear_registry(&mut self) {
        self.addrs.clear();
        self.names.clear();
    }

    pub fn find_name(&self, target: &str) -> Option<usize> {
        self.names.iter().position(|name| name == target)
    }

    pub fn find_address(&self, target: SocketAddr) -> Option<usize> {
        self.addrs.iter().position(|addr| *addr == target)
    }

    pub fn adress_of_names(&self, names: Vec<String>) -> Vec<SocketAddr> {
        names.iter().map(|name| {
            match self.find_name(name) {
                Some(i) => self.addrs[i],
                None => sock_uri!(0,0,0,0,0),
            }
        }).collect()
    }

    pub fn discover_sock(&mut self, name: String, addr: &SocketAddr) {
        while self.addrs.len() < self.names.len() {
            self.addrs.push(sock_uri!(0,0,0,0,0));
        }

        match self.find_name(&name) {
            Some(i) => self.addrs[i] = *addr,

            _ => {
                self.addrs.push(*addr);
                self.names.push(name);
            }, 
        }
    }

    pub fn stamp_packet(&self) -> UdpPacket {
        let mut buffer = [0; UDP_PACKET_SIZE];
        dump_string(0, &self.name, &mut buffer);
        buffer
    }

    pub fn names_packet(&self, mode: u8) -> UdpPacket {
        let mut buffer = self.stamp_packet();
        buffer[(buffer[0]+1) as usize] = mode;
        dump_all_names((buffer[0]+2) as usize, &mut buffer, &self.names);
        buffer
    }

    pub fn addrs_packet(&self, mode: u8, addrs: Vec<SocketAddr>) -> UdpPacket {
        let mut buffer = self.stamp_packet();
        buffer[(buffer[0]+1) as usize] = mode;  // 0-1 tells receiver if requesting or insisting
        dump_all_addrs((buffer[0]+2) as usize, &mut buffer, &addrs);

        buffer
    }

    pub fn data_request_packet(&self, data: Vec<f64>) -> UdpPacket {
        let mut buffer = self.stamp_packet();
        let data_start = (buffer[0]+1) as usize;
        buffer[data_start] = 2;             // force write mode for data packets
        buffer
    }

    pub fn data_packet(&self, data: Vec<f64>) -> UdpPacket {
        let mut buffer = self.stamp_packet();
        let data_start = (buffer[0]+1) as usize;
        let bytes: Vec<u8> = data.iter().map(|x| (*x as f32).to_be_bytes()).flatten().collect();

        buffer[data_start] = 2;             // force write mode for data packets
        buffer[data_start+1] = bytes.len() as u8; // data length for parsing
        buffer[data_start+2..data_start+bytes.len()+2].copy_from_slice(&bytes);
        buffer
    }

    pub fn send_to(&mut self, mut buffer: UdpPacket, addr: SocketAddr) -> bool {
        match !self.is_shutdown() {
            true => match self.socket.send_to(&mut buffer, addr) {
                Ok(_) => {
                    self.send_count += 1;
                    true
                }
                Err(_) => false,
            },

            false => false,
        }
    }

    pub fn recv(&mut self, buffer: &mut UdpPacket) -> SocketAddr {
        match !self.is_shutdown() {
            true => match self.socket.recv_from(buffer) {
                Ok((_size, src_addr)) => {
                    self.recv_count += 1;
                    src_addr
                }

                Err(_) => sock_uri!(0,0,0,0,0),
            },
            false => sock_uri!(0,0,0,0,0),
        }
    }

    pub fn is_shutdown(&self) -> bool {
        *self.shutdown.read().unwrap()
    }

    pub fn shutdown(&self, status: bool) {
        *self.shutdown.write().unwrap() = status;
    }

    pub fn log<T: std::fmt::Debug>(&self, message: T) {
        println!(
            "[{:?}]:{:?}-{}<{},{}>\t\t{:?}\t({}s)",
            self.name,
            self.socket.local_addr().unwrap(),
            self.is_shutdown(),
            self.send_count,
            self.recv_count,
            message,
            self.lifetime.elapsed().as_micros() as f64 * 1E-6,
        );
    }

    pub fn log_heavy(&self) {
        println!(
            "==[Sockage]==\n[{:?}]:{}-{}<{},{}>\t({}s)\nSocks:\n\t{:?}\t\n\t{:?}",
            self.name,
            self.socket.local_addr().unwrap(),
            self.is_shutdown(),
            self.send_count,
            self.recv_count,
            self.lifetime.elapsed().as_micros() as f64 * 1E-6,
            self.names,
            self.addrs,
        );
    }
}

// pub struct FullDuplexChannel {
//     // Sopic or Sockic, idk
//     pub tx: Sender<UdpPacket>, // data from the server
//     pub rx: Receiver<UdpPacket>, // data from the user
// }

// impl FullDuplexChannel {
//     pub fn partner(
//         tx: Sender<UdpPacket>,
//         rx: Receiver<UdpPacket>,
//     ) -> FullDuplexChannel {
//         FullDuplexChannel { rx: rx, tx: tx }
//     }

//     pub fn new() -> (
//         FullDuplexChannel,
//         Sender<[u8; UDP_PACKET_SIZE]>,
//         Receiver<[u8; UDP_PACKET_SIZE]>,
//     ) {
//         let (partner_tx, rx): (
//             Sender<[u8; UDP_PACKET_SIZE]>,
//             Receiver<[u8; UDP_PACKET_SIZE]>,
//         ) = mpsc::channel();
//         let (tx, partner_rx): (
//             Sender<[u8; UDP_PACKET_SIZE]>,
//             Receiver<[u8; UDP_PACKET_SIZE]>,
//         ) = mpsc::channel();
//         (FullDuplexChannel::partner(tx, rx), partner_tx, partner_rx)
//     }

//     pub fn clone(&mut self) -> FullDuplexChannel {
//         let (partner_tx, rx): (
//             Sender<[u8; UDP_PACKET_SIZE]>,
//             Receiver<[u8; UDP_PACKET_SIZE]>,
//         ) = mpsc::channel();
//         let (tx, partner_rx): (
//             Sender<[u8; UDP_PACKET_SIZE]>,
//             Receiver<[u8; UDP_PACKET_SIZE]>,
//         ) = mpsc::channel();
//         self.tx = tx;
//         self.rx = rx;

//         FullDuplexChannel::partner(partner_tx, partner_rx)
//     }
// }
