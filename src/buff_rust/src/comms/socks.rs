use std::{
    env,
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    sync::{Arc, RwLock,
        mpsc,
        mpsc::{Receiver, Sender},
    },
    thread::{Builder, JoinHandle},
    time::{Duration, Instant},
};

const TCP_PACKET_SIZE: usize = 1024;

#[derive(Clone)]
pub struct TcpControlFlags {
    pub shutdown: Arc<RwLock<bool>>,
}

pub struct FullDuplexChannel { // Sopic or Sockic, idk
    pub tx: Sender<[u8; TCP_PACKET_SIZE]>,    // data from the server
    pub rx: Receiver<[u8; TCP_PACKET_SIZE]>,  // data from the user
}

impl FullDuplexChannel {
    pub fn partner(tx: Sender<[u8; TCP_PACKET_SIZE]>, rx: Receiver<[u8; TCP_PACKET_SIZE]>) -> FullDuplexChannel {
        FullDuplexChannel {
            rx: rx,
            tx: tx,
        }
    }

    pub fn new() -> (FullDuplexChannel, Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) {
        let (partner_tx, rx): (Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) = mpsc::channel();
        let (tx, partner_rx): (Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) = mpsc::channel();
        (FullDuplexChannel::partner(tx, rx),
        partner_tx,
        partner_rx
        )
    }

    pub fn clone(&mut self) -> FullDuplexChannel {
        let (partner_tx, rx): (Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) = mpsc::channel();
        let (tx, partner_rx): (Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) = mpsc::channel();
        self.tx = tx;
        self.rx = rx;

        FullDuplexChannel::partner(partner_tx, partner_rx)
    }
}

pub struct Sockage {
    pub server: bool,
    pub client_id: u8,
    pub millis_rate: u32,

    pub fdc: FullDuplexChannel, 

    pub shutdown: Arc<RwLock<bool>>,
}

impl Sockage {
    pub fn new(client_id: u8, millis_rate: u32) -> (Sockage, Sender<[u8; TCP_PACKET_SIZE]>, Receiver<[u8; TCP_PACKET_SIZE]>) {
        let (fdc, user_tx, user_rx) = FullDuplexChannel::new();
        (Sockage {
            server: false,
            client_id: client_id,
            millis_rate: millis_rate,

            fdc: fdc,

            shutdown: Arc::new(RwLock::new(false)),
        },
        user_tx,
        user_rx,
        )
    }

    pub fn clone(&mut self) -> Sockage {
        Sockage {
            server: self.server,
            client_id: self.client_id,
            millis_rate: self.millis_rate,

            fdc: self.fdc.clone(),

            shutdown: self.shutdown.clone(),
        }
    }

    pub fn from_bytes(buffer: &[u8]) -> Sockage {
        let rate = f32::from_le_bytes([buffer[1020], buffer[1021], buffer[1022], buffer[1023]]);
        let millis_rate = ((1.0 / rate) * 1000.0) as u128;
        let (sock, _, _) = Sockage::new(buffer[1], millis_rate as u32);
        sock
    }

    pub fn receive(&self) -> [u8; TCP_PACKET_SIZE] {
        self.fdc.rx.try_recv().unwrap_or([0; TCP_PACKET_SIZE])
    }

    pub fn send(&self, buffer: [u8; TCP_PACKET_SIZE]) {
        self.fdc.tx.send(buffer).unwrap();
    }

    pub fn is_shutdown(&self) -> bool {
        *self.shutdown.read().unwrap()
    }

    pub fn shutdown(&self, status: bool) {
        *self.shutdown.write().unwrap() = status;
    }

    pub fn comms_packet(&self, write: bool) -> [u8; TCP_PACKET_SIZE] {
        let mut packet = [0; TCP_PACKET_SIZE];
        packet[0] = 255;
        packet[1] = self.client_id;
        match write {
            true => packet[2] = 1,
            _ => packet[2] = 0,
        }

        packet
    }

    pub fn shutdown_packet(&self) -> [u8; TCP_PACKET_SIZE] {
        let mut packet = [0; TCP_PACKET_SIZE];
        packet[0] = 13;
        packet[1] = self.client_id;
        packet
    }

    pub fn read_stream(&self, stream: &mut TcpStream, mut buffer: [u8; TCP_PACKET_SIZE]) -> usize {
        match stream.read(&mut buffer) {
            Ok(size) => size,
            Err(e) => {
                println!("[Sockage {}{}]: Read: {:?}", self.client_id, self.server as u8, e);
                0
            }
        }
    }

    pub fn read_and_push(&self, stream: &mut TcpStream) {
        let buffer = [0; TCP_PACKET_SIZE];
        match self.read_stream(stream, buffer) {
            TCP_PACKET_SIZE => match buffer[0] {
                13 => {
                    self.send(buffer);
                    self.shutdown(true);
                    println!("[Sockage {}{}]: Shutdown", self.client_id, self.server as u8);
                }

                255 => self.send(buffer),
                _ => {},
            },
            _ => {},// self.shutdown(true),
        }
    }

    pub fn write_stream(&self, stream: &mut TcpStream, buffer: [u8; TCP_PACKET_SIZE]) {
        match stream.write_all(&buffer) {
            Ok(_) => stream.flush().unwrap(),
            Err(e) => println!("[Sockage {}{}]: Write: {:?}", self.client_id, self.server as u8, e),
        }
    }

    pub fn write_and_push(&self, stream: &mut TcpStream) {
        let buffer = self.receive();
        match buffer[0] {
            0 => {},
            _ => {
                println!("[Sockage {}{}]: channel available", self.client_id, self.server as u8);
                self.write_stream(stream, buffer);
            }
        }
    }

    pub fn thread(&self, mut stream: TcpStream) {
        let lifetime = Instant::now();

        while !self.is_shutdown() {
            let t = Instant::now();
            self.read_and_push(&mut stream);
            self.write_and_push(&mut stream);
            while self.millis_rate as u128 > t.elapsed().as_millis() {}
        }

        self.write_stream(&mut stream, self.shutdown_packet());
        println!("[Sockage {}{}]: Shutdown {}", self.client_id, self.server as u8, lifetime.elapsed().as_micros() as f64 * 1E-6);
    }

    pub fn worker(mut sock: Sockage, stream: TcpStream, server: bool) -> JoinHandle<()> {
        sock.shutdown(false);
        sock.server = server;

        Builder::new()
            .name(format!("Sock-Work-{}", sock.client_id))
            .spawn(move || sock.thread(stream)).unwrap()
    }
}

impl Drop for Sockage {
    fn drop(&mut self) {
        self.shutdown(true);
    }
}

pub struct SockCore { // Shit, fool doesn't even have Sockage
    pub shutdown: bool,
    pub socks: Vec<Sockage>,
    // pub threads: Vec<JoinHandle<()>>,
    pub buffers: Vec<[u8; TCP_PACKET_SIZE]>,
}

impl SockCore {
    pub fn new() -> SockCore {
        SockCore {
            shutdown: false,
            socks: vec![],
            // threads: vec![],
            buffers: vec![],
        }
    }

    pub fn check_new_connections(&mut self, listener: &TcpListener) -> usize {
        match listener.accept() {
            Ok((mut stream, _)) => {
                let mut buffer = [0; TCP_PACKET_SIZE];

                match stream.read(&mut buffer) {
                    Ok(size) => {
                        match (size, buffer[0]) {
                            (TCP_PACKET_SIZE, 255) => {
                                // println!("[Sock-Core]: Sync request id {}, mode {}", buffer[1], buffer[2]);
                                let mut sock = Sockage::from_bytes(&buffer);
                                match self.socks.iter().position(|s| s.client_id == sock.client_id) {
                                    Some(idx) => {
                                        // println!("[Sock-Core]: New thread id {}, mode {}", buffer[1], buffer[2]);

                                        let sock = self.socks[idx].clone();
                                        sock.send(buffer);
                                        Sockage::worker(sock, stream, true);
                                    } 
                                    None => {
                                        println!("[Sock-Core]: New Sock id {}, mode {}", buffer[1], buffer[2]);

                                        let clone = sock.clone();
                                        clone.send(buffer);
                                        Sockage::worker(clone, stream, true);
                                        self.socks.push(sock);
                                        self.buffers.push([0; TCP_PACKET_SIZE]);
                                    }
                                }
                                return 1;
                            }

                            (TCP_PACKET_SIZE, 13) => {
                                println!("[Sock-Core]: Shutdown from {}", buffer[1]);
                            }

                            (_, _) => {},
                        }
                    }
                    _ => {
                        println!("[Sock-Core]: stream read failed");
                    },
                }
            }
            _ => {}, // println!("[Sock-Core]: No new connections"),
        }

        0
    }

    pub fn push_channels(&mut self) {
        // Send current sock data to assigned channels
        (0..self.socks.len()).for_each(|i| {
            let buffer = self.socks[i].receive();
            match (buffer[0], buffer[2]) {
                (255, 1) => {
                    self.buffers[i] = buffer;
                }
                (255, 0) => {
                    (0..buffer[3]).map(|i| buffer[4+i as usize] as usize).for_each(|j| {
                        match self.socks.iter().position(|sock| sock.client_id == j as u8) {
                            Some(j) => {
                                self.socks[i].send(self.buffers[j]);
                                println!("[Sock-Core]: forwarding {} -> {}", j, i);
                            }
                            None => {
                                println!("[Sock-Core]: Sock does not exist {}", j);
                            },
                        }
                    })
                }
                (13, _) => println!("[Sock-Core]: Shutdown from {}", buffer[1]),
                (_, _) => {},
            }
        });
    }

    pub fn shutdown_socks(&self) {
        println!("[Sock-Core]: Cleaning {} Socks", self.socks.len());

        self.socks.iter().for_each(|sock| {
            sock.shutdown(true);
        });
    }

    // pub fn join_threads(&mut self) { // idk man
    //     println!("[Server-Core]: Joining {} Threads", self.socks.len());

    //     let mut ctr = 0;
    //     while self.threads.len() > 0 {
    //         self.threads.remove(0).join().expect("[Sock-Core]: Thread failed");
    //         println!("Joined thread {}", ctr);
    //         ctr += 1;
    //     }
    // }

    pub fn start(&mut self, timeout: u64) {
        let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        listener
            .set_nonblocking(true)
            .expect("Cannot set non-blocking");

        println!(
            "[Sock-Core]: Bound to {:?}",
            env::var("DYSE_CORE_URI").unwrap()
        );

        let mut t = Instant::now();
        let lifetime = Instant::now();
        while !self.shutdown && t.elapsed().as_secs() < timeout {
            self.check_new_connections(&listener);
            self.socks.iter().for_each(|sock| self.shutdown &= sock.is_shutdown());

            self.push_channels();
        }

        self.shutdown_socks();
        // self.join_threads();
        println!("[Sock-Core]: Shutdown {}s", lifetime.elapsed().as_micros() as f64 * 1E-6);
    }
}

pub struct SockServer {}

impl SockServer {
    pub fn core() {
        let mut core = SockCore::new();
        core.start(5);
    }
}

pub struct SockClient {}

impl SockClient {
    pub fn sock_handle(sock: Sockage) {
        println!(
            "Client {} running every {}ms",
            sock.client_id, sock.millis_rate
        );
    }

    pub fn request_subscribe(client_id: u8, rate: f64, client_ids: Vec<u8>) -> Sockage {
        let millis_rate = (1.0 / rate) * 1000.0;
        let (mut sock, _, _) = Sockage::new(client_id, millis_rate as u32);
        let stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        stream
            .set_read_timeout(Some(Duration::new(0, sock.millis_rate)))
            .expect("set_read_timeout call failed");

        let clone = sock.clone();
        Sockage::worker(clone, stream, false);

        let mut buffer = sock.comms_packet(false);
        buffer[3] = client_ids.len() as u8;
        (0..client_ids.len()).for_each(|i| buffer[4+i] = client_ids[i]);

        sock.send(buffer);

        sock
    }

    pub fn request_delayed_subscribe(
        client_id: u8,
        rate: f64,
        client_ids: Vec<u8>,
        delay: u128,
    ) -> Sockage {
        let t = Instant::now();
        loop {
            match t.elapsed().as_millis() >= delay {
                true => {
                    break;
                }
                _ => {}
            }
        }

        SockClient::request_subscribe(client_id, rate, client_ids)
    }

    pub fn request_publish(client_id: u8, rate: f64, data: Vec<u8>) {
        let millis_rate = (1.0 / rate) * 1000.0;
        let (sock, _, _) = Sockage::new(client_id, millis_rate as u32);

        let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        stream
            .set_read_timeout(Some(Duration::new(0, sock.millis_rate)))
            .expect("set_read_timeout call failed");

        let mut buffer = sock.comms_packet(true);
        buffer[3] = data.len() as u8;
        (0..data.len()).for_each(|i| buffer[4+i] = data[i]);

        sock.write_stream(&mut stream, sock.comms_packet(true));
        
        sock.shutdown(true);
    }
}

// pub fn sock_base(test: u8, start: u128, data: Vec<u8>) {
//     let t = Instant::now();
//     loop {
//         match t.elapsed().as_millis() >= start {
//             true => {
//                 break;
//             }
//             _ => {}
//         }
//     }

//     println!("Client {} alive", test);

//     let millis_rate = 100;
//     let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
//     stream
//         .set_read_timeout(Some(Duration::new(0, millis_rate)))
//         .expect("set_read_timeout call failed");

//     stream.write_all(&data).unwrap();
//     stream.flush().unwrap();

//     let mut buffer = [0; TCP_PACKET_SIZE];
//     buffer[1] = test;
//     loop {
//         match stream.read(&mut buffer) {
//             Ok(size) => match size {
//                 TCP_PACKET_SIZE => {
//                     if buffer[0] == 13 {
//                         println!(
//                             "[Client] {}: Received Kill code {}",
//                             test,
//                             t.elapsed().as_millis() - start
//                         );
//                         break;
//                     }
//                     // maybe a callback here
//                 }
//                 _ => {}
//             },
//             Err(e) => println!("[Client] {}: {:?}", test, e),
//         }

//         stream.write_all(&data).unwrap();
//         stream.flush().unwrap();
//     }
// }

// pub fn sock_server_gen2() {
//     let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
//     listener
//         .set_nonblocking(true)
//         .expect("Cannot set non-blocking");

//     println!("Server Bound to {:?}", env::var("DYSE_CORE_URI").unwrap());

//     let mut threads = vec![];

//     let tcf = TcpControlFlags {
//         shutdown: Arc::new(RwLock::new(false)),
//     };

//     let mut t = Instant::now();

//     while !*tcf.shutdown.read().unwrap() {
//         if threads.len() > 0 {
//             t = Instant::now();
//         }

//         match listener.accept() {
//             Ok((stream, addr)) => {
//                 let tcfc = tcf.clone();

//                 threads.push(
//                     Builder::new()
//                         .name("[Stream-Listen]".to_string())
//                         .spawn(move || {
//                             listener_thread(tcfc, stream, addr);
//                         })
//                         .unwrap(),
//                 );
//             }
//             Err(_) => {}
//         }

//         threads
//             .iter()
//             .position(|thread| thread.is_finished())
//             .into_iter()
//             .for_each(|i| {
//                 threads.remove(i);
//             });

//         if t.elapsed().as_secs() > 2 {
//             break;
//         }
//     }

//     *tcf.shutdown.write().unwrap() = true;

//     println!("Joining {} threads", threads.len());

//     threads.into_iter().for_each(|thread| {
//         thread.join().expect("[Stream-Listen]: failed");
//     });

//     println!("what the fuck");
// }

// pub fn listener_thread(tcf: TcpControlFlags, mut stream: TcpStream, addr: SocketAddr) {
//     println!("[Server] {}: Listening", addr);
//     let mut buffer = [0; TCP_PACKET_SIZE];
//     let mut t = Instant::now();
//     let mut packet_writes = 0.0;
//     let mut packet_reads = 0.0;
//     let mut millis_rate = 0;

//     stream
//         .set_read_timeout(Some(Duration::new(0, 1)))
//         .expect("set_read_timeout call failed");

//     while !*tcf.shutdown.read().unwrap() && t.elapsed().as_secs() <= 5 {
//         match stream.read(&mut buffer) {
//             Ok(size) => match size {
//                 TCP_PACKET_SIZE => {
//                     packet_reads += 1.0;

//                     t = Instant::now();
//                     if buffer[0] == 255 {
//                         packet_writes += 1.0;

//                         let rate = f32::from_le_bytes([
//                             buffer[1020],
//                             buffer[1021],
//                             buffer[1022],
//                             buffer[1023],
//                         ]);
//                         millis_rate = ((1.0 / rate) * 1000.0) as u128;

//                         // let n_targets = buffer[2];
//                         // let targets: Vec<u8> = (0..n_targets).map(|i| buffer[(i+3) as usize]).collect();

//                         let mut reply_buffer = ByteBuffer::tcp();
//                         reply_buffer.put(0, 255);
//                         reply_buffer.put(1, 255);
//                         reply_buffer.put_floats(2, vec![packet_writes, packet_reads]);

//                         stream.write_all(&reply_buffer.buffer()).unwrap();
//                         stream.flush().unwrap();
//                     } else if buffer[0] == 13 {
//                         println!("[Server] {}: Received Kill code from {}", addr, buffer[1]);
//                         *tcf.shutdown.write().unwrap() = true;
//                         break;
//                     }
//                 }
//                 _ => {}
//             },
//             Err(e) => println!("[Server]: {:?}", e),
//         }

//         while t.elapsed().as_millis() < millis_rate {}
//     }
//     stream.write_all(&[13; TCP_PACKET_SIZE]).unwrap();
//     stream.flush().unwrap();
//     println!("[Server] {}: Sent kill code to {}", addr, buffer[1]);
// }
