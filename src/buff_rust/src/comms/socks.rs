use std::{
    env,
    io::{Read, Write},
    net::{TcpListener, TcpStream, SocketAddr}, 
    thread::{Builder, JoinHandle},
    sync::{Arc, RwLock},
    time::{Duration, Instant},
};

use crate::{utilities::{data_structures::*}};

const SEND_PACKET_SIZE: usize = 1024;


#[derive(Clone)]
pub struct TcpControlFlags {
    pub shutdown: Arc<RwLock<bool>>,
}

pub struct Sockage {
    pub client_id: u8,
    pub millis_rate: u32,
    pub targets: Arc<RwLock<Vec<u8>>>,
    pub thread: Option<JoinHandle<()>>,

    pub send_buffer: Arc<RwLock<ByteBuffer>>,
    pub receive_buffer: Arc<RwLock<ByteBuffer>>,

    pub mail_flag: Arc<RwLock<bool>>,
    pub shutdown: Arc<RwLock<bool>>,
}

impl Sockage {
    pub fn new(client_id: u8, millis_rate: u32)-> Sockage {
        Sockage {
            client_id: client_id,
            millis_rate: millis_rate,
            targets: Arc::new(RwLock::new(vec![])),
            thread: None,

            send_buffer: Arc::new(RwLock::new(ByteBuffer::tcp())),
            receive_buffer: Arc::new(RwLock::new(ByteBuffer::tcp())),
            
            mail_flag: Arc::new(RwLock::new(false)),
            shutdown: Arc::new(RwLock::new(false)),
        }
    }

    pub fn from_bytes(buffer: Vec<u8>) -> Sockage {
        let rate = f32::from_le_bytes([buffer[1020], buffer[1021], buffer[1022], buffer[1023]]);
        let millis_rate = ((1.0 / rate) * 1000.0) as u128;
        Sockage::new(buffer[1], millis_rate as u32)
    }

    pub fn clone(&self) -> Sockage {
        Sockage {
            client_id: self.client_id,
            millis_rate: self.millis_rate,
            thread: None,
            targets: self.targets.clone(),
            send_buffer: self.send_buffer.clone(),
            receive_buffer: self.receive_buffer.clone(),
            mail_flag: self.mail_flag.clone(),
            shutdown: self.shutdown.clone(),
        }
    }

    pub fn receive(&self, buffer: Vec<u8>) {
        self.receive_buffer.write().unwrap().puts(0, buffer);
        *self.mail_flag.write().unwrap() = true;
    }

    pub fn process(&self) {
        *self.mail_flag.write().unwrap() = false;
    }

    pub fn available(&self) -> bool {
        *self.mail_flag.read().unwrap()
    }

    pub fn is_shutdown(&self) -> bool {
        *self.shutdown.read().unwrap()
    }

    pub fn shutdown(&self) {
        *self.shutdown.write().unwrap() = true;
    }
}

impl Drop for Sockage {
    fn drop(&mut self) {
        self.shutdown();
    }
}

pub struct SockServer {}

impl SockServer {
    pub fn core() {
        let mut socks: Vec<Sockage> = vec![];
        let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        listener.set_nonblocking(true).expect("Cannot set non-blocking");

        println!("[Sock-Core]: Bound to {:?}", env::var("DYSE_CORE_URI").unwrap());

        let mut shutdown = false;
        let mut t = Instant::now();

        while !shutdown {
            if socks.len() > 0 {
                t = Instant::now();    
            }

            match listener.accept() {
                Ok((mut stream, addr)) => {

                    let mut buffer = [0; SEND_PACKET_SIZE];
                    match stream.read(&mut buffer) {
                        Ok(size) => match size {
                            SEND_PACKET_SIZE => {
                                let mut new_sock = Sockage::from_bytes(buffer.to_vec());
                                match socks.iter().find(|sock| sock.client_id == new_sock.client_id) {
                                    Some(_) => println!("Sock already has thread"),
                                    None => {
                                        if buffer[0] == 255 {
                                            let rate = f32::from_le_bytes([buffer[1020], buffer[1021], buffer[1022], buffer[1023]]);
                                            new_sock.millis_rate = ((1.0 / rate) * 1000.0) as u32;

                                            if buffer[2] != 64 {
                                                match new_sock.targets.read().unwrap().len() {
                                                    0 => {
                                                        *new_sock.targets.write().unwrap() = (0..buffer[2]).map(|i| buffer[(i+3) as usize]).collect();
                                                        println!("[Sock-Core]: {} {} new target {:?}", new_sock.client_id, buffer[2], new_sock.targets.read().unwrap());
                                                    }
                                                    _ => {},
                                                }
                                            }
                                        }
                                        let sock_clone = new_sock.clone();
                                        match Builder::new()
                                            .name("[Sock-Thread]".to_string())
                                            .spawn(move || {
                                                SockServer::thread(sock_clone, stream, addr);
                                            }) {
                                            Ok(thread) => new_sock.thread = Some(thread),
                                            _ => {}
                                        }
                                        socks.push(new_sock);

                                    }
                                }
                            }
                            _ => {},
                        }
                        _ => {},
                    }
                }
                Err(_) => {}
            }

            shutdown = socks.len() > 0;
            let mut dead_socks = vec![];
            (0..socks.len()).for_each(|i| {
                match socks[i].is_shutdown(){
                    true => {
                        dead_socks.push(i)
                    }
                    false => {
                        shutdown = false;
                        if socks[i].targets.read().unwrap().len() > 0 {
                            socks[i].targets.read().unwrap().iter().enumerate().for_each(|(j, target)| {
                                if (*target as usize) < socks.len() {
                                    println!("[Sock-Core]: {} -> {} available", j, i);
                                    socks[i].send_buffer.write().unwrap().puts(3 + (64 * j), socks[*target as usize].receive_buffer.read().unwrap().buffer()[3..67].to_vec());
                                }
                            });
                        }
                    },
                }
            });

            
            dead_socks.iter().enumerate().for_each(|(i, s)| {
                socks.remove(s - i);
            });

            if t.elapsed().as_secs() > 5 {
                break;
            }
            println!("[Sock-Core] {} {}", socks.len(), t.elapsed().as_millis() as f64 * 1E-3);
        }

        println!("[Server-Core]: Cleaning {} Socks", socks.len());

        socks.into_iter().for_each(|sock| {
            sock.shutdown();
        });
    }

    pub fn thread(sock: Sockage, mut stream: TcpStream, addr: SocketAddr) {
        println!("[Sock-Thread] {}: Listening", addr);
        let mut buffer = [0; SEND_PACKET_SIZE];
        let mut t = Instant::now();
        stream.set_read_timeout(Some(Duration::new(0, 1))).expect("set_read_timeout call failed");
        
        while !sock.is_shutdown() && t.elapsed().as_millis() <= (50 * sock.millis_rate) as u128 {
            
            match stream.read(&mut buffer) {
                Ok(size) => match size {
                    SEND_PACKET_SIZE => {
                        t = Instant::now();
                        match buffer[0] {
                            255 => {
                                sock.receive(buffer[0..64].to_vec())
                            }
                            13 => {
                                println!("[Sock-thread] {}: Received Kill code from {}", addr, buffer[1]);
                                sock.shutdown();
                                break;
                            }

                            _ => {}
                        }
                    }
                    _ => {}
                }
                Err(e) => println!("[Sock-Thread] {}: {:?}", addr, e),
            }


            match stream.write_all(sock.send_buffer.read().unwrap().buffer()) {
                Ok(_) => stream.flush().unwrap(),
                _ => {},
            }

            while t.elapsed().as_millis() < sock.millis_rate as u128 {}

        }

        match stream.write_all(&[13; SEND_PACKET_SIZE]) {
            Ok(_) => stream.flush().unwrap(),
            _ => {},
        }
        println!("[Sock-Thread] {}: Sent kill code to {}", addr, buffer[1]);
    }
}

pub struct SockClient {}

impl SockClient {
    pub fn sock_handle(sock: Sockage) {

        println!("Client {} running every {}ms", sock.client_id, sock.millis_rate);

        let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        stream.set_read_timeout(Some(Duration::new(0, sock.millis_rate))).expect("set_read_timeout call failed");

        println!("Sock targets: {:?}", sock.targets.read().unwrap());
        sock.send_buffer.write().unwrap().puts(0, [255, sock.client_id, sock.targets.read().unwrap().len() as u8].into_iter().chain(sock.targets.read().unwrap().iter().map(|t| *t)).collect());
        sock.send_buffer.write().unwrap().put_float(1020, (1.0 / sock.millis_rate as f64) * 1000.0);

        stream.write_all(sock.send_buffer.read().unwrap().buffer()).unwrap();
        stream.flush().unwrap();

        while !sock.is_shutdown() {
            let t = Instant::now();
            let mut buffer = [0; SEND_PACKET_SIZE];
            match stream.read(&mut buffer) {
                Ok(size) => match size {
                    SEND_PACKET_SIZE => {
                        sock.receive(buffer.to_vec());
                        if sock.receive_buffer.read().unwrap().get(0) == 13 {
                            println!("[Client] {}: Received Kill code", sock.client_id);
                            break;
                        }
                        // maybe a callback here
                    }
                    _ => {}
                }
                Err(e) => println!("[Client] {}: {:?}", sock.client_id, e),
            }

            stream.write_all(sock.send_buffer.read().unwrap().buffer()).unwrap();
            stream.flush().unwrap();

            while (t.elapsed().as_millis() as u32) < sock.millis_rate {}
        }

        sock.shutdown();
    }

    pub fn request_subscribe(client_id: u8, rate: f64, client_ids: Vec<u8>) -> Sockage {
        let millis_rate = (1.0 / rate) * 1000.0;
        let mut sock = Sockage::new(client_id, millis_rate as u32);
        let sock_clone = sock.clone();
        *sock.targets.write().unwrap() = client_ids;

        match Builder::new()
                    .name(format!("[Subscriber{}]", sock.client_id))
                    .spawn(move || {
                        SockClient::sock_handle(sock_clone);
                    }) {
                        Ok(thread) => sock.thread = Some(thread),
                        _ => {}
                    }
        sock
    }

    pub fn request_delayed_subscribe(client_id: u8, rate: f64, client_ids: Vec<u8>, delay: u128) -> Sockage {
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
        let sock = Sockage::new(client_id, millis_rate as u32);
        sock.send_buffer.write().unwrap().puts(0, [255, sock.client_id, 64].into_iter().chain(data.into_iter().take(64)).collect());
        sock.send_buffer.write().unwrap().put_float(1020, rate);

        let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        stream.set_read_timeout(Some(Duration::new(0, sock.millis_rate))).expect("set_read_timeout call failed");

        stream.write_all(sock.send_buffer.read().unwrap().buffer()).unwrap();
        stream.flush().unwrap();

        sock.shutdown();
        SockClient::terminate(&mut stream);

    }

    pub fn terminate(stream: &mut  TcpStream) {

        stream.write_all(&[13; SEND_PACKET_SIZE]).unwrap();
        stream.flush().unwrap();
    }
}



pub fn sock_base(test: u8, start: u128, data: Vec<u8>) {

    let t = Instant::now();
    loop {
        match t.elapsed().as_millis() >= start {
            true => {
                break;
            }
            _ => {}
        }
    }

    println!("Client {} alive", test);

    let millis_rate = 100;
    let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
    stream.set_read_timeout(Some(Duration::new(0, millis_rate))).expect("set_read_timeout call failed");

    stream.write_all(&data).unwrap();
    stream.flush().unwrap();

    let mut buffer = [0; SEND_PACKET_SIZE];
    buffer[1] = test;
    loop {
        match stream.read(&mut buffer) {
            Ok(size) => match size {
                SEND_PACKET_SIZE => {
                    if buffer[0] == 13 {
                        println!("[Client] {}: Received Kill code {}", test, t.elapsed().as_millis() - start);
                        break;
                    }
                    // maybe a callback here
                }
                _ => {}
            }
            Err(e) => println!("[Client] {}: {:?}", test, e),
        }

        stream.write_all(&data).unwrap();
        stream.flush().unwrap();
    }
}

pub fn sock_server_gen2() {
    let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
    listener.set_nonblocking(true).expect("Cannot set non-blocking");

    println!("Server Bound to {:?}", env::var("DYSE_CORE_URI").unwrap());

    let mut threads = vec![];

    let tcf = TcpControlFlags{shutdown: Arc::new(RwLock::new(false))};

    let mut t = Instant::now();

    while !*tcf.shutdown.read().unwrap() {
        if threads.len() > 0 {
            t = Instant::now();    
        }

        match listener.accept() {
            Ok((stream, addr)) => {
                let tcfc = tcf.clone();

                threads.push(Builder::new()
                    .name("[Stream-Listen]".to_string())
                    .spawn(move || {
                        listener_thread(tcfc, stream, addr);
                    })
                    .unwrap());

            }
            Err(_) => {}
        }

        threads.iter().position(|thread| thread.is_finished()).into_iter().for_each(|i| {
            threads.remove(i);
        });

        if t.elapsed().as_secs() > 2 {
            break;
        }
    }

    *tcf.shutdown.write().unwrap() = true;

    println!("Joining {} threads", threads.len());

    threads.into_iter().for_each(|thread| {
        thread.join().expect("[Stream-Listen]: failed");
    });

    println!("what the fuck");
}

pub fn listener_thread(tcf: TcpControlFlags, mut stream: TcpStream, addr: SocketAddr) {
    println!("[Server] {}: Listening", addr);
    let mut buffer = [0; SEND_PACKET_SIZE];
    let mut t = Instant::now();
    let mut packet_writes = 0.0;
    let mut packet_reads = 0.0;
    let mut millis_rate = 0;

    stream.set_read_timeout(Some(Duration::new(0, 1))).expect("set_read_timeout call failed");
    
    while !*tcf.shutdown.read().unwrap() && t.elapsed().as_secs() <= 5 {
        match stream.read(&mut buffer) {
            Ok(size) => match size {
                SEND_PACKET_SIZE => {
                    packet_reads += 1.0;

                    t = Instant::now();
                    if buffer[0] == 255 {
                        packet_writes += 1.0;

                        let rate = f32::from_le_bytes([buffer[1020], buffer[1021], buffer[1022], buffer[1023]]);
                        millis_rate = ((1.0 / rate) * 1000.0) as u128;

                        // let n_targets = buffer[2];
                        // let targets: Vec<u8> = (0..n_targets).map(|i| buffer[(i+3) as usize]).collect();

                        let mut reply_buffer = ByteBuffer::tcp();
                        reply_buffer.put(0, 255);
                        reply_buffer.put(1, 255);
                        reply_buffer.put_floats(2, vec![packet_writes, packet_reads]);
                        
                        stream.write_all(&reply_buffer.buffer()).unwrap();
                        stream.flush().unwrap();

                    } else if buffer[0] == 13 {
                        println!("[Server] {}: Received Kill code from {}", addr, buffer[1]);
                        *tcf.shutdown.write().unwrap() = true;
                        break;
                    }
                }
                _ => {}
            }
            Err(e) => println!("[Server]: {:?}", e),
        }

        while t.elapsed().as_millis() < millis_rate {}

    }
    stream.write_all(&[13; SEND_PACKET_SIZE]).unwrap();
    stream.flush().unwrap();
    println!("[Server] {}: Sent kill code to {}", addr, buffer[1]);
}