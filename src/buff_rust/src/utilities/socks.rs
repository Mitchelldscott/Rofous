use std::{
    env,
    io::{Read, Write},
    net::{TcpListener, TcpStream}, 
    thread::Builder,
    sync::{Arc, RwLock},
    time::{Duration, Instant},
};

const SEND_PACKET_SIZE: usize = 1024;

pub fn sock_server_gen1() {
    let mut read_count: i64 = 0;
    let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
    println!("Server Bound");

    for stream in listener.incoming() {

        let mut stream = stream.unwrap();
        let mut buffer = [0; SEND_PACKET_SIZE];

        match stream.read(&mut buffer).expect("Sock error") {
            SEND_PACKET_SIZE => {
                if buffer[0] == 255 {
                    println!("This is the report header {}-{}", buffer[1], read_count+1);
                    stream.write_all(&buffer).unwrap();
                    stream.flush().unwrap();
                } else if buffer[0] == 13 {
                    stream.write_all(&buffer).unwrap();
                    println!("Server Kill code");
                    break;
                }
                else {
                    println!("{} {:?}", read_count, buffer);
                    stream.write_all(b"Hello, client").unwrap();
                    stream.flush().unwrap();
                }
            }
            _ => {}
        } 
        read_count += 1;
    }
    println!("what the fuck");
}

#[derive(Clone)]
pub struct TcpControlFlags {
    pub shutdown: Arc<RwLock<bool>>,
}

pub fn listener_thread(tcf: TcpControlFlags, mut stream: TcpStream) {
    println!("Listener Active");
    let mut buffer = [0; SEND_PACKET_SIZE];
    let mut t = Instant::now();

    stream.set_read_timeout(Some(Duration::new(1, 0))).expect("set_read_timeout call failed");
    
    while !*tcf.shutdown.read().unwrap() && t.elapsed().as_secs() <= 5 {
        match stream.read(&mut buffer) {
            Ok(size) => match size {
                SEND_PACKET_SIZE => {
                    t = Instant::now();
                    if buffer[0] == 255 {
                        stream.write_all(&buffer).unwrap();
                        stream.flush().unwrap();
                    } else if buffer[0] == 13 {
                        println!("[Server]: Received Kill code from {}", buffer[1]);
                        *tcf.shutdown.write().unwrap() = true;
                        stream.shutdown(std::net::Shutdown::Both).unwrap();
                        break;
                    }
                }
                _ => {}
            }
            Err(e) => println!("[Server]: {:?}", e),
        }
    }
    stream.write_all(&[13; SEND_PACKET_SIZE]).unwrap();
    stream.flush().unwrap();
    println!("[Server]: Sent kill code to {}", buffer[1]);
}

pub fn sock_server_gen2() {
    let listener = TcpListener::bind(env::var("DYSE_CORE_URI").unwrap()).unwrap();
    listener.set_nonblocking(true).expect("Cannot set non-blocking");

    println!("Server Bound to {:?}", env::var("DYSE_CORE_URI").unwrap());

    let mut threads = vec![];

    let tcf = TcpControlFlags{shutdown: Arc::new(RwLock::new(false))};

    while !*tcf.shutdown.read().unwrap() {
        let t = Instant::now();

        match listener.accept() {
            Ok((stream, _)) => {
                let tcfc = tcf.clone();

                threads.push(Builder::new()
                    .name("TCP Server".to_string())
                    .spawn(move || {
                        listener_thread(tcfc, stream);
                    })
                    .unwrap());

            }
            Err(_) => {}
        }

        let finished = threads.iter().position(|thread| thread.is_finished());
        finished.into_iter().for_each(|i| {
            threads.remove(i);
        });

        while t.elapsed().as_millis() > 1 {

        }
    }

    println!("Destroying {} threads", threads.len());

    threads.into_iter().for_each(|thread| {
        thread.join().expect("[Stream-thread]: failed");
    });

    println!("what the fuck");
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

        let mut stream = TcpStream::connect(env::var("DYSE_CORE_URI").unwrap()).unwrap();
        stream.set_read_timeout(Some(Duration::new(1, 0))).expect("set_read_timeout call failed");

        stream.write_all(&data).unwrap();
        stream.flush().unwrap();

        let mut buffer = [0; SEND_PACKET_SIZE];
        loop {
            match stream.read(&mut buffer) {
                Ok(size) => match size {
                    SEND_PACKET_SIZE => {
                        if buffer[0] == 13 {
                            break;
                        }
                        // maybe a callback here
                    }
                    _ => {}
                }
                Err(e) => break,
            }

        }
        println!("[Client] {}: Received Kill code {}", test, t.elapsed().as_millis() - start);
    }