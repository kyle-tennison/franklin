#![allow(dead_code)]
#![allow(unused_variables)]

use std::{
    collections::{HashMap, VecDeque},
    env,
    io::{self, Read, Write},
    net::TcpStream,
    thread::{self, sleep},
    time::{Duration, Instant},
};

use term_size::dimensions;

const ESP_ADDR: &str = "192.168.4.1:80";
const HEADER_BYTE: u8 = 0x46;
const GYRO_GRAPH_LIM: u8 = 45;

#[derive(Clone)]
enum Operation {
    Message = 0,
    Update = 1,
    Ping = 2,
    StatusRequest = 3,
}

#[derive(Clone, Debug)]
enum VariableUpdateTarget {
    PidProportional = 0,
    PidIntegral = 1,
    PidDerivative = 2,
    LinearVelocity = 3,
    AngularVelocity = 4,
    MotorEnabled = 5,
    GyroOffset = 6,
}

struct FranklinClient {
    socket: TcpStream,
}
impl FranklinClient {
    fn new(esp_address: &str) -> FranklinClient {
        let socket = TcpStream::connect(esp_address).unwrap();
        FranklinClient { socket }
    }

    /// breaks a big integer into a vector of u8 whose sum is the big int
    fn break_big_int(&self, mut int: usize) -> Vec<u8> {
        let mut container = Vec::new();

        while int > 0 {
            if int >= 255 {
                container.push(255);
                int -= 255;
            } else {
                container.push(int.try_into().unwrap());
                int = 0;
            }
        }
        container
    }

    /// creates a header from operation and content length
    fn get_header(&mut self, operation: Operation, content_length: usize) -> Vec<u8> {
        // println!("debug: creating header for operation code {} with length {}", operation.clone() as usize, content_length);
        let op_code = operation as u8;
        let mut header: Vec<u8> = vec![HEADER_BYTE, HEADER_BYTE, op_code];
        header.append(&mut self.break_big_int(content_length));
        header.push(0);
        header
    }

    /// sends a generic message
    fn send_message(&mut self, message: String) {
        let mut header = self.get_header(Operation::Message, message.len());
        header.extend_from_slice(message.as_bytes());

        // println!("debug: sending message -- {:?}", &header);

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => println!("error: failed to send message -- {err}"),
        };
    }

    /// times ping
    fn send_ping(&mut self) -> u128 {
        let start = Instant::now();
        let ping_content = Vec::from("p i n g g g");
        let mut header = self.get_header(Operation::Ping, ping_content.len());
        header.append(&mut ping_content.clone());
        // println!("debug: sending ping -- {:?}", &header);

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => println!("error: failed ping server -- {err}"),
        };

        let mut ping_response: Vec<u8> = vec![0; ping_content.len()];
        self.socket.read_exact(&mut ping_response).unwrap();

        for i in 0..ping_response.len() {
            if ping_content[i] != ping_response[i] {
                println!("error: echo is not equal");
                println!(" {:?} != {:?}", ping_content, ping_response);
                println!(" {:?} != {:?}", ping_content[i], ping_response[i]);
                panic!();
            }
        }
        // println!("debug: outbound ping and incoming are\nout: {:?}\nin:  {:?}", ping_content, ping_response);

        let elapsed = (Instant::now() - start).as_millis();
        println!("info: pinged server in {elapsed} milliseconds");
        elapsed
    }

    /// requests general status info
    fn poll_status(&mut self, show: bool) -> HashMap<String, f32> {
        let mut header = self.get_header(Operation::StatusRequest, 1);

        header.push(1); // add a random byte so we don't have an empty packet

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => panic!("error: failed to send status request: {err}"),
        }

        let mut status_response: VecDeque<u8> = VecDeque::new();
        let mut content_len: usize = 0;
        let mut content_len_determined = false;

        let mut byte_index = 0;
        loop {
            let mut recv_buf: [u8; 1] = [0];

            match self.socket.read(&mut recv_buf) {
                Ok(1) => (),
                Ok(n) => {
                    panic!("error: read {n} bytes while intending to read one");
                }
                Err(err) => {
                    panic!("error: failed to read from status response: {err}");
                }
            }

            let recv = recv_buf[0];

            // println!("recv: {recv}");

            if byte_index <= 1 {
                if recv != HEADER_BYTE {
                    println!("error: byte {recv} is not a header byte");
                    continue;
                }
            } else if byte_index == 2 {
                // ignore the operation byte
            } else if byte_index == 3 {
                content_len = recv.into()
            } else if byte_index > 3 {
                if !content_len_determined {
                    content_len += usize::from(recv);

                    if recv == 0 {
                        content_len_determined = true;
                    }
                } else {
                    status_response.push_back(recv);

                    if status_response.len() == content_len {
                        // println!("debug: done collecting status");
                        break;
                    }
                }
            }

            byte_index += 1;
        }

        let map = self.deserialize_status(&mut status_response);

        if show {
            println!("Status response: {{");
            let mut sorted_pairs: Vec<_> = map.iter().collect();
            sorted_pairs.sort_by(|a, b| a.0.cmp(b.0));
            for (key, value) in sorted_pairs {
                println!("\t{}: {}", key, value);
            }
            println!("}}");
        }

        map
    }

    /// Deserializes the status response from ESP into a hash map
    fn deserialize_status(&self, status_response: &mut VecDeque<u8>) -> HashMap<String, f32> {
        let mut map: HashMap<String, f32> = HashMap::new();

        assert!(
            *status_response.back().unwrap() == 0,
            "status response must end in zero"
        );

        while !status_response.is_empty() {
            let key = status_response.pop_front().unwrap();
            let b0 = status_response.pop_front().unwrap();
            let b1 = status_response.pop_front().unwrap();
            let end = status_response.pop_front().unwrap();

            assert_eq!(end, 0, "end must be zero");

            let value_raw = (((b0 as u16) << 8) + b1 as u16) as i16;
            let mut value = f32::from(value_raw);

            let identifier = match key {
                0 => "PID Proportional",
                1 => "PID Integral",
                2 => "PID Derivative",
                3 => "Motors Enabled",
                4 => "Gyro Offset",
                5 => "Gyro Value",
                6 => "Integral Sum",
                7 => "Motor Target",
                _ => panic!("failed to deserialize status response: unknown key"),
            }
            .to_string();

            // fix caveats
            if identifier == "Gyro Value" {
                value /= 100.;
            } else if identifier == "Gyro Offset" {
                value -= 128.;
                value /= 10.;
            }

            map.insert(identifier, value);
        }
        map
    }

    /// sends a variable update
    fn send_update(&mut self, target: VariableUpdateTarget, value: usize) {
        let mut update = vec![target.clone() as u8];
        update.append(&mut self.break_big_int(value));
        update.push(0);
        let mut header = self.get_header(Operation::Update, update.len());
        header.append(&mut update);
        // println!("debug: sending update -- {:?}", &header);

        println!("info: updating {:?} to value {}", &target, value);

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => println!("error: failed to send update -- {err}"),
        };
    }
}

struct PythonClient {
    socket: TcpStream,
}
impl PythonClient {
    fn new(python_address: &str) -> PythonClient {
        let socket = TcpStream::connect(python_address).unwrap();

        PythonClient { socket }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut franklin_container: Option<FranklinClient> = None;

    if args.len() > 1 && args[1].to_lowercase() == "skip" {
        println!("info: skipping franklin login")
    } else {
        println!("info: connecting to franklin...");
        let franklin = FranklinClient::new(ESP_ADDR);
        franklin_container = Some(franklin);
        let franklin = match &mut franklin_container {
            Some(franklin) => franklin,
            None => panic!("lost reference to franklin during setup")
        };

        thread::sleep(Duration::from_millis(1000));
        println!("info: connected to clientet");

        franklin.send_message("hello from rust".to_string());

        for _ in 0..2 {
            franklin.send_ping();
            thread::sleep(Duration::from_millis(1000));
        }

        franklin.poll_status(true);

        // Setup default values
        franklin.send_update(VariableUpdateTarget::PidProportional, 29);
        franklin.send_update(VariableUpdateTarget::PidIntegral, 81);
        franklin.send_update(VariableUpdateTarget::PidDerivative, 11);
        franklin.send_update(VariableUpdateTarget::GyroOffset, 148);
        franklin.send_update(VariableUpdateTarget::MotorEnabled, 1);
    }

    println!("\nBeginning Console:\n");

    loop {
        print!(">>> ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read line");

        let command_raw: &str = input.trim();

        let command: Vec<&str> = command_raw.split(' ').collect();

        match command[0] {
            "pid" => {

                match &mut franklin_container {
                    Some(franklin) => {
                        if command.len() != 3 {
                            println!("error: missing arguments");
                            continue;
                        }
        
                        let target = match command[1] {
                            "d" => VariableUpdateTarget::PidDerivative,
                            "p" => VariableUpdateTarget::PidProportional,
                            "i" => VariableUpdateTarget::PidIntegral,
                            _ => {
                                println!("error: invalid pid target '{}'", command[1]);
                                continue;
                            }
                        };
        
                        let value = match command[2].to_string().parse::<usize>() {
                            Ok(val) => val,
                            Err(err) => {
                                println!("error: illegal value {}", command[2]);
                                continue;
                            }
                        };
        
                        franklin.send_update(target, value);
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "mot" => {
                match &mut franklin_container {
                    Some(franklin) => {
                        if command.len() != 2 {
                            println!("error: missing arguments");
                            continue;
                        }
        
                        let value = match command[1] {
                            "on" => 1,
                            "off" => 0,
                            _ => {
                                println!("error: invalid argument. expected enable/disable");
                                continue;
                            }
                        };
        
                        franklin.send_update(VariableUpdateTarget::MotorEnabled, value);
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "gyro" => {
                match &mut franklin_container {
                    Some(franklin) => {
                        if command.len() != 2 {
                            println!("error: missing arguments");
                            continue;
                        }
        
                        let value_float = match command[1].to_string().parse::<f32>() {
                            Ok(val) => val,
                            Err(err) => {
                                println!("error: illegal value {}", command[2]);
                                continue;
                            }
                        };
        
                        let value = (value_float.round() as i32 * 10 + 128) as usize;
        
                        franklin.send_update(VariableUpdateTarget::GyroOffset, value)
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "graph" => {
                match &mut franklin_container {
                    Some(franklin) => {

                        if command.len() != 2 {
                            println!("error: missing arguments");
                            continue;
                        }
        
                        let (width, _height) = match dimensions() {
                            Some((w, h)) => (w, h),
                            None => panic!("unable to determine console width"),
                        };
        
                        let start = Instant::now();
        
                        let duration = match command[1].to_string().parse::<u64>() {
                            Ok(val) => val,
                            Err(err) => {
                                println!("error: illegal value {}", command[2]);
                                continue;
                            }
                        };
        
                        while (Instant::now() - start).as_secs() < duration {
                            let map = franklin.poll_status(false);
                            sleep(Duration::from_millis(10));
        
                            let gyro_value = map.get("Gyro Value").unwrap();
        
                            let mut scale = gyro_value / GYRO_GRAPH_LIM as f32;
                            if scale.abs() > 1. {
                                scale = scale / scale.abs();
                            }
        
                            let delta_cells = ((width as f32 / 2.) * scale).floor() as i16;
                            let total_cells = ((width as f32 / 2.).floor() as i16 + delta_cells) as usize;
        
                            let mut output: Vec<char> = Vec::with_capacity(width);
        
                            for _ in 0..total_cells {
                                output.push('█');
                            }
        
                            let leftover = width - output.len();
        
                            for _ in 0..leftover {
                                output.push(' ')
                            }
        
                            output[width / 2] = '|';
        
                            let final_str: String = output.into_iter().collect();
                            println!("{}", final_str);
                        }
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "python" => {
                if command.len() != 2 {
                    println!("error: missing arguments");
                    continue;
                }

                if command[1] == "begin" {
                } else {
                    println!("error: invalid argument {}", command[1])
                }
            }
            "ping" => {
                match &mut franklin_container{
                    Some(franklin) => {
                        franklin.send_ping();
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "poll" => {
                match &mut franklin_container{
                    Some(franklin) => {
                        franklin.poll_status(true);
                    },
                    None => {
                        println!("error: not connected to franklin")
                    }
                }
            }
            "exit" => std::process::exit(0),
            "" => {}
            _ => println!("error: unknown command"),
        }
    }

    // client.send_update(VariableUpdateTarget::PidProportional, 1);
    // client.send_update(VariableUpdateTarget::PidDerivative, 100);
    // client.send_update(VariableUpdateTarget::PidIntegral, 20000);
}
