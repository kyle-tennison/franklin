#![allow(dead_code)]
#![allow(unused_variables)]


use std::{
    collections::{HashMap, VecDeque}, io::{self, Read, Write}, net::TcpStream, thread, time::{Duration, Instant}
};

const ESP_ADDR: &str = "192.168.4.1:80";
const HEADER_BYTE: u8 = 0x46;

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

struct WebsocketClient {
    socket: TcpStream,
}
impl WebsocketClient {
    fn new(esp_address: &str) -> WebsocketClient {
        let socket = TcpStream::connect(esp_address).unwrap();
        WebsocketClient { socket }
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
    fn poll_status(&mut self) {
        let mut header = self.get_header(Operation::StatusRequest, 1);

        header.push(1); // add a random byte so we don't have an empty packet

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => panic!("error: failed to send status request: {err}")
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
                    print!("error: read {n} bytes while intending to read one");
                    return
                }
                Err(err) => {
                    print!("error: failed to read from status response: {err}");
                    return
                }
            }

            let recv = recv_buf[0];

            if byte_index <= 1 {
                if recv != HEADER_BYTE {
                    println!("error: byte {recv} is not a header byte");
                    continue;
                }
            }

            else if byte_index == 2  {
                // ignore the operation byte
            }

            else if byte_index == 3 {
                content_len = recv.into()

            }
            else if byte_index > 3 {

                if !content_len_determined {
                    content_len += usize::from(recv);

                    if recv == 0 {
                        content_len_determined = true;
                    }
                }
                else {
                    status_response.push_back(recv);

                    if status_response.len() == content_len {
                        println!("debug: done collecting status");
                        break
                    }
                }

            }

            byte_index += 1;
        }

        let map = self.deserialize_status(&mut status_response);

        println!("Status response: {{");
        for k in map.keys(){
            let v = map.get(k).unwrap();
            
            println!("\t{k} : {v},");
        }
        println!("}}");


    }

    /// Deserializes the status response from ESP into a hash map
    fn deserialize_status(&self, status_response: &mut VecDeque<u8>) -> HashMap<String, usize>{

        let mut map: HashMap<String, usize> = HashMap::new();

        assert!(*status_response.back().unwrap() == 0, "status response must end in zero");

        while !status_response.is_empty() {
            let key = status_response.pop_front().unwrap();
            let mut value: usize = status_response.pop_front().unwrap().into();
            let mut next = status_response.pop_front().unwrap();

            while next != 0{
                value += usize::from(next);
                next = status_response.pop_front().unwrap();
            }

            let identifier = match key {
                0 => "PID Proportional",
                1 => "PID Integral",
                2 => "PID Derivative",
                3 => "Motors enabled",
                _ => panic!("failed to deserialize status response: unknown key")
            }.to_string();

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
        println!("debug: sending update -- {:?}", &header);

        println!("info: updating {:?} to value {}", &target, value);

        match self.socket.write(&header) {
            Ok(_) => (),
            Err(err) => println!("error: failed to send update -- {err}"),
        };
    }
}

fn main() {
    let mut sock = WebsocketClient::new(ESP_ADDR);


    thread::sleep(Duration::from_millis(1000));
    println!("info: connected to socket");

    sock.send_message("hello from rust".to_string());

    for _ in 0..2 {
        sock.send_ping();
        thread::sleep(Duration::from_millis(1000));
    }

    sock.poll_status();

    // Setup default values
    sock.send_update(VariableUpdateTarget::PidProportional, 29);
    sock.send_update(VariableUpdateTarget::PidIntegral, 81);
    sock.send_update(VariableUpdateTarget::PidDerivative, 11);
    sock.send_update(VariableUpdateTarget::GyroOffset, 148);

    loop {
        print!(">>> ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input)
            .expect("Failed to read line");

        let command_raw: &str = input.trim();

        let command: Vec<&str> = command_raw.split(' ').collect();



        match command[0] {
            "pid" => {

                if command.len() != 3{
                    println!("error: missing arguments");
                    continue;
                }

                let target = match command[1] {
                    "d" => VariableUpdateTarget::PidDerivative,
                    "p" => VariableUpdateTarget::PidProportional,
                    "i" => VariableUpdateTarget::PidIntegral,
                    _ => {
                        println!("error: invalid pid target '{}'", command[1]);
                        continue
                    }
                };


                let value = match command[2].to_string().parse::<usize>() {
                    Ok(val) => val,
                    Err(err) => {
                        println!("error: illegal value {}", command[2]);
                        continue
                    }
                };

                sock.send_update(target, value);
            },
            "mot" => {
                if command.len() != 2{
                    println!("error: missing arguments");
                    continue;
                }

                let value = match command[1] {
                    "on" => 1,
                    "off" => 0,
                    _ => {
                        println!("error: invalid argument. expected enable/disable");
                        continue
                    }
                };

                sock.send_update(VariableUpdateTarget::MotorEnabled, value);
            },
            "gyro" => {
                if command.len() != 2{
                    println!("error: missing arguments");
                    continue;
                }

                let value_float = match command[1].to_string().parse::<f32>() {
                    Ok(val) => val,
                    Err(err) => {
                        println!("error: illegal value {}", command[2]);
                        continue
                    }
                };

                let value = (value_float.round() as i32 * 10 + 128) as usize;

                sock.send_update(VariableUpdateTarget::GyroOffset, value)


            }
            "ping" => {sock.send_ping();},
            "poll" => {sock.poll_status();},
            "exit" => {std::process::exit(0)},
            "" => {},
            _ => println!("error: unknown command")
        }
    }


    // sock.send_update(VariableUpdateTarget::PidProportional, 1);
    // sock.send_update(VariableUpdateTarget::PidDerivative, 100);
    // sock.send_update(VariableUpdateTarget::PidIntegral, 20000);


}