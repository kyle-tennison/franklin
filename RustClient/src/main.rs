use std::{
    io::{Read, Write},
    net::TcpStream,
    thread,
    time::{Duration, Instant},
};

const ESP_ADDR: &str = "192.168.4.1:80";
const HEADER_BYTE: u8 = 0x46;

#[derive(Clone)]
enum Operation {
    Message = 0,
    Update = 1,
    Ping = 2,
}

#[derive(Clone, Debug)]
enum VariableUpdateTarget {
    PidProportional = 0,
    PidIntegral = 1,
    PidDerivative = 2,
    LinearVelocity = 3,
    AngularVelocity = 4,
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

fn main() {
    let mut sock = WebsocketClient::new(ESP_ADDR);
    // sock.send_message("hello from rust".to_string());
    thread::sleep(Duration::from_millis(4000));
    println!("info: connected to socket");

    sock.send_message("hello from rust".to_string());

    for _ in 0..2 {
        sock.send_ping();
        thread::sleep(Duration::from_millis(4000));
    }

    sock.send_update(VariableUpdateTarget::LinearVelocity, 0);
    thread::sleep(Duration::from_millis(4000));
    sock.send_update(VariableUpdateTarget::LinearVelocity, 10);
    thread::sleep(Duration::from_millis(4000));
    sock.send_update(VariableUpdateTarget::LinearVelocity, 40);

    let loop_size = 120;
    let loop_step = 3;
    loop {
        for s in 0..=loop_size {
            let speed = loop_step * s;
            sock.send_update(VariableUpdateTarget::LinearVelocity, speed);
            thread::sleep(Duration::from_millis(250));
        }
        for s in 0..=loop_size {
            let speed = (loop_size*loop_step) - loop_step * s;
            sock.send_update(VariableUpdateTarget::LinearVelocity, speed);
            thread::sleep(Duration::from_millis(250));
        }
        sock.send_ping();
        thread::sleep(Duration::from_millis(3000));
    }
}

// fn main() {
//     let mut socket = TcpStream::connect(ESP_ADDR).unwrap();

//     println!("info: socket connected");

//     // write a generic message
//     let mut message = Vec::from("hello from rust");
//     let mut msg_header = vec![
//         HEADER_BYTE,
//         HEADER_BYTE,
//         0,
//         message.len().try_into().unwrap(),
//         0,
//     ];

//     msg_header.append(&mut message);
//     socket.write(&msg_header).unwrap();

//     println!("info: sent message {:?}", msg_header);
// }

//     for _ in 0..2 {
//         thread::sleep(Duration::from_millis(4000));

//         // ping server
//         let ping_content = Vec::from("p i n g g g");
//         let mut ping_header = vec![
//             HEADER_BYTE,
//             HEADER_BYTE,
//             2,
//             ping_content.len().try_into().unwrap(),
//             0,
//         ];
//         ping_header.append(&mut ping_content.clone());

//         let start = Instant::now();
//         socket.write(&ping_header).unwrap();

//         println!("info: sent ping {:?}", ping_header);

//         let mut ping_response: Vec<u8> = vec![0; ping_content.len()];

//         socket.read_exact(&mut ping_response).unwrap();

//         println!(
//             "info: pinged server in {} milliseconds",
//             (Instant::now() - start).as_millis()
//         );
//         for i in 0..ping_response.len() {
//             if ping_content[i] != ping_response[i] {
//                 println!("error: echo is not equal");
//                 println!(" {:?} != {:?}", ping_content, ping_response);
//                 println!(" {:?} != {:?}", ping_content[i], ping_response[i]);
//                 panic!();
//             }
//         }
//     }
// }
