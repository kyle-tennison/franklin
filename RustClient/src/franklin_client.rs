/*
An interface to the ESP socket server

January 2024
*/

use super::{EspOperation, VariableUpdateTarget};
use std::{
    collections::{HashMap, VecDeque},
    io::{Read, Write},
    net::TcpStream,
    time::Instant,
};

const HEADER_BYTE: u8 = 0x46;
pub struct FranklinClient {
    socket: TcpStream,
}
impl FranklinClient {
    pub fn new(esp_address: &str) -> FranklinClient {
        let socket = TcpStream::connect(esp_address).unwrap();
        FranklinClient { socket }
    }

    /// Creates a header for a socket message
    ///
    /// # Arguments
    /// * `operation` - The corresponding enum identifying the operation
    /// * `content_length` - The length of the payload
    pub fn create_header(&mut self, operation: EspOperation, content_length: u16) -> [u8; 5] {
        let b1 = (content_length >> 8) as u8;
        let b2 = content_length as u8;

        [HEADER_BYTE, HEADER_BYTE, operation as u8, b1, b2]
    }

    /// Sends a generic text message
    ///
    /// # Arguments
    /// * `message` - The message to send
    pub fn send_message(&mut self, message: String) {
        let message_bytes = message.as_bytes();
        let header = self.create_header(
            EspOperation::Message,
            message_bytes.len().try_into().unwrap(),
        );

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + message_bytes.len());
        write_buf.extend_from_slice(&header);
        write_buf.extend_from_slice(&message_bytes);

        self.socket.write(&write_buf).unwrap();

        println!(
            "debug: sending message -- {:?} + {:?}",
            &header, &message_bytes
        );
    }

    /// Pings server and times response
    ///
    /// # Returns
    /// * The number of milliseconds it took the server to respond
    pub fn send_ping(&mut self) -> u128 {
        let start = Instant::now();
        let ping_content = "p i n g g g".as_bytes();
        let header = self.create_header(EspOperation::Ping, ping_content.len().try_into().unwrap());

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + ping_content.len());
        write_buf.extend_from_slice(&header);
        write_buf.extend_from_slice(&ping_content);

        self.socket.write(&write_buf).unwrap();

        let mut ping_response: Vec<u8> = vec![0; ping_content.len()];
        self.socket.read_exact(&mut ping_response).unwrap();

        if ping_response.len() != ping_content.len() {
            println!(
                "error: did not read entire message back. {:?} != {:?}",
                &ping_content, &ping_response
            );
        }

        for i in 0..ping_response.len() {
            if ping_content[i] != ping_response[i] {
                println!("error: echo is not equal");
                println!(" {:?} != {:?}", ping_content, ping_response);
                println!(" {:?} != {:?}", ping_content[i], ping_response[i]);
                panic!();
            }
        }

        println!("debug: ping response: {:?}", ping_response);

        let elapsed = (Instant::now() - start).as_millis();
        println!("info: pinged server in {elapsed} milliseconds");
        elapsed
    }

    /// Polls for telemetry
    ///
    /// # Arguments
    /// * `show` - Whether of not to print response to stdout
    ///
    /// # Returns
    /// * A map matching variables to their values
    pub fn poll_status(&mut self, show: bool) -> HashMap<String, f32> {
        let header = self.create_header(EspOperation::StatusRequest, 1);

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + 1);
        write_buf.extend_from_slice(&header);
        write_buf.push(0); // send a random byte so we don't have an empty packet

        self.socket.write(&write_buf).unwrap();

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
    ///
    /// # Arguments
    /// * `status_response` - The queue of bytes to deserialize
    ///
    /// # Returns
    /// * The deserialized HashMap of the status
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

            {
                match identifier.as_str() {
                    "Gyro Value" => {
                        value /= 100.;
                    }
                    "Gyro Offset" => value = value / 10.,
                    "Integral Sum" => value /= 10.,
                    "Motor Target" => value /= 100.,
                    _ => (),
                };
            }

            map.insert(identifier, value);
        }
        map
    }

    /// Uploads a variable update
    ///
    /// # Arguments
    /// * `target` - The variable to target
    /// * `value` - The value to update the variable to
    pub fn send_update(&mut self, target: VariableUpdateTarget, value: i16) {
        let b1 = (value >> 8) as u8;
        let b2 = value as u8;
        let payload: [u8; 3] = [target as u8, b1, b2];
        let header = self.create_header(EspOperation::Update, payload.len().try_into().unwrap());

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + payload.len());
        write_buf.extend_from_slice(&header);
        write_buf.extend_from_slice(&payload);

        self.socket.write(&write_buf).unwrap();
    }
}
