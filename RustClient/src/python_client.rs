/*
An interface to the local Python socket server (for GUI data)

January 2024
*/

use std::{
    collections::{hash_map::OccupiedEntry, HashMap},
    io::Write,
    net::TcpStream,
};

use crate::EspOperation;

/// A client for communicating with the Python socket server
pub struct PythonClient {
    socket: TcpStream,
}
impl PythonClient {
    pub fn new(python_address: &str) -> PythonClient {
        let socket = TcpStream::connect(python_address).unwrap();

        PythonClient { socket }
    }

    /// Creates a header for the socket message
    ///
    /// # Arguments
    /// * `operation_code` - The code that identifies the operation type
    /// * `content_length` - The length of the payload, in bytes
    ///
    /// # Returns
    /// * An u8 array of the header
    fn create_header(&mut self, operation_code: u8, content_length: u16) -> [u8; 5] {
        let c1: u8 = (content_length >> 8) as u8;
        let c2 = content_length as u8;

        let header: [u8; 5] = [70, 70, operation_code, c1, c2];

        header
    }

    /// Sends a generic text message
    ///
    /// # Arguments
    /// * `message` - The message to send
    pub fn send_message(&mut self, message: &str) {
        println!("debug: sending message {:?}", &message);

        let header = self.create_header(
            EspOperation::Message as u8,
            message.len().try_into().unwrap(),
        );

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + 1);
        write_buf.extend_from_slice(&header);
        write_buf.extend_from_slice(&message.as_bytes());

        self.socket.write(&write_buf).unwrap();
    }

    /// Sends a termination message
    pub fn stop(&mut self) {
        let header = self.create_header(1, 1);

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + 1);
        write_buf.extend_from_slice(&header);
        write_buf.push(0);

        self.socket.write(&write_buf).unwrap();
    }

    /// Sends a status update as a json
    ///
    /// # Arguments
    /// * `map` - A HashMap to serialize into a json
    pub fn send_update_json(&mut self, map: HashMap<String, f32>) {
        let mut outbound: Vec<char> = Vec::new();

        outbound.push('{');
        for (i, key) in map.keys().enumerate() {
            let value = map.get(key).unwrap().to_string();

            outbound.push('"');
            outbound.extend(key.chars());
            outbound.push('"');
            outbound.push(':');
            outbound.extend(value.chars());

            if i != map.len() - 1 {
                outbound.push(',');
            }
        }
        outbound.push('}');

        let content_length: u16 = outbound.len() as u16;
        let outbound_string: String = outbound.into_iter().collect();

        println!("sending update: {}", outbound_string);

        let header = self.create_header(EspOperation::StatusRequest as u8, content_length);

        // concat message
        let mut write_buf: Vec<u8> = Vec::with_capacity(header.len() + 1);
        write_buf.extend_from_slice(&header);
        write_buf.extend_from_slice(outbound_string.as_bytes());

        self.socket.write(&write_buf).unwrap();
    }
}
