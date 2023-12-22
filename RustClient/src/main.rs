use std::{
    io::{Read, Write},
    net::TcpStream,
    thread,
    time::{Duration, Instant},
};

const ESP_ADDR: &str = "192.168.4.1:80";
const HEADER_BYTE: u8 = 0x46;

fn main() {
    let mut socket = TcpStream::connect(ESP_ADDR).unwrap();

    println!("info: socket connected");

    loop {
        thread::sleep(Duration::from_millis(4000));

        // write a generic message
        // let mut message = Vec::from("hello from rust");
        // let mut msg_header = vec![
        //     HEADER_BYTE,
        //     HEADER_BYTE,
        //     0,
        //     message.len().try_into().unwrap(),
        //     0,
        // ];

        // msg_header.append(&mut message);
        // socket.write(&msg_header).unwrap();

        // println!("info: sent message {:?}", msg_header);

        // ping server
        let ping_content = Vec::from("p i n g g g");
        let mut ping_header = vec![
            HEADER_BYTE,
            HEADER_BYTE,
            2,
            ping_content.len().try_into().unwrap(),
            0,
        ];
        ping_header.append(&mut ping_content.clone());

        let start = Instant::now();
        socket.write(&ping_header).unwrap();

        println!("info: sent ping {:?}", ping_header);

        let mut ping_response: Vec<u8> = vec![0; ping_content.len()];

        socket.read_exact(&mut ping_response).unwrap();

        println!(
            "info: pinged server in {} milliseconds",
            (Instant::now() - start).as_millis()
        );
        for i in 0..ping_response.len() {
            if ping_content[i] != ping_response[i] {
                println!("error: echo is not equal");
                println!(" {:?} != {:?}", ping_content, ping_response);
                println!(" {:?} != {:?}", ping_content[i], ping_response[i]);
                panic!();
            }
        }
    }
}
