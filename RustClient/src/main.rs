/*
Franklin's Rust Client

This manages connection to the ESP32 on Franklin and communicates with the
Python GUI.

January 2024
*/

mod cli;
mod datatypes;
mod franklin_client;
mod python_client;

use datatypes::*;
use franklin_client::FranklinClient;
use python_client::PythonClient;

use std::{env, thread, time::Duration};

const ESP_ADDR: &str = "192.168.4.1:80";
const PYTHON_ADDR: &str = "localhost:2999";

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut esp_container: Option<FranklinClient> = None;

    // Allow skip login
    if args.len() > 1 && args[1].to_lowercase() == "skip" {
        println!("info: skipping franklin login")
    } else {
        println!("info: connecting to franklin...");

        let franklin = FranklinClient::new(ESP_ADDR);
        esp_container = Some(franklin);

        let franklin = match &mut esp_container {
            Some(franklin) => franklin,
            None => panic!("lost reference to franklin during setup"),
        };

        thread::sleep(Duration::from_millis(200));
        println!("info: connected to client");

        franklin.send_message("hello from rust".to_string());

        for _ in 0..2 {
            franklin.send_ping();
            thread::sleep(Duration::from_millis(500));
        }

        // Setup default values
        franklin.send_update(VariableUpdateTarget::PidProportional, 300);
        franklin.send_update(VariableUpdateTarget::PidIntegral, 48);
        franklin.send_update(VariableUpdateTarget::PidDerivative, 5);
        franklin.send_update(VariableUpdateTarget::GyroOffset, -20);
    }

    cli::start_console(esp_container, PYTHON_ADDR)
}
