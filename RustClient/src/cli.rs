/*
A CLI interface for Franklin

January 2024
*/

use super::{FranklinClient, PythonClient, VariableUpdateTarget};
use std::{
    io::{self, Write},
    thread::sleep,
    time::{Duration, Instant},
};
use term_size::dimensions;

const GYRO_GRAPH_LIM: u8 = 45;

/// Handles PID update command
///
/// # Arguments
/// * `command` - A Vec of arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> pid i 310` Will update PID Integral to 310
fn handle_pid(command: Vec<&str>, esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            if command.len() != 3 {
                println!("error: missing arguments");
                return;
            }

            let target = match command[1] {
                "d" => VariableUpdateTarget::PidDerivative,
                "p" => VariableUpdateTarget::PidProportional,
                "i" => VariableUpdateTarget::PidIntegral,
                _ => {
                    println!("error: invalid pid target '{}'", command[1]);
                    return;
                }
            };

            let value = match command[2].to_string().parse::<i16>() {
                Ok(val) => val,
                Err(_err) => {
                    println!("error: illegal value {}", command[2]);
                    return;
                }
            };

            esp.send_update(target, value);
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Handles motor update command
///
/// # Arguments
/// * `command` - A Vec of arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> mot on` Will turn motors on
fn handle_mot(command: Vec<&str>, esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            if command.len() != 2 {
                println!("error: missing arguments");
                return;
            }

            let value = match command[1] {
                "on" => 1,
                "off" => 0,
                _ => {
                    println!("error: invalid argument. expected enable/disable");
                    return;
                }
            };

            esp.send_update(VariableUpdateTarget::MotorEnabled, value);
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Handles gyro update command
///
/// # Arguments
/// * `command` - A Vec of arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> gyro 1.3` Sets gyroscope offset to 1.3°
fn handle_gyro(command: Vec<&str>, esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            if command.len() != 2 {
                println!("error: missing arguments");
                return;
            }

            let value_float = match command[1].to_string().parse::<f32>() {
                Ok(val) => val,
                Err(_err) => {
                    println!("error: illegal value {}", command[2]);
                    return;
                }
            };

            let value = (value_float.round() * 10.) as i16;

            esp.send_update(VariableUpdateTarget::GyroOffset, value)
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Handles rudimentary graph output
///
/// # Arguments
/// * `command` - A Vec of arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> graph 20` Graphs gyroscope output for 20 seconds
fn handle_graph(command: Vec<&str>, esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            if command.len() != 2 {
                println!("error: missing arguments");
                return;
            }

            let (width, _height) = match dimensions() {
                Some((w, h)) => (w, h),
                None => panic!("unable to determine console width"),
            };

            let start = Instant::now();

            let duration = match command[1].to_string().parse::<u64>() {
                Ok(val) => val,
                Err(_err) => {
                    println!("error: illegal value {}", command[2]);
                    return;
                }
            };

            while (Instant::now() - start).as_secs() < duration {
                let map = esp.poll_status(false);
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
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Handles all python commands
///
/// # Arguments
/// * `command` - A Vec of arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
/// * `python_container` - A mutable reference to PythonClient wrapped in Option<>
/// * `python_addr` - The address of the python socket server
///
/// # Example
/// `>>> python stream 30` Streams status messages to python for 30 seconds
fn handle_python(
    command: Vec<&str>,
    esp_container: &mut Option<FranklinClient>,
    python_container: &mut Option<PythonClient>,
    python_addr: &str,
) {
    if command.len() < 2 {
        println!("error: missing arguments");
        return;
    }

    match command[1] {
        "begin" => {
            *python_container = Some(PythonClient::new(python_addr));
        }
        "msg" => {
            if command.len() != 3 {
                println!("error: missing arguments");
                return;
            }
            if let Some(python_client) = python_container {
                python_client.send_message(command[2]);
            } else {
                println!("error: python not connected")
            }
        }
        "stream" => {
            if command.len() != 3 {
                println!("error: missing arguments");
                return;
            }

            if let Some(python_client) = python_container {
                if let Some(esp) = esp_container {
                    let start = Instant::now();

                    let duration = match command[2].to_string().parse::<u64>() {
                        Ok(val) => val,
                        Err(err) => {
                            println!("error: illegal value {}: {err}", command[2]);
                            return;
                        }
                    };

                    while (Instant::now() - start).as_secs() < duration {
                        python_client.send_update_json(esp.poll_status(false));

                        sleep(Duration::from_millis(50));
                    }
                } else {
                    println!("error: esp not connected")
                }
            } else {
                println!("error: python not connected")
            }
        }
        "stop" => {
            if let Some(python_client) = python_container {
                python_client.stop();
            } else {
                println!("error: python not connected")
            }
        }
        _ => println!("error: invalid argument {}", command[1]),
    }
}

/// Times ping to esp
///
/// # Arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> ping` Will measure the time to echo a few bytes
fn handle_ping(esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            esp.send_ping();
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Polls the status of the ESP and displays in an ordered-json format
///
/// # Arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
///
/// # Example
/// `>>> poll` Will poll for the status and print the result
fn handle_poll(esp_container: &mut Option<FranklinClient>) {
    match esp_container {
        Some(esp) => {
            esp.poll_status(true);
        }
        None => {
            println!("error: not connected to esp")
        }
    }
}

/// Starts the CLI
///
/// # Arguments
/// * `esp_container` - A mutable reference to FranklinClient wrapped in Option<>
/// * `python_addr` - The address of the Python socket server
pub fn start_console(mut esp_container: Option<FranklinClient>, python_addr: &str) {
    println!("\nBeginning Console:\n");

    let mut python_container: Option<PythonClient> = None;

    loop {
        print!(">>> ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read line");

        let command_raw: &str = input.trim();
        let command: Vec<&str> = command_raw.split(' ').collect();

        // dispatch command
        match command[0] {
            "pid" => handle_pid(command, &mut esp_container),
            "mot" => handle_mot(command, &mut esp_container),
            "gyro" => handle_gyro(command, &mut esp_container),
            "graph" => handle_graph(command, &mut esp_container),
            "python" => handle_python(
                command,
                &mut esp_container,
                &mut python_container,
                python_addr,
            ),
            "ping" => handle_ping(&mut esp_container),
            "poll" => handle_poll(&mut esp_container),
            "exit" => std::process::exit(0),
            "" => (),
            _ => println!("error: unknown command"),
        }
    }
}
