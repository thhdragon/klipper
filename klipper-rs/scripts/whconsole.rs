//! Async Rust port of whconsole.py: interactive console for Klipper webhooks interface
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Ported to Rust by the Klipper async porting team

use std::env;
use std::path::Path;
use std::process::exit;
use std::io::{self, Write};
use tokio::net::UnixStream;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use serde_json::Value;

#[tokio::main]
async fn main() {
    // Parse args: expect one argument (socket path)
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <socket filename>", args[0]);
        exit(1);
    }
    let uds_path = &args[1];

    // Connect to Unix socket, retry on ECONNREFUSED
    let stream = loop {
        match UnixStream::connect(Path::new(uds_path)).await {
            Ok(s) => break s,
            Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => {
                // Retry on connection refused
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                continue;
            }
            Err(e) => {
                eprintln!("Unable to connect socket {} [{}]", uds_path, e);
                exit(1);
            }
        }
    };
    eprintln!("Connection.");

    // Split socket for concurrent read/write
    let (mut sock_reader, mut sock_writer) = stream.into_split();
    let mut sock_buf = Vec::new();

    // Setup async stdin
    let stdin = tokio::io::stdin();
    let mut stdin_reader = BufReader::new(stdin).lines();

    // Buffer for partial socket reads (for messages split across reads)
    let mut socket_partial = Vec::new();

    loop {
        tokio::select! {
            // Read line from stdin
            line = stdin_reader.next_line() => {
                match line {
                    Ok(Some(line)) => {
                        let line = line.trim();
                        if line.is_empty() || line.starts_with('#') {
                            continue;
                        }
                        match serde_json::from_str::<Value>(line) {
                            Ok(json_val) => {
                                let cm = serde_json::to_string(&json_val).unwrap();
                                println!("SEND: {}", cm);
                                if let Err(e) = sock_writer.write_all(cm.as_bytes()).await {
                                    eprintln!("Socket write error: {}", e);
                                    break;
                                }
                                if let Err(e) = sock_writer.write_all(b"\x03").await {
                                    eprintln!("Socket write error: {}", e);
                                    break;
                                }
                            }
                            Err(_) => {
                                eprintln!("ERROR: Unable to parse line");
                            }
                        }
                    }
                    Ok(None) => {
                        // EOF on stdin
                        eprintln!("Stdin closed. Exiting.");
                        break;
                    }
                    Err(e) => {
                        eprintln!("Stdin read error: {}", e);
                        break;
                    }
                }
            }
            // Read from socket
            res = sock_reader.read_buf(&mut sock_buf) => {
                match res {
                    Ok(0) => {
                        eprintln!("Socket closed");
                        break;
                    }
                    Ok(_n) => {
                        // Process all complete \x03-delimited messages
                        let mut start = 0;
                        for (i, &b) in sock_buf.iter().enumerate() {
                            if b == 0x03 {
                                let mut msg = socket_partial.split_off(0);
                                msg.extend_from_slice(&sock_buf[start..i]);
                                start = i + 1;
                                println!("GOT: {}", String::from_utf8_lossy(&msg));
                            }
                        }
                        // Save any partial message for next read
                        socket_partial.extend_from_slice(&sock_buf[start..]);
                        sock_buf.clear();
                    }
                    Err(e) => {
                        eprintln!("Socket read error: {}", e);
                        break;
                    }
                }
            }
        }
    }
    eprintln!("Exiting.");
}
