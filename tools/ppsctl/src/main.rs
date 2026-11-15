//! ppsctl: command-line control for the payload positioning system.

mod link;

use std::io::{self, BufRead};
use std::process::ExitCode;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use link::Link;

const REPLY_TIMEOUT: Duration = Duration::from_millis(1500);

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// Serial port of the PPS console, for example COM5 or /dev/ttyUSB0
    #[arg(short, long, env = "PPS_PORT", global = true)]
    port: Option<String>,

    /// Serial baud rate
    #[arg(short, long, default_value_t = 115_200, global = true)]
    baud: u32,

    /// Talk to a pps_sim executable instead of a serial port
    #[arg(long, env = "PPS_SIM", global = true, conflicts_with = "port")]
    sim: Option<String>,

    #[command(subcommand)]
    action: Action,
}

#[derive(Subcommand)]
enum Action {
    /// Tell the payload that launch happened; it deploys after landing
    Launch,
    /// Deploy now, skipping launch and landing detection
    Deploy,
    /// Retract the payload to its stowed position
    Retract,
    /// Stop the motor and latch the safe state
    Abort,
    /// Leave the safe state and clear the latched fault
    Reset,
    /// Print the current state once
    Status,
    /// Print state changes and periodic status until interrupted
    Monitor {
        /// Milliseconds between status lines
        #[arg(short, long, default_value_t = 500)]
        interval: u32,
    },
    /// Interactive session: type console commands, see every reply
    Shell,
    /// List the serial ports on this machine
    Ports,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(cli) {
        Ok(code) => code,
        Err(message) => {
            eprintln!("ppsctl: {message}");
            ExitCode::from(2)
        }
    }
}

fn run(cli: Cli) -> Result<ExitCode, String> {
    if let Action::Ports = cli.action {
        return list_ports();
    }

    let mut link = connect(&cli)?;
    match cli.action {
        Action::Launch => request(&mut link, "launch"),
        Action::Deploy => request(&mut link, "deploy"),
        Action::Retract => request(&mut link, "retract"),
        Action::Abort => request(&mut link, "abort"),
        Action::Reset => request(&mut link, "reset"),
        Action::Status => request(&mut link, "status"),
        Action::Monitor { interval } => monitor(&mut link, interval),
        Action::Shell => shell(&mut link),
        Action::Ports => unreachable!("handled before connecting"),
    }
}

fn connect(cli: &Cli) -> Result<Link, String> {
    if let Some(path) = &cli.sim {
        return Link::simulator(path).map_err(|e| format!("cannot start simulator {path}: {e}"));
    }
    let port = cli
        .port
        .as_deref()
        .ok_or("no port given, use --port, PPS_PORT or --sim (see 'ppsctl ports')")?;
    Link::serial(port, cli.baud).map_err(|e| format!("cannot open {port}: {e}"))
}

fn list_ports() -> Result<ExitCode, String> {
    let ports = serialport::available_ports().map_err(|e| format!("cannot list ports: {e}"))?;
    if ports.is_empty() {
        println!("no serial ports found");
    }
    for port in ports {
        println!("{}", port.port_name);
    }
    Ok(ExitCode::SUCCESS)
}

/// Sends one command and waits for its reply, printing events seen meanwhile.
fn request(link: &mut Link, command: &str) -> Result<ExitCode, String> {
    link.send(command)
        .map_err(|e| format!("send failed: {e}"))?;

    let deadline = Instant::now() + REPLY_TIMEOUT;
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        let Some(line) = link.receive(remaining) else {
            break;
        };
        println!("{line}");
        let expected = if command == "status" { "status" } else { "ok" };
        if line.starts_with(expected) {
            return Ok(ExitCode::SUCCESS);
        }
        if line.starts_with("err") {
            return Ok(ExitCode::from(1));
        }
    }
    Err(format!("no reply to '{command}'"))
}

fn monitor(link: &mut Link, interval: u32) -> Result<ExitCode, String> {
    link.send(&format!("stream {interval}"))
        .map_err(|e| format!("send failed: {e}"))?;

    let silence = Duration::from_millis(u64::from(interval) * 4 + 2000);
    while let Some(line) = link.receive(silence) {
        println!("{line}");
    }
    Err("link went quiet".to_string())
}

fn shell(link: &mut Link) -> Result<ExitCode, String> {
    let (sender, typed) = mpsc::channel();
    thread::spawn(move || {
        for line in io::stdin().lock().lines().map_while(Result::ok) {
            if sender.send(line).is_err() {
                break;
            }
        }
    });

    eprintln!("connected, type 'help' for commands, 'exit' to leave");
    loop {
        match typed.recv_timeout(Duration::from_millis(20)) {
            Ok(line) if matches!(line.trim(), "exit" | "quit") => break,
            Ok(line) => link
                .send(line.trim())
                .map_err(|e| format!("send failed: {e}"))?,
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                drain(link, Duration::from_millis(300));
                break;
            }
        }
        drain(link, Duration::from_millis(5));
    }
    Ok(ExitCode::SUCCESS)
}

fn drain(link: &mut Link, wait: Duration) {
    while let Some(line) = link.receive(wait) {
        println!("{line}");
    }
}
