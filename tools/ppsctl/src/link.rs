//! Line-oriented link to the PPS console, over a serial port or a simulator process.

use std::io::{self, BufRead, BufReader, Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::Duration;

pub struct Link {
    writer: Box<dyn Write + Send>,
    lines: Receiver<String>,
    simulator: Option<Child>,
}

impl Link {
    /// Opens a serial port at 8N1.
    pub fn serial(port: &str, baud: u32) -> io::Result<Self> {
        let port = serialport::new(port, baud)
            .timeout(Duration::from_millis(100))
            .open()?;
        let reader = port.try_clone()?;
        Ok(Self {
            writer: Box::new(port),
            lines: spawn_reader(reader),
            simulator: None,
        })
    }

    /// Starts the simulator executable and talks to it over its stdin/stdout.
    pub fn simulator(path: &str) -> io::Result<Self> {
        let mut child = Command::new(path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let stdin = child.stdin.take().expect("simulator stdin is piped");
        let stdout = child.stdout.take().expect("simulator stdout is piped");
        Ok(Self {
            writer: Box::new(stdin),
            lines: spawn_reader(stdout),
            simulator: Some(child),
        })
    }

    pub fn send(&mut self, line: &str) -> io::Result<()> {
        self.writer.write_all(line.as_bytes())?;
        self.writer.write_all(b"\n")?;
        self.writer.flush()
    }

    /// Next complete line from the device, or None on timeout or a closed link.
    pub fn receive(&mut self, timeout: Duration) -> Option<String> {
        self.lines.recv_timeout(timeout).ok()
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        if let Some(child) = self.simulator.as_mut() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

fn spawn_reader<R: Read + Send + 'static>(source: R) -> Receiver<String> {
    let (sender, receiver) = mpsc::channel();
    thread::spawn(move || {
        let mut reader = BufReader::new(source);
        let mut buffer = Vec::new();
        loop {
            match reader.read_until(b'\n', &mut buffer) {
                Ok(0) => break,
                Ok(_) if buffer.ends_with(b"\n") => {
                    let line = String::from_utf8_lossy(&buffer).trim().to_string();
                    buffer.clear();
                    if !line.is_empty() && sender.send(line).is_err() {
                        break;
                    }
                }
                Ok(_) => {}
                Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
                Err(_) => break,
            }
        }
    });
    receiver
}
