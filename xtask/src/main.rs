//! Project tasks: `cargo xtask <embedded|pps|sim> ...`.

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use clap::{Args, Parser, Subcommand};

const FIRMWARE_PRESET: &str = "stm32l476";
const FIRMWARE_ELF: &str = "build/stm32l476/app/pps/pps_app";
const SIMULATOR: &str = "build/native/app/pps/sim/pps_sim";
const GDB_PORT: &str = "localhost:3333";

#[derive(Parser)]
#[command(about = "Build, flash and command the payload positioning system")]
struct Cli {
    #[command(subcommand)]
    task: Task,
}

#[derive(Subcommand)]
enum Task {
    /// Firmware tasks for the STM32L476 board
    Embedded {
        #[command(subcommand)]
        action: Embedded,
    },
    /// Send console commands to the board (runs ppsctl, see `pps --help`)
    #[command(disable_help_flag = true)]
    Pps(Passthrough),
    /// Same as `pps`, against the PC simulator; opens a shell by default
    #[command(disable_help_flag = true)]
    Sim(Passthrough),
}

#[derive(Subcommand)]
enum Embedded {
    /// Configure and compile the firmware
    Build {
        /// Optimised build instead of debug
        #[arg(short, long)]
        release: bool,
    },
    /// Program the last build onto the board and start it
    Flash,
    /// Restart the firmware on the board
    Reset,
    /// Open GDB on the running firmware without reprogramming it
    Attach,
    /// Build, then flash
    Run {
        /// Optimised build instead of debug
        #[arg(short, long)]
        release: bool,
    },
}

#[derive(Args)]
struct Passthrough {
    /// Arguments handed to ppsctl unchanged
    #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
    args: Vec<String>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let root = repo_root();

    let result: Result<ExitCode, String> = match cli.task {
        Task::Embedded { action } => match action {
            Embedded::Build { release } => build_firmware(&root, release),
            Embedded::Flash => flash(&root),
            Embedded::Reset => openocd(&root, &["init", "reset run", "shutdown"]),
            Embedded::Attach => attach(&root),
            Embedded::Run { release } => build_firmware(&root, release).and_then(|()| flash(&root)),
        }
        .map(|()| ExitCode::SUCCESS),
        Task::Pps(passthrough) => ppsctl(&root, &passthrough.args),
        Task::Sim(passthrough) => simulate(&root, passthrough.args),
    };

    match result {
        Ok(code) => code,
        Err(message) => {
            eprintln!("xtask: {message}");
            ExitCode::FAILURE
        }
    }
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask lives one level below the repository root")
        .to_path_buf()
}

fn build_firmware(root: &Path, release: bool) -> Result<(), String> {
    let mode = if release { "Release" } else { "Debug" };
    cmake_build(root, FIRMWARE_PRESET, mode)
}

fn cmake_build(root: &Path, preset: &str, mode: &str) -> Result<(), String> {
    run(Command::new("cmake").current_dir(root).args([
        &format!("--preset={preset}"),
        "-DTARGET_APP=",
        &format!("-DCMAKE_BUILD_TYPE={mode}"),
    ]))?;
    run(Command::new("cmake")
        .current_dir(root)
        .args(["--build", &format!("build/{preset}")]))
}

fn flash(root: &Path) -> Result<(), String> {
    require(root, FIRMWARE_ELF, "cargo xtask embedded build")?;
    let program = format!("program {{{FIRMWARE_ELF}}} verify reset exit");
    openocd(root, &[&program])
}

fn attach(root: &Path) -> Result<(), String> {
    require(root, FIRMWARE_ELF, "cargo xtask embedded build")?;

    let mut server = openocd_command(root, &[])
        .spawn()
        .map_err(|e| format!("cannot start openocd: {e}"))?;
    let session = run(Command::new("arm-none-eabi-gdb")
        .current_dir(root)
        .arg(FIRMWARE_ELF)
        .args(["-ex", &format!("target extended-remote {GDB_PORT}")]));

    let _ = server.kill();
    let _ = server.wait();
    session
}

fn openocd(root: &Path, commands: &[&str]) -> Result<(), String> {
    run(&mut openocd_command(root, commands))
}

fn openocd_command(root: &Path, commands: &[&str]) -> Command {
    let mut command = Command::new("openocd");
    command.current_dir(root).args([
        "-s",
        "mcu_support/stm32",
        "-f",
        "stlink-dap.cfg",
        "-f",
        "l4xx/stm32l4x.cfg",
    ]);
    for script in commands {
        command.args(["-c", script]);
    }
    command
}

/// Runs ppsctl and hands its exit code back, it reports its own errors.
fn ppsctl(root: &Path, args: &[String]) -> Result<ExitCode, String> {
    let status = Command::new(env!("CARGO"))
        .current_dir(root)
        .args(["run", "--quiet", "--release", "--package", "ppsctl", "--"])
        .args(args)
        .status()
        .map_err(|e| format!("cannot run ppsctl: {e}"))?;
    Ok(ExitCode::from(status.code().unwrap_or(1) as u8))
}

fn simulate(root: &Path, mut args: Vec<String>) -> Result<ExitCode, String> {
    let simulator = format!("{SIMULATOR}{}", std::env::consts::EXE_SUFFIX);
    if !root.join(&simulator).exists() {
        cmake_build(root, "native", "Debug")?;
    }
    if args.is_empty() {
        args.push("shell".to_string());
    }
    args.splice(0..0, ["--sim".to_string(), simulator]);
    ppsctl(root, &args)
}

fn require(root: &Path, file: &str, hint: &str) -> Result<(), String> {
    if root.join(file).exists() {
        Ok(())
    } else {
        Err(format!("{file} not found, run `{hint}` first"))
    }
}

fn run(command: &mut Command) -> Result<(), String> {
    let name = command.get_program().to_string_lossy().into_owned();
    let status = command
        .status()
        .map_err(|e| format!("cannot run {name}: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{name} failed ({status})"))
    }
}
