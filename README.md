# Payload Positioning System for Long Beach Rocketry Launch

## Purpose

This system is designed to provide accurate payload positioning d control for Long Beach Rocketry launches. It integrates sensor data, motor control, and real-time logic to ensure the payload is correctly positioned and stabilized throughout the flight. The system is modular, supporting various hardware interfaces and custom applications for rocketry needs.

---

This is the payload positioning system for Long Beach Rocketry launches.

Created by: **@pasinponsww**

**Generic bare metal supports (I2C, PWM):** @yshiblanco  
**GPIO and system clock:** @TJMalaska  
**IMU, PPS app, delay, etc.:** @pasinponsww

---

## How it works

The firmware is a bare-metal loop with no RTOS. Every 10 ms it samples the
sensors, runs one step of the payload state machine and services the console.
Every transition is gated by sensor feedback.

```
Standby --launch accel--> Flight --at rest (or timeout)--> Deploying
Deploying --extended switch closes at the expected travel--> Holding
Holding --retract command--> Retracting --retracted switch closes--> Standby

Deploying / Retracting --fault--> Recovery --> retry | retract (fallback) | Safed
```

- **Travel** runs between two limit switches, one at each end. Deploying is a
  closed PID loop on the encoder over the expected travel of 720 degrees
  (48 CPR encoder, 9.7:1 gearbox, about 931 counts), then a slow creep until
  the extended switch closes. The switch has to close within a window of the
  expected count, otherwise the encoder and the switch disagree.
- **Retracting** is the same in reverse onto the retracted switch, which also
  re-zeroes the encoder. A deploy that starts away from the retracted switch
  retracts first to find it.
- **FDIR** watches every motion for a driver fault, encoder fault, stall,
  wrong-direction travel, timeout and limit switch mismatch. Recoverable faults
  are retried with a growing backoff (a stall also gets a short reverse nudge).
  A deploy that still cannot reach the extended switch falls back to
  retracting, then latches `Safed` with the fault recorded. If retracting fails
  too, or the encoder cannot be trusted, the driver is put to sleep in `Safed`.
  `reset` leaves `Safed`.

| Path | Contents |
| --- | --- |
| `app/pps/pps.*` | payload state machine |
| `app/pps/pps_config.h` | every mission constant and tuning value |
| `app/pps/fdir/` | fault detection and recovery policy |
| `app/pps/control/` | PID position loop |
| `app/pps/motor_support/` | DC motor actuator with encoder feedback |
| `app/pps/console/` | serial command console |
| `app/pps/pps_bsp/` | pin map and hardware bring-up for the L476 board |
| `app/pps/sim/` | PC simulator of the mechanism |
| `tools/ppsctl/` | command-line tool (Rust) |
| `xtask/` | `cargo xtask` build, flash and control tasks |

## Commanding the payload

The console is on the debug USART (PB6 TX, PB7 RX) at 115200 8N1. It is plain
text, one command per line, so any serial terminal works:

```
launch | deploy | retract | abort | reset | status | stream <ms> | help
```

Everything is also wrapped as cargo tasks, run from anywhere in the repo:

```
cargo xtask embedded build      # compile the firmware (--release for optimised)
cargo xtask embedded flash      # program the board over ST-Link and start it
cargo xtask embedded run        # build, then flash
cargo xtask embedded reset      # restart the firmware
cargo xtask embedded attach     # GDB on the running firmware

cargo xtask pps ports           # list serial ports
cargo xtask pps --port COM5 status
cargo xtask pps --port COM5 launch | deploy | retract | abort | reset
cargo xtask pps --port COM5 monitor
cargo xtask pps --port COM5 shell
```

Set `PPS_PORT` once (`$env:PPS_PORT = "COM5"`) to drop `--port`. `pps` runs
`ppsctl` (in `tools/ppsctl`); `embedded` needs CMake, the ARM toolchain and
OpenOCD on the PATH.

Without hardware, `sim` runs the same commands against a simulated mechanism
and builds it on first use:

```
cargo xtask sim                 # interactive shell
cargo xtask sim status
```

Inside the simulator shell, faults can be injected: `sim jam on`,
`sim drvfault on`, `sim reverse on`, `sim deadswitch on`, `sim moving on`,
`sim boost 400`.

---

## Build & Compile Instructions

### Prerequisites

- **ARM toolchain**:  
	- Ubuntu: `sudo apt-get install arm-none-eabi-gcc`  
	- Windows: [Download here](https://developer.arm.com/downloads/-/gnu-rm) and add to PATH.  
	- Check: `arm-none-eabi-gcc --version`

- **CMake**:  
	- Ubuntu: `sudo apt-get install cmake`  
	- Windows: [Download here](https://cmake.org/download/) and add to PATH.  
	- Check: `cmake --version`

- **Ninja**:  
	- Ubuntu: `sudo apt-get install ninja-build`  
	- Windows: [Download here](https://github.com/ninja-build/ninja/releases) and add to PATH.  
	- Check: `ninja --version`

- **Dependencies**:  
	- Run: `git submodule update --init --recursive`

### Building

- **Windows**: Use the `.ps1` script in PowerShell.  
- **Linux/Unix**: Use the `.sh` script.  
- Example: `./make.ps1 -t <name of preset>` (see `CMakePresets.json`)
- To build a specific app: `./make.ps1 -t <preset> -a <app>`
- Clean build: `./make.ps1 -t <preset> -c`
- Release mode: `./make.ps1 -t <preset> -r`

### Debugging

- Install OpenOCD:  
	- Ubuntu: `sudo apt-get install openocd`  
	- Windows: [Download here](https://openocd.org/pages/getting-openocd.html) and add to PATH.
- Install the Cortex-Debug extension for VSCode.
- Reference launch configurations are in `.vscode/launch.json`.

---
