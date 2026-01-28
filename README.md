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
