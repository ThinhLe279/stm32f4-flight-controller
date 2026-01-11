# STM32F4 Flight Controller – Project Roadmap

A phased development plan for building a custom flight controller from scratch, covering hardware design, bare-metal firmware, and flight control software.

---

## Phase 0: System Definition
**Status:** `ACTIVE`

Define the complete system architecture and component selection before hardware design begins.

### Hardware Architecture
- [ ] Define system block diagram (MCU, sensors, motors, power, debug)
- [ ] Select IMU (gyroscope + accelerometer)
- [ ] Select barometer for altitude estimation
- [ ] Select magnetometer for heading reference
- [ ] Define motor/ESC interface protocol (PWM, DShot, etc.)

### Power System
- [ ] Define battery type and voltage range
- [ ] Define required voltage rails (VBAT, 5V, 3V3, etc.)
- [ ] Plan current sensing and protection

### Development Interface
- [ ] Define debug and programming interface (SWD, UART, USB)
- [ ] Define clock source strategy (internal oscillator vs. external crystal)
- [ ] Create initial MCU pinout table

> **Tracking:** All tasks tracked as GitHub Issues  
> **Exit Criteria:** Complete system specification document and validated pinout

---

## Phase 1: Basic Electronics
**Status:** `PLANNED`

Design and validate the core power and debug circuitry.

### Power Distribution
- [ ] Battery input protection (reverse polarity, overcurrent)
- [ ] 5V voltage regulator design and validation
- [ ] 3.3V voltage regulator design and validation
- [ ] Decoupling capacitor placement
- [ ] Ground plane design and routing

### Development Infrastructure
- [ ] SWD connector footprint and pinout
- [ ] Boot pin configuration (BOOT0, BOOT1)
- [ ] Reset circuit design
- [ ] Clock source implementation (crystal + load caps)

### Validation
- [ ] Power rail measurement under load
- [ ] Power supply noise characterization
- [ ] Voltage ripple testing

> **Exit Criteria:** Stable power delivery with clean rails verified by oscilloscope

---

## Phase 2: Bare-Metal Bring-Up
**Status:** `PLANNED`

Establish reliable code execution on the target hardware.

### Boot & Execution
- [ ] Startup assembly code
- [ ] Linker script configuration (Flash/RAM layout)
- [ ] Clock tree initialization (PLL, AHB, APB clocks)
- [ ] System tick timer configuration

### Basic I/O
- [ ] GPIO initialization and LED blink test
- [ ] UART initialization and "Hello World" output
- [ ] SWD debug session establishment

### Reliability
- [ ] HardFault handler with diagnostic output
- [ ] Flash memory read/write validation
- [ ] RAM test pattern verification
- [ ] SWD stability under continuous debugging

> **Exit Criteria:** Reliable code execution with UART output and debugger attachment

---

## Phase 3: Firmware Skeleton
**Status:** `PLANNED`

Establish the development environment and RTOS foundation.

### Build System
- [ ] Repository structure (src, inc, drivers, lib, docs)
- [ ] CMake build configuration
- [ ] CMSIS integration (device headers, startup files)
- [ ] ST LL (Low-Layer) driver integration

### Real-Time Operating System
- [ ] FreeRTOS kernel integration
- [ ] Task creation and scheduling verification
- [ ] Idle task and tick hook configuration
- [ ] Heap management strategy

### Infrastructure
- [ ] Logging system (UART, ITM, or RTT)
- [ ] Error handling and assertion framework
- [ ] Stack overflow hooks
- [ ] Malloc failure hooks

> **Exit Criteria:** Multi-task FreeRTOS application running with logging

---

## Phase 4: Hardware Drivers
**Status:** `PLANNED`

Implement and validate all peripheral drivers for sensor and motor control.

### Communication Interfaces
- [ ] SPI driver (blocking and DMA modes)
- [ ] I2C driver (for magnetometer/barometer if needed)
- [ ] UART driver with DMA for telemetry

### Sensor Integration
- [ ] IMU driver (SPI read/write, register configuration)
- [ ] Barometer driver
- [ ] Magnetometer driver
- [ ] Sensor calibration routines (gyro bias, accel offset, mag hard/soft iron)

### Motor Control
- [ ] Timer configuration for PWM generation
- [ ] Multi-channel PWM output (4+ motors)
- [ ] DShot protocol implementation (if selected)

### Performance Optimization
- [ ] DMA configuration for sensor reads
- [ ] Interrupt priority assignment (sensor > control > telemetry)
- [ ] Timing analysis and profiling

> **Exit Criteria:** All sensors reading valid data, motors responding to PWM commands

---

## Phase 5: Flight Software
**Status:** `PLANNED`

Implement the flight control algorithms and operational logic.

### Software Architecture
- [ ] Task model definition (sensor, control, telemetry, supervisor)
- [ ] Inter-task communication (queues, semaphores, mutexes)
- [ ] Control loop timing validation (1kHz+ target)

### Sensor Processing
- [ ] Sensor data fusion (complementary filter or Kalman filter)
- [ ] Attitude estimation (roll, pitch, yaw)
- [ ] Altitude estimation from barometer + accelerometer
- [ ] Heading estimation from magnetometer

### Flight Control
- [ ] Flight mode state machine (disarmed, armed, stabilize, altitude hold, etc.)
- [ ] PID controllers (rate, angle, altitude)
- [ ] Motor mixing algorithms
- [ ] Control loop profiling and optimization

### Safety & Reliability
- [ ] Failsafe logic (signal loss, low battery, sensor failure)
- [ ] Arming/disarming logic
- [ ] Emergency stop and motor cutoff

### Monitoring & Tuning
- [ ] Telemetry output (MAVLink or custom protocol)
- [ ] Real-time parameter adjustment
- [ ] PID tuning procedure and logging
- [ ] Flight data recording for analysis

> **Exit Criteria:** Stable flight in basic stabilize mode with telemetry feedback

---

## Development Principles

**Incremental Validation**  
Each phase builds on validated work from the previous phase. No phase begins until its predecessor is complete and stable.

**Test-Driven Hardware**  
Every circuit addition is validated with measurements before moving forward.

**Continuous Integration**  
Automated builds and unit tests run on every commit where applicable.

**Documentation First**  
Design decisions, schematics, and pinouts are documented before implementation.

---

## Tools & Resources

**Hardware:** STM32CubeMX, KiCad, Oscilloscope, Logic Analyzer  
**Software:** ARM GCC, CMake, OpenOCD, GDB, FreeRTOS  
**Version Control:** Git + GitHub Issues for task tracking  
**References:** ST Reference Manual, FreeRTOS docs, Betaflight source code

---

**Last Updated:** January 2026  
**Current Phase:** Phase 0 (System Definition)