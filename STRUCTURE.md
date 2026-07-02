# ArduMotorBlimp Repository Structure and Architecture

**Version:** 1.1  
**Date:** July 1, 2026  
**Repository:** https://github.com/DaniK-51/ArduMotorBlimp  
**Author:** DaniK-51 (Daniyar)  
**License:** GPL-3.0

---

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
3. [Repository Structure](#repository-structure)
4. [Main Components](#main-components)
5. [Build System](#build-system)
6. [Flight Modes](#flight-modes)
7. [Control System](#control-system)
8. [Telemetry and Communication](#telemetry-and-communication)
9. [Safety](#safety)
10. [Parameters](#parameters)
11. [Integration with ArduPilot](#integration-with-ardupilot)
12. [Connection with the_blimp_swp Project](#connection-with-the_blimp_swp-project)

---

## Introduction

### Document Purpose

This document describes the architecture, structure, and components of the **ArduMotorBlimp** repository — a custom implementation of a "Blimp" (Lighter-Than-Air) vehicle type for the ArduPilot platform.

### Target Audience

- Control system developers
- ArduPilot integration engineers
- Project team members
- Code reviewers

---

## Project Overview

### What is ArduMotorBlimp?

**ArduMotorBlimp** is a specialized blimp (Lighter-Than-Air vehicle) implementation based on the official ArduPilot platform. The repository contains the initial state of the official `Blimp` module with custom modifications.

### Key Characteristics

| Parameter | Value |
|-----------|-------|
| **Vehicle Type** | Motorized blimp |
| **Languages** | C++ (96.5%), C (3.2%), Python (0.3%) |
| **License** | GPL-3.0 |
| **Base** | Official ArduPilot repository |
| **Status** | Initial development (Initial commit) |

### Main Features

- Support for standard ArduPilot flight modes
- MAVLink protocol integration
- Safety and failsafe system
- Support for various motor configurations
- Telemetry logging
- EKF (Extended Kalman Filter) validation

---

## Repository Structure

### Root Directory

```
ArduMotorBlimp/
│
├── Main Application Files
│   ├── Blimp.cpp                  # Entry point (main loop)
│   ├── Blimp.h                    # Main header file
│   ├── config.h                   # System configuration
│   ├── defines.h                  # Constant definitions
│   └── version.h                  # Firmware version
│
├── Parameters and Configuration
│   ├── Parameters.cpp             # Parameters implementation
│   ├── Parameters.h               # Parameters declaration
│   └── wscript                    # Build script (Waf)
│
├── Flight Modes
│   ├── mode.cpp                   # Base mode class
│   ├── mode.h                     # Modes header
│   ├── mode_manual.cpp            # Manual control
│   ├── mode_loiter.cpp            # Position hold
│   ├── mode_hold.cpp              # Hold mode
│   ├── mode_auto.cpp              # Automatic mode
│   ├── mode_land.cpp              # Landing
│   ├── mode_rtl.cpp               # Return-To-Launch
│   ├── mode_velocity.cpp          # Velocity control
│   ├── Loiter.cpp                 # Loiter logic
│   └── Loiter.h
│
├── Control System
│   ├── motors.cpp                 # Motor output and arming checks
│   ├── Fins.cpp                   # Actuator mixing (frame-specific motor/servo output)
│   ├── Fins.h                     # Fins header with output fields and mixing matrix
│   ├── commands.cpp               # Command processing
│   ├── radio.cpp                  # RC radio control
│   └── inertia.cpp                # Inertial navigation
│
├── Sensors and Navigation
│   ├── sensors.cpp                # Sensor reading
│   └── ekf_check.cpp              # EKF validation
│
├── Telemetry and Communication
│   ├── GCS_Blimp.cpp              # Ground Control Station
│   ├── GCS_Blimp.h
│   ├── GCS_MAVLink_Blimp.cpp      # MAVLink processing
│   ├── GCS_MAVLink_Blimp.h
│   ── Log.cpp                    # Data logging
│
├── Safety
│   ├── AP_Arming_Blimp.cpp        # Arming checks
│   ├── AP_Arming_Blimp.h
│   ├── failsafe.cpp               # Failsafe logic
│   ├── events.cpp                 # Event handling
│   └── system.cpp                 # System functions
│
├── System State
│   ├── AP_State.cpp               # Vehicle state
│   └── AP_State.h
│
└── RC Channels
    ├── RC_Channel_Blimp.cpp       # RC channels processing
    └── RC_Channel_Blimp.h
```

### File Statistics

| Category | Number of Files |
|----------|-----------------|
| **Main (.cpp)** | 18 files |
| **Headers (.h)** | 13 files |
| **Configuration** | 3 files |
| **Total** | 42 files |

---

## Main Components

### 1. Blimp.cpp / Blimp.h

**Purpose:** Entry point and main application loop

**Main Functions:**
```cpp
// Main blimp class
class Blimp : public AP_HAL::HAL::Callbacks {
public:
    // System initialization
    void init() override;
    
    // Main loop (called 1000 times per second)
    void loop() override;
    
    // Get singleton instance
    static Blimp& get_instance();
    
private:
    // Internal methods
    void update_loop();
    void read_sensors();
    void update_control();
};
```

**Lifecycle:**
```
init() -> Setup all subsystems
   |
loop() -> Main loop
   |
1. read_sensors()
2. update_EKF()
3. update_control()
4. update_motors()
5. send_telemetry()
6. logger.write()
   |
[Repeat 1000 Hz]
```

### 2. Parameters.cpp / Parameters.h

**Purpose:** Parameter system for behavior configuration

**Example Parameters:**
```cpp
// Parameter declaration
const AP_Param::GroupInfo Parameters::var_info[] = {
    // Flight modes
    AP_GROUPINFO("RTL_ALT", 1, Parameters, rtl_altitude, 1500),
    
    // Motors
    AP_GROUPINFO("MOT_MAX", 2, Parameters, motor_max, 2000),
    AP_GROUPINFO("MOT_MIN", 3, Parameters, motor_min, 1000),
    
    // Safety
    AP_GROUPINFO("ARMING_CHECK", 4, Parameters, arming_check, 1),
    
    AP_GROUPEND
};
```

**Parameter Categories:**
- **Blimp_** — Blimp-specific parameters
- **MOT_** — Motor settings
- **RTL_** — Return-to-launch parameters
- **ARMING_** — Arming checks
- **LOG_** — Logging settings

### 3. mode.cpp / mode.h

**Purpose:** Base class for all flight modes

**Mode Hierarchy:**
```
Mode (base class)
   ├── ModeManual        -> Manual control
   ├── ModeLoiter        -> Position hold
   ├── ModeHold          -> Point hold
   ├── ModeAuto          -> Automatic mission
   ├── ModeLand          -> Landing
   ├── ModeRTL           -> Return to launch
   └── ModeVelocity      -> Velocity control
```

**Mode Interface:**
```cpp
class Mode {
public:
    // Mode initialization
    virtual bool init(bool ignore_checks) = 0;
    
    // Main mode loop
    virtual void run() = 0;
    
    // Mode availability check
    virtual bool requires_GPS() const = 0;
    
    // Get name
    virtual const char* name() const = 0;
    
protected:
    // Common methods
    void set_desired_velocity(float vx, float vy, float vz);
    void set_desired_position(float x, float y, float z);
};
```

### 4. motors.cpp

**Purpose:** Motor output and arming checks

**Key Functions:**
- `motors_output()` — sends final PWM signals to servos/motors via `SRV_Channels`
- `arm_motors_check()` — handles arming/disarming logic
- Calls `motors->output()` which delegates to the Fins class

### 5. Fins.cpp / Fins.h

**Purpose:** Actuator mixing — converts control outputs into per-motor signals

**Output fields (interface from Loiter):**
```cpp
float forward_out;  // forward movement, -1 to +1
float roll_out;     // roll rotation, -1 to +1
float pitch_out;    // pitch rotation, -1 to +1
float yaw_out;      // yaw rotation, -1 to +1
```

**Supported frame types:**
| Frame | Enum | Description |
|-------|------|-------------|
| `FISHBLIMP` | 1 | 4 fin servos (Back, Front, Right, Left) — sinusoidal flapping |
| `FOUR_MOTOR` | 2 | 4 motors (FrontLeft, FrontRight, Up, Right) — linear mixing |
| `ROTARY_BLIMP` | 3 | 4 motors — forward + 3 rotational axes mixing |

**Mixing matrix:** Each motor has `_amp_factor` coefficients that define how much of each output axis it responds to:
```cpp
_thrpos[i] = forward_out * _forward_amp_factor[i]
           + roll_out    * _roll_amp_factor[i]
           + pitch_out   * _pitch_amp_factor[i]
           + yaw_out     * _yaw_amp_factor[i];
```

**Key methods:**
- `setup_rotary()` / `setup_motors()` / `setup_fins()` — define mixing matrix per frame
- `output_rotary()` / `output_motors()` / `output_fins()` — apply mixing and send to SRV_Channels
- `output_min()` — zero all outputs
- `get_throttle()` — max absolute output across all axes (for MAVLink indicator)

### 6. GCS_Blimp.cpp / GCS_MAVLink_Blimp.cpp

**Purpose:** Ground Control Station communication

**Protocols:**
- **MAVLink** — Main data exchange protocol
- **UDP/TCP** — Transport protocols

**Transmitted Data:**
- Position (GPS coordinates)
- Orientation (roll, pitch, yaw)
- Speed and altitude
- Battery status
- Motor status
- Sensor telemetry

**Received Commands:**
- Control commands
- Parameter changes
- Mission uploads
- Arming/disarming commands

---

## Build System

### Waf Build System

The project uses **Waf** — a Python-based build system, standard for ArduPilot.

### wscript File

```python
def build(bld):
    # Vehicle name
    vehicle = bld.path.name
    
    # Create static library
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AC_InputManager',
            'AP_Avoidance',
            'AP_LTM_Telem',
            'AP_Devo_Telem',
            'AP_KDECAN',
            'AP_AdvancedFailsafe',   # TODO for some reason compiling GCS_Common.cpp (in libraries) fails if this isn't included.
            'AC_AttitudeControl',    # for PSCx logging
        ],
    )
    
    # Create executable
    bld.ap_program(
        program_name='blimp',
        program_groups=['bin', 'blimp'],
        use=vehicle + '_libs',
    )
```

### Build Process

```bash
# 1. Clone repository
git clone https://github.com/DaniK-51/ArduMotorBlimp.git
cd ArduMotorBlimp

# 2. Integration with ArduPilot
# (usually copied to ArduPilot/Blimp/ folder)

# 3. Configuration
./waf configure --board sitl

# 4. Build
./waf blimp

# 5. Result
./build/sitl/bin/blimp
```

### Dependencies

**Common ArduPilot Libraries:**
- AP_Common — Common utilities
- AP_HAL — Hardware Abstraction Layer
- AP_Math — Mathematics
- AP_Param — Parameter system
- AP_Scheduler — Task scheduler

**Specific Libraries:**
- AP_AHRS — Attitude system
- AP_NavEKF3 — Kalman filter
- AP_Motors — Motor control
- RC_Channel — RC channels
- GCS_MAVLink — MAVLink protocol
- AP_Logger — Logging

---

## Flight Modes

### 1. Manual

**File:** `mode_manual.cpp`

**Description:** Direct motor control via RC transmitter

**Characteristics:**
- No stabilization
- Full pilot control
- Requires constant attention

### 2. Loiter (Position Hold)

**Files:** `mode_loiter.cpp`, `Loiter.cpp`

**Description:** Automatic current position hold

**Characteristics:**
- Uses GPS for positioning
- PID controller for drift correction
- Wind compensation

**Algorithm:**
```
1. Remember current position (GPS)
2. Read current position
3. Calculate error (desired - current)
4. PID controller -> speed correction
5. Send commands to motors
```

### 3. Hold

**File:** `mode_hold.cpp`

**Description:** Current position and altitude hold

**Difference from Loiter:**
- Stricter hold
- Smaller allowed error
- Uses barometer for altitude

### 4. Auto (Automatic)

**File:** `mode_auto.cpp`

**Description:** Execute pre-defined mission

**Capabilities:**
- Follow waypoints
- Execute commands (takeoff, land, delay)
- Automatic return on connection loss

### 5. Land

**File:** `mode_land.cpp`

**Description:** Automatic landing

**Algorithm:**
```
1. Descend with controlled speed
2. Maintain horizontal position
3. Detect ground (range finder)
4. Disable motors after landing
```

### 6. RTL (Return-To-Launch)

**File:** `mode_rtl.cpp`

**Description:** Automatic return to takeoff point

**Sequence:**
```
1. Climb to safe altitude
2. Fly to launch point (GPS)
3. Descend and land
```

### 7. Velocity (Velocity Control)

**File:** `mode_velocity.cpp`

**Description:** Control via velocity command

**Applications:**
- Precise motion control
- External system integration
- Wind compensation

---

## Control System

### Control Architecture

```
+-------------------------------------------------------------+
|  Pilot / GCS                                                |
|  (RC channels / MAVLink commands)                           |
+----------------------------+--------------------------------+
                             |
                             v
+-------------------------------------------------------------+
|  Mode (current flight mode)                                 |
|  - ModeManual    -> direct pilot control                    |
|  - ModeVelocity  -> target velocity -> Loiter.run_vel()     |
|  - ModeLoiter    -> target position -> Loiter.run()         |
|  - Auto          -> mission (SCurve) -> Loiter.run()        |
|  - RTL           -> return home     -> Loiter.run()         |
|  - Land          -> landing         -> Loiter.run_vel()     |
|  - Hold          -> full stop (all outputs = 0)             |
+----------------------------+--------------------------------+
                             |
                             v
+-------------------------------------------------------------+
|  Loiter (cascade PID controller)                            |
|                                                             |
|  +-------------------+     +-------------------+            |
|  |  Position PID      |---->|  Velocity PID      |           |
|  |  pid_pos_{x,y,z,   |     |  pid_vel_{x,y,z,   |           |
|  |        yaw}        |     |        yaw}        |           |
|  |  pos -> vel        |     |  vel -> force      |           |
|  +-------------------+     +--------+----------+            |
|                                     |                        |
|  Scaler: compensates coupled axes  |                        |
|  (if motor0 is used for both       |                        |
|   forward and pitch, scaler        |                        |
|   reduces each axis contribution)  |                        |
+-------------------------------------+-----------------------+
                                      |
        motors->{forward_out, roll_out, pitch_out, yaw_out}
                                      |
                                      v
+-------------------------------------------------------------+
|  Fins (actuator mixing)                                     |
|                                                             |
|  Takes 4 values and matrix-mixes into motor signals:        |
|                                                             |
|  _thrpos[i] = forward_out * _forward_amp_factor[i]          |
|             + roll_out    * _roll_amp_factor[i]             |
|             + pitch_out   * _pitch_amp_factor[i]            |
|             + yaw_out     * _yaw_amp_factor[i]              |
|                                                             |
|  Frames: FISHBLIMP (fins), FOUR_MOTOR, ROTARY_BLIMP         |
+----------------------------+--------------------------------+
                             |
                             v
+-------------------------------------------------------------+
|  Motors / Servos                                            |
|  (PWM signals via SRV_Channels)                             |
+-------------------------------------------------------------+
```

### Separation of Concerns

**Loiter** -- does not know how motors are wired. Writes 4 values:
`forward_out` (forward), `roll_out` (roll), `pitch_out` (pitch), `yaw_out` (yaw).

**Fins** -- does not know where these values come from. Takes 4 values and through the `_amp_factor` matrix converts them into per-motor/servo signals.

This separation allows changing the Fins interface without touching PID, and vice versa.

### PID Cascade (Loiter)

Two operating modes:

1. **Full cascade** (`run()`): position -> velocity -> motors. Used in Loiter, RTL, Auto.
2. **Velocity only** (`run_vel()`): velocity -> motors. Used in Velocity, Land.

### Loiter Parameters

| Parameter | Purpose |
|-----------|---------|
| `LOIT_VEL{X,Y,Z,YAW}_{P,I,D}` | Velocity PID per axis |
| `LOIT_POS{X,Y,Z,YAW}_{P,I,D}` | Position PID per axis |
| `LOIT_MAX_VEL{X,Y,Z,YAW}` | Maximum velocity |
| `LOIT_MAX_POS{X,Y,Z,YAW}` | Maximum position change rate |
| `LOIT_DIS_MASK` | Axis disable bitmask |
| `LOIT_PID_DZ` | Position PID deadzone (m) |
| `LOIT_POS_LAG` | Allowed target position lag (s) |

### PID Examples

All PID controllers are implemented in the `Loiter` class via the `AC_PID` library.

**Position PID (position -> velocity):**
```cpp
// Loiter::run()
target_vel_ef.x = pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
target_vel_ef.y = pid_pos_y.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
target_vel_ef.z = pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
target_vel_yaw  = pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
```

**Velocity PID (velocity -> force/moment):**
```cpp
// Loiter::run_vel()
actuator.x = pid_vel_x.update_all(target_vel_bf_c.x * scaler_x, vel_bf_filtd.x, dt, limit.x);
actuator.y = pid_vel_y.update_all(target_vel_bf_c.y * scaler_y, vel_bf_filtd.y, dt, limit.y);
act_down   = pid_vel_z.update_all(target_vel_bf_c.z * scaler_z, vel_bf_filtd.z, dt, limit.z);
act_yaw    = pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd, dt, limit.yaw);
```

**Direct pilot control (Manual mode, no PID):**
```cpp
// ModeManual::run()
motors->forward_out = pilot.x * g.max_man_thr;
motors->roll_out    = pilot.y * g.max_man_thr;
motors->pitch_out   = pilot.z * g.max_man_thr;
motors->yaw_out     = pilot_yaw * g.max_man_thr;
```

---

## Telemetry and Communication

### MAVLink Protocol

**Main Messages (Transmitted):**

| Message | Frequency | Description |
|---------|-----------|-------------|
| **HEARTBEAT** | 1 Hz | System status |
| **ATTITUDE** | 10-50 Hz | Roll, pitch, yaw |
| **GLOBAL_POSITION_INT** | 10 Hz | GPS position |
| **SYS_STATUS** | 1 Hz | System state |
| **VFR_HUD** | 5 Hz | Speed, altitude |
| **RC_CHANNELS** | 5 Hz | RC channels |
| **SERVO_OUTPUT_RAW** | 5 Hz | PWM outputs |

**Main Commands (Received):**

| Command | Description |
|---------|-------------|
| **COMMAND_LONG** | Long commands (takeoff, RTL) |
| **SET_POSITION_TARGET_GLOBAL_INT** | Target position |
| **SET_ATTITUDE_TARGET** | Target attitude |
| **PARAM_SET** | Set parameter |
| **MISSION_ITEM** | Mission item |

### GCS_Blimp

**Functions:**
```cpp
// Send telemetry
void GCS_Blimp::send_telemetry() {
    send_heartbeat();
    send_attitude();
    send_position();
    send_battery_status();
    send_rc_channels();
}

// Process commands
void GCS_Blimp::handle_message(mavlink_message_t& msg) {
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;
        // ... other commands
    }
}
```

### Logging (Log.cpp)

**Log Types:**
- **ATT** — Attitude
- **POS** — Position
- **MOT** — Motors
- **BAT** — Battery
- **GPS** — GPS data
- **IMU** — IMU data
- **CMD** — Commands

**Format:**
```cpp
// Write log
struct Log_Attitude {
    float roll;
    float pitch;
    float yaw;
    float roll_desired;
    float pitch_desired;
};

logger.Write("ATT", "roll,pitch,yaw,roll_d,pitch_d", 
             "fff,ff", 
             attitude.roll, 
             attitude.pitch, 
             attitude.yaw,
             attitude.roll_desired,
             attitude.pitch_desired);
```

---

## Safety

### AP_Arming_Blimp

**Pre-arming Checks:**

```cpp
bool AP_Arming_Blimp::pre_arm_checks() {
    bool success = true;
    
    // 1. GPS check
    if (!gps_ok()) {
        gcs().send_text("PreArm: GPS not ready");
        success = false;
    }
    
    // 2. EKF check
    if (!ekf_ok()) {
        gcs().send_text("PreArm: EKF not ready");
        success = false;
    }
    
    // 3. Battery check
    if (battery_voltage < min_voltage) {
        gcs().send_text("PreArm: Battery too low");
        success = false;
    }
    
    // 4. RC calibration check
    if (!rc_calibrated()) {
        gcs().send_text("PreArm: RC not calibrated");
        success = false;
    }
    
    // 5. Sensor check
    if (!sensors_ok()) {
        gcs().send_text("PreArm: Sensors not healthy");
        success = false;
    }
    
    return success;
}
```

### Failsafe

**Failsafe Scenarios:**

1. **RC Signal Loss:**
   ```
   1. Detect signal loss (> 500ms)
   2. Switch to RTL mode
   3. Return home and land
   ```

2. **Low Battery:**
   ```
   1. Warning (warning level)
   2. Critical warning (critical level)
   3. Automatic landing (emergency level)
   ```

3. **GPS Loss:**
   ```
   1. Switch to GPS-less mode
   2. Hold position using barometer
   3. Warn pilot
   ```

4. **EKF Error:**
   ```
   1. Detect EKF divergence
   2. Switch to backup EKF
   3. If fails - land
   ```

### EKF Check (ekf_check.cpp)

**EKF Checks:**
```cpp
bool EKF_Check::healthy() {
    // Position variance check
    if (pos_variance > threshold) {
        return false;
    }
    
    // Velocity variance check
    if (vel_variance > threshold) {
        return false;
    }
    
    // Convergence check
    if (!ekf_converged()) {
        return false;
    }
    
    return true;
}
```

---

## Parameters

### Parameter Categories

**Blimp Parameters:**
```cpp
// Configuration
Blimp_TYPE           // Blimp type
Blimp_ENABLE         // System enable
Blimp_OPTIONS        // Options

// Motors
Blimp_MOT_MAX        // Maximum PWM
Blimp_MOT_MIN        // Minimum PWM
Blimp_MOT_IDLE       // Idle
Blimp_MOT_SPIN_MIN   // Minimum spin

// Control
Blimp_RTL_ALT        // Return altitude (cm)
Blimp_LAND_SPEED     // Landing speed
Blimp_LOITER_SPEED   // Loiter speed

// Safety
Blimp_FS_ENABLE      // Failsafe enable
Blimp_FS_THR_ENABLE  // Throttle failsafe
Blimp_FS_THR_VALUE   // Throttle threshold
```

### Loading/Saving Parameters

```bash
# In MAVProxy
param load blimp_params.parm    # Load parameters
param save blimp_params.parm    # Save parameters
param show                      # Show all parameters
param set Blimp_RTL_ALT 2000    # Set parameter
param diff                      # Show differences from default
```

---

## Integration with ArduPilot

### Connection with Main Repository

**ArduMotorBlimp** is a **fork/module** of the official ArduPilot repository.

**Integration Structure:**
```
ardupilot/                    # Main ArduPilot repository
└── Blimp/                    # Our module (copy of ArduMotorBlimp)
    ├── Blimp.cpp
    ├── mode.cpp
    └── ...
    
libraries/                    # Common libraries
├── AP_Motors/
├── AP_AHRS/
── AP_NavEKF3/
└── ...
```

### Differences from Standard Blimp

**ArduMotorBlimp** contains:
- Initial blimp implementation
- Custom flight modes
- Fins interface with 3 frame types (FISHBLIMP, FOUR_MOTOR, ROTARY_BLIMP)
- Cascade PID controller (Loiter) with position/velocity PIDs and axis scaler
- Extended safety checks

### Update Process

```bash
# Sync with ArduPilot
git remote add ardupilot https://github.com/ArduPilot/ardupilot.git
git fetch ardupilot
git merge ardupilot/master
git push origin main
```

---

## Connection with the_blimp_swp Project

### Architectural Connection

```
+-------------------------------------------------------------+
|  the_blimp_swp                                              |
|  (team repository)                                          |
|                                                             |
|  +-- sitl/                      # Docker + SITL             |
|  |   +-- Dockerfile                                         |
|  |   +-- docker-compose.yml                                 |
|  |   +-- params/blimp.parm                                  |
|  |   +-- scripts/blimp_motor_control.lua                    |
|  |                                                          |
|  +-- docs/                      # Documentation             |
|      +-- REPOSITORY_STRUCTURE.md                            |
+-----------------------------+-------------------------------+
                              | Tests and uses
                              v
+-------------------------------------------------------------+
|  ArduMotorBlimp                                             |
|  (firmware repository)                                      |
|                                                             |
|  +-- Blimp.cpp                  # Main application          |
|  +-- mode.cpp                   # Flight modes              |
|  +-- motors.cpp                 # Motor output              |
|  +-- ...                        # All components            |
+-----------------------------+-------------------------------+
                              | Based on
                              v
+-------------------------------------------------------------+
|  ArduPilot                                                  |
|  (official repository)                                      |
|                                                             |
|  +-- libraries/                 # Common libraries          |
|  +-- Tools/                     # Build tools               |
|  +-- wscript                    # Build system              |
+-------------------------------------------------------------+
```

---

## Useful Commands

### Build and Run

```bash
# Build for SITL
./waf configure --board sitl
./waf blimp

# Run
./build/sitl/bin/blimp --model +

# With custom parameters
./build/sitl/bin/blimp --model + --add-param-file=blimp.parm
```

### Debugging

```bash
# View logs
mavlog.py *.bin

# Analyze parameters
param.py show

# Check status
mavproxy.py --master=127.0.0.1:14550
```

### Git Workflow

```bash
# Create branch
git checkout -b feature/new-mode

# Make changes
git add .
git commit -m "feat: add new flight mode"

# Push
git push origin feature/new-mode

# Create PR
# (on GitHub)
```

---

## Appendices

### A. Glossary

| Term | Definition |
|------|------------|
| **Arming** | Enabling motors (preparation for flight) |
| **Disarming** | Disabling motors |
| **Failsafe** | Automatic reaction to emergency situation |
| **EKF** | Extended Kalman Filter — navigation filter |
| **GCS** | Ground Control Station |
| **Loiter** | Position hold |
| **RTL** | Return-To-Launch |
| **PWM** | Pulse Width Modulation — motor control |
| **RC** | Radio Control |
| **MAVLink** | Autopilot communication protocol |
| **SITL** | Software In The Loop — simulation |
| **PID** | Proportional-Integral-Derivative — controller |

### B. Useful Links

**Official Resources:**
- [ArduPilot.org](https://ardupilot.org/)
- [Developer Wiki](https://ardupilot.org/dev/)
- [ArduMotorBlimp GitHub](https://github.com/DaniK-51/ArduMotorBlimp)
- [Discord](https://discord.com/invite/ardupilot)

**Documentation:**
- [Building ArduPilot](https://ardupilot.org/dev/docs/building-the-code.html)
- [Parameters List](https://ardupilot.org/copter/docs/parameters.html)
- [MAVLink Protocol](https://mavlink.io/)

### C. Pre-commit Checklist

- [ ] Code compiles without errors
- [ ] No compiler warnings
- [ ] All tests pass
- [ ] Documentation updated
- [ ] Parameters documented
- [ ] License compliance (GPL-3.0)

---

## Change History

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2026-07-01 | 1.0 | Initial document version | Blimp Team |
| 2026-07-01 | 1.1 | Removed Workflow and Development Plan sections | Blimp Team |

---

**This document will be updated as the ArduMotorBlimp project evolves.**

**Last updated:** July 1, 2026
