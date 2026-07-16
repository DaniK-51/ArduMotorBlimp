# ArduMotorBlimp Repository Structure and Architecture

**Version:** 2.0  
**Date:** July 16, 2026  
**Branch:** `feat/manual-only`  
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
8. [Safety](#safety)
9. [Parameters](#parameters)
10. [Integration with ArduPilot](#integration-with-ardupilot)

---

## Introduction

### Document Purpose

This document describes the architecture, structure, and components of the **ArduMotorBlimp** repository — a custom blimp vehicle implementation for the ArduPilot platform.

### Current State

This is the **manual-only** branch — a minimal build containing only Manual and BRAKE modes for first flight testing. All autonomous modes (Velocity, Loiter, RTL, AUTO) have been removed to reduce binary size and complexity.

---

## Project Overview

### What is ArduMotorBlimp?

**ArduMotorBlimp** is a motorized blimp (Lighter-Than-Air vehicle) implementation based on the official ArduPilot platform. It uses 4 static motors with a configurable mixing matrix instead of the original oscillating fins.

### Key Characteristics

| Parameter | Value |
|-----------|-------|
| **Vehicle Type** | Motorized blimp |
| **Languages** | C++ |
| **License** | GPL-3.0 |
| **Base** | Official ArduPilot repository |
| **Status** | Manual-only build for first flight |

### Main Features

- Configurable 4×4 motor mixing matrix
- Manual mode (direct RC passthrough)
- BRAKE mode (emergency stop for failsafe)
- MAVLink heartbeat and basic telemetry
- Parameter system via Mission Planner / QGroundControl

---

## Repository Structure

### File Tree

```
ArduMotorBlimp/
├── Core
│   ├── Blimp.cpp              # Main loop, scheduler, AHRS reading
│   ├── Blimp.h                # Main class definition
│   ├── config.h               # Compile-time configuration
│   ├── defines.h              # Constants and enums
│   └── version.h              # Firmware version
│
├── Parameters
│   ├── Parameters.cpp         # Parameter definitions
│   └── Parameters.h           # Parameter declarations
│
├── Flight Modes
│   ├── mode.cpp               # Base Mode class, get_pilot_input()
│   ├── mode.h                 # Mode enum and class declarations
│   ├── mode_manual.cpp        # Manual mode (direct passthrough)
│   └── mode_brake.cpp         # BRAKE mode (emergency stop)
│
├── Motor Control
│   ├── MotorMix.cpp           # 4×4 mixing matrix, servo output
│   ├── MotorMix.h             # MotorMix class declaration
│   └── motors.cpp             # motors_output() pipeline, arming check
│
├── RC Input
│   ├── radio.cpp              # RC channel reading, failsafe
│   └── RC_Channel.cpp         # RC channel processing
│
├── GCS / MAVLink
│   ├── GCS_Blimp.cpp          # GCS class, sensor status
│   ├── GCS_Blimp.h            # GCS_Blimp declaration
│   ├── GCS_Mavlink.cpp        # MAVLink message handling
│   └── GCS_Mavlink.h          # GCS_MAVLINK_Blimp declaration
│
├── Safety
│   ├── AP_Arming.cpp          # Arming checks
│   ├── AP_Arming.h
│   ├── failsafe.cpp           # Failsafe timer
│   ├── events.cpp             # Failsafe event handling
│   └── ekf_check.cpp          # EKF validation
│
├── System
│   ├── system.cpp             # init_ardupilot(), allocate_motors()
│   ├── AP_State.cpp           # Vehicle state flags
│   ├── commands.cpp           # Home position management
│   ├── inertia.cpp            # Inertial navigation
│   └── sensors.cpp            # Barometer reading
│
├── Logging
│   └── Log.cpp                # MOTORI/MOTORO log messages
│
└── Build
    └── wscript                # Waf build configuration
```

### File Count

| Category | Count |
|----------|-------|
| Source (.cpp) | 18 |
| Headers (.h) | 12 |
| Config | 3 |
| Documentation | 4 |
| **Total** | **37** |

---

## Main Components

### MotorMix — Motor Mixing Matrix

**Files:** `MotorMix.h`, `MotorMix.cpp`

Replaces the original `Fins` oscillating fin system with a simple 4×4 mixing matrix.

**4 Control Axes:**
| Axis | Field | Range | Description |
|------|-------|-------|-------------|
| Yaw | `yaw_out` | [-1, +1] | Rotation around Z |
| Pitch | `pitch_out` | [-1, +1] | Rotation around Y |
| Roll | `roll_out` | [-1, +1] | Rotation around X |
| X | `x_out` | [-1, +1] | Linear forward/backward |

**Mixing Logic:**
```cpp
for each motor m (0..3):
    motor_out[m] = motor_yaw[m]   * yaw_out
                 + motor_pitch[m] * pitch_out
                 + motor_roll[m]  * roll_out
                 + motor_x[m]     * x_out
    constrain(motor_out[m], -1, 1)
    SRV_Channels::set_output_scaled(k_motor(m+1), motor_out[m] * 1000)
```

**Parameters:** `M1_YAW`..`M4_X` (16 float parameters, range -1..1)

### Mode Manual

**File:** `mode_manual.cpp`

Direct RC passthrough — no PID, no stabilization.

```
CH1 (Roll)     → motors->roll_out
CH2 (Pitch)    → motors->x_out     (forward/backward)
CH3 (Throttle) → motors->pitch_out (altitude)
CH4 (Yaw)      → motors->yaw_out
```

### Mode BRAKE

**File:** `mode_brake.cpp`

Emergency stop — zeros all motor outputs. Used as failsafe target.

---

## Control System

### Data Flow

```
RC Receiver
    │ PWM [1000..2000]
    ▼
radio.cpp: read_radio()
    │ channel->get_control_in() / 1000 → [-1, +1]
    ▼
mode.cpp: get_pilot_input()
    │
    ▼
mode_manual.cpp: run()
    │ motors->*_out = pilot.*
    ▼
motors.cpp: motors_output()
    │
    ▼
MotorMix.cpp: output()
    │ [yaw, pitch, roll, x] × matrix → [M1..M4]
    ▼
SRV_Channels → PWM → Motors
```

### Scheduler

| Task | Rate | Priority |
|------|------|----------|
| INS update | FAST | 0 |
| motors_output | FAST | 1 |
| read_AHRS | FAST | 2 |
| update_flight_mode | FAST | 3 |
| update_home_from_EKF | FAST | 4 |
| rc_loop | 100 Hz | 3 |
| GCS update | 400 Hz | 51-54 |

---

## Flight Modes

### Manual (Mode 1)

**Purpose:** Direct pilot control  
**GPS Required:** No  
**Stabilization:** None  

Stick positions are mapped directly to motor mixing matrix inputs.

### BRAKE (Mode 0)

**Purpose:** Emergency stop  
**GPS Required:** No  
**Stabilization:** None  

All motor outputs set to zero. Used as failsafe target for:
- Radio signal loss
- EKF failure
- GCS connection loss

---

## Safety

### Failsafe Chain

```
RC signal lost (> 500ms)
    → set_failsafe_radio(true)
        → failsafe_radio_on_event()
            → set_mode_brake_failsafe()

EKF error detected
    → failsafe_ekf_event()
        → set_mode_brake_failsafe()
```

### Arming

- Motor interlock via rudder
- Throttle must be at zero for disarming
- Auto-disarm after configurable delay

---

## Parameters

### Motor Mixing (MOTOR_*)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `M1_YAW` | 0 | -1..1 | Motor 1 yaw contribution |
| `M1_PITCH` | 0 | -1..1 | Motor 1 pitch contribution |
| `M1_ROLL` | 0 | -1..1 | Motor 1 roll contribution |
| `M1_X` | 0 | -1..1 | Motor 1 X contribution |
| ... | ... | ... | ... |
| `M4_X` | 0 | -1..1 | Motor 4 X contribution |

### System

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FLTMODE1` | MANUAL | Flight mode on CH5 low |
| `FLTMODE2` | MANUAL | Flight mode on CH5 mid-low |
| `FLTMODE3` | MANUAL | Flight mode on CH5 mid |
| `FLTMODE4` | MANUAL | Flight mode on CH5 mid-high |
| `FLTMODE5` | MANUAL | Flight mode on CH5 high |
| `SYSID_THISMAV` | 1 | MAVLink system ID |
| `ARMING_DELAY` | 2s | Delay after arming |

### Failsafe

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FS_THR_ENABLE` | 3 | Throttle failsafe: 0=Disabled, 3=Always BRAKE |
| `FS_THR_VALUE` | 975 | Throttle failsafe threshold (PWM) |
| `FS_EKF_ACTION` | 1 | EKF failsafe: 1=Switch to BRAKE |

---

## Integration with ArduPilot

### Required Upstream Libraries

This vehicle code depends on ArduPilot libraries (not included in this repo):

- `AP_HAL` — Hardware Abstraction Layer
- `AP_AHRS` — Attitude and Heading Reference System
- `AP_Param` — Parameter storage
- `AP_Scheduler` — Task scheduling
- `AP_Logger` — Data logging
- `SRV_Channel` — Servo output
- `RC_Channel` — RC input
- `GCS_MAVLink` — MAVLink protocol
- `AP_BattMonitor` — Battery monitoring
- `AP_Arming` — Arming checks
- `AP_InertialNav` — Inertial navigation

### Build

```bash
# Place this repo as ArduPilot/Blimp/
cd ardupilot
./waf configure --board sitl
./waf blimp
```

---

## Change History

| Date | Version | Description |
|------|---------|-------------|
| 2026-07-01 | 1.0 | Initial document |
| 2026-07-16 | 2.0 | Updated for manual-only branch |

**Last updated:** July 16, 2026
