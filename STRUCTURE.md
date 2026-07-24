# ArduMotorBlimp Repository Structure and Architecture

**Version:** 3.0  
**Date:** July 24, 2026  
**Branch:** `feat/manual-only`  
**Repository:** https://github.com/DaniK-51/ArduMotorBlimp  
**Author:** DaniK-51 (Daniyar)  
**License:** GPL-3.0  
**ArduPilot Base:** Copter-4.6.3

---

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
3. [Repository Structure](#repository-structure)
4. [Main Components](#main-components)
5. [Control System](#control-system)
6. [Flight Modes](#flight-modes)
7. [Safety](#safety)
8. [Parameters](#parameters)
9. [Build System](#build-system)
10. [Integration with ArduPilot](#integration-with-ardupilot)

---

## Introduction

This is the **manual-only** branch — a minimal build containing only Manual and BRAKE modes for first flight testing. All autonomous modes, sensors (AHRS, EKF, GPS), and PID controllers have been removed to minimize binary size and complexity.

**No AHRS, EKF, GPS, compass, or barometer required.** Only RC input → MotorMix → Motors.

---

## Project Overview

**ArduMotorBlimp** is a motorized blimp (Lighter-Than-Air vehicle) based on ArduPilot. It uses 4 static motors with a configurable mixing matrix via `AP_MotorsBlimp` (inherits `AP_Motors`).

| Parameter | Value |
|-----------|-------|
| **Vehicle Type** | Motorized blimp (MAV_TYPE_AIRSHIP) |
| **Language** | C++ |
| **License** | GPL-3.0 |
| **ArduPilot Base** | Copter-4.6.3 |
| **Target Board** | MicoAir743v2 (ChibiOS) |

---

## Repository Structure

```
ArduMotorBlimp/
├── Blimp.cpp              # Main loop, scheduler, constructor
├── Blimp.h                # Main class definition
├── AP_MotorsBlimp.cpp     # Inherits AP_Motors, mixing matrix, servo output
├── AP_MotorsBlimp.h       # AP_MotorsBlimp class declaration
├── AP_Arming.cpp          # Arming checks (simplified for manual-only)
├── AP_Arming.h
├── AP_State.cpp           # Vehicle state flags
├── GCS_Blimp.cpp          # GCS class, sensor status
├── GCS_Blimp.h
├── GCS_Mavlink.cpp        # MAVLink message handling (stripped)
├── GCS_Mavlink.h
├── Log.cpp                # MOTORI/MOTORO log messages
├── Parameters.cpp         # Parameter definitions
├── Parameters.h
├── RC_Channel.cpp         # RC channel processing, AUX functions (ARMDISARM)
├── RC_Channel.h
├── config.h               # Compile-time configuration
├── defines.h              # Constants and enums
├── events.cpp             # Failsafe event handling
├── failsafe.cpp           # Failsafe timer (main loop hang detection)
├── mode.cpp               # Base Mode class, get_pilot_input()
├── mode.h                 # Mode enum and class declarations
├── mode_brake.cpp         # BRAKE mode (emergency stop)
├── mode_manual.cpp        # Manual mode (direct passthrough)
├── motors.cpp             # motors_output() pipeline
├── radio.cpp              # RC channel reading, failsafe
├── system.cpp             # init_ardupilot(), allocate_motors()
├── version.h
└── wscript                # Waf build configuration
```

**28 source files.**

---

## Main Components

### AP_MotorsBlimp

Inherits `AP_Motors` from upstream ArduPilot. Provides:
- Configurable 4×4 mixing matrix (M1_ROLL..M4_THR parameters)
- Motor protocol selection via `MOTOR_PWM_TYPE` (DSHOT, OneShot, PWM)
- Bidirectional motor output (PWM 1000-2000)
- Arming/disarm state management
- Spool state machine

**4 Control Axes:**

| Axis | API Method | Range | Description |
|------|-----------|-------|-------------|
| Roll | `set_roll()` | [-1, +1] | Rotation around X |
| Pitch | `set_pitch()` | [-1, +1] | Rotation around Y |
| Yaw | `set_yaw()` | [-1, +1] | Rotation around Z |
| X | `set_throttle()` | [0, +1] | Linear forward/backward |

**Mixing Logic:**
```cpp
for each motor m (0..3):
    motor_out[m] = motor_roll[m]  * roll_out
                 + motor_pitch[m] * pitch_out
                 + motor_yaw[m]   * yaw_out
                 + motor_x[m]     * throttle
    constrain(motor_out, 0, 1)
    pwm = 1000 + motor_out * 1000  // [1000, 2000]
    rc_write(motor_func, pwm)
```

---

## Control System

### Data Flow (Manual Mode)

```
RC Receiver → radio.cpp → mode_manual.cpp → AP_MotorsBlimp → Motors
```

**No AHRS, EKF, GPS, or other sensors required.**

### Scheduler

| Task | Rate | Priority |
|------|------|----------|
| INS update | FAST | 0 |
| motors_output | FAST | 1 |
| update_flight_mode | FAST | 3 |
| rc_loop | 100 Hz | 3 |
| arm_motors_check | 10 Hz | 18 |
| notify update | 50 Hz | 36 |
| GCS receive | 400 Hz | 51 |
| GCS send | 400 Hz | 54 |

---

## Flight Modes

| Mode | Number | Description |
|------|--------|-------------|
| **BRAKE** | 0 | Emergency stop — zeros all motors, used for failsafe |
| **MANUAL** | 1 | Direct RC passthrough to mixing matrix |

### Manual Mode

Stick positions mapped directly to motor mixing matrix:
```
CH1 (Roll)     → motors->set_roll()
CH2 (Pitch)    → motors->set_throttle()  (forward/backward)
CH3 (Throttle) → motors->set_pitch()     (rotation Y)
CH4 (Yaw)      → motors->set_yaw()
```

### BRAKE Mode

All motor outputs zeroed. Used as failsafe target.

---

## Safety

### Failsafe Chain

```
RC signal lost (> 500ms) → set_failsafe_radio(true) → BRAKE
Battery failsafe         → handle_battery_failsafe() → BRAKE
GCS connection lost      → failsafe_gcs_check() → BRAKE
Main loop hang (2s)      → failsafe_check() → output_min()
```

### Arming

Button-based via AUX channel:
1. Set RC channel (5-8) to `AUX_FUNC=31` (ARMDISARM)
2. Toggle switch HIGH → arm, LOW → disarm

No rudder arming.

---

## Parameters

### Motor Mixing (MOTOR_*)

16 float parameters for the 4×4 mixing matrix:

| Motor | M_ROLL | M_PITCH | M_YAW | M_X |
|-------|--------|---------|-------|-----|
| M1 | -1..1 | -1..1 | -1..1 | -1..1 |
| M2 | -1..1 | -1..1 | -1..1 | -1..1 |
| M3 | -1..1 | -1..1 | -1..1 | -1..1 |
| M4 | -1..1 | -1..1 | -1..1 | -1..1 |

### Motor Protocol

`MOTOR_PWM_TYPE` parameter:

| Value | Protocol |
|-------|----------|
| 0 | Normal PWM |
| 1 | OneShot |
| 2 | OneShot125 |
| 3 | Brushed |
| 4 | DShot150 |
| 5 | DShot300 (default) |
| 6 | DShot600 |
| 7 | DShot1200 |

### Flight Modes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FLTMODE1`-`FLTMODE6` | MANUAL | Flight mode per CH5 position |

### Failsafe

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FS_THR_ENABLE` | 3 | 0=Disabled, 3=Always BRAKE |
| `FS_THR_VALUE` | 975 | Throttle failsafe threshold (PWM) |
| `FS_GCS_ENABLE` | 0 | GCS failsafe: 0=Disabled |

### System

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SYSID_THISMAV` | 1 | MAVLink system ID |
| `ARMING_DELAY` | 2s | Delay after arming |
| `DISARM_DELAY` | 10s | Auto-disarm delay |

---

## Build System

### wscript

```python
ap_libraries = ap_common_vehicle_libraries() + [
    'AC_InputManager',
    'AP_InertialNav',
    'AP_Motors',            # required by AP_MotorsBlimp
    'AC_AttitudeControl',
    'AP_AdvancedFailsafe',  # required by GCS_Common.cpp
    'AP_Avoidance',         # required by RC_Channel base class
    'AP_Winch',             # required by Lua bindings
    'AC_PrecLand',          # required by Lua bindings
    'AP_Follow',            # required by Lua bindings
]
```

**Note:** AP_AdvancedFailsafe, AP_Avoidance, AP_Winch, AC_PrecLand, AP_Follow are required by base classes (GCS_MAVLink, RC_Channel, AP_Scripting) even though we don't use them directly.

### Build Commands

```bash
./waf configure --board MicoAir743v2
./waf blimp
```

---

## Integration with ArduPilot

This vehicle code depends on upstream ArduPilot libraries (NOT included in this repo):

- `AP_Motors` — Motor base class (AP_MotorsBlimp inherits from this)
- `AP_HAL` — Hardware Abstraction Layer
- `AP_Param` — Parameter storage
- `AP_Scheduler` — Task scheduling
- `AP_Logger` — Data logging
- `SRV_Channel` — Servo/PWM output
- `RC_Channel` — RC input
- `GCS_MAVLink` — MAVLink protocol
- `AP_Arming` — Arming checks
- `AC_PID` — PID controllers (used by AP_Motors base)

Build requires the full ArduPilot repo structure with `libraries/`, `Tools/`, `modules/`.

---

## Change History

| Date | Version | Description |
|------|---------|-------------|
| 2026-07-01 | 1.0 | Initial document |
| 2026-07-16 | 2.0 | Updated for manual-only branch |
| 2026-07-24 | 3.0 | AP_MotorsBlimp, stripped GCS, removed sensor files |

**Last updated:** July 24, 2026
