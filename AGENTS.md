# AI Contribution Guidelines for ArduMotorBlimp

This document provides guidelines for AI assistants contributing code to the ArduMotorBlimp project. This repository contains the **Blimp vehicle code** extracted from the official [ArduPilot](https://github.com/ArduPilot/ardupilot) repository.

**Important**: This repo does NOT contain `libraries/`, `Tools/`, `modules/`, or other vehicle directories. Those are managed in the upstream [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) repo. We only modify vehicle-level code here.

**Current branch**: `feat/manual-only` — minimal Manual + BRAKE mode build for first flight testing.

---

## Table of Contents

- [1. Code of Conduct & Ethics](#1-code-of-conduct--ethics)
- [2. Repository Structure](#2-repository-structure)
- [3. Architecture](#3-architecture)
- [4. Coding Style](#4-coding-style)
- [5. Build System](#5-build-system)
- [6. Parameter Documentation](#6-parameter-documentation)
- [7. Commit Messages](#7-commit-messages)
- [8. What AI Should NOT Do](#8-what-ai-should-not-do)

---

## 1. Code of Conduct & Ethics

- **Never** generate code that supports weaponization or code specific to the control of aircraft in control of human life.
- **Never** fabricate test results, log data, or claim testing that was not actually performed.

---

## 2. Repository Structure

Current files (manual-only branch):

```text
ArduMotorBlimp/
├── Core
│   ├── Blimp.cpp              # Main loop, scheduler, constructor
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
│   ├── AP_MotorsBlimp.cpp     # Inherits AP_Motors, mixing matrix, servo output
│   ├── AP_MotorsBlimp.h       # AP_MotorsBlimp class declaration
│   └── motors.cpp             # motors_output() pipeline, arming check
│
├── RC Input
│   ├── radio.cpp              # RC channel reading, failsafe
│   └── RC_Channel.cpp         # RC channel processing, AUX functions
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
│   ├── failsafe.cpp           # Failsafe timer (main loop hang detection)
│   └── events.cpp             # Failsafe event handling (radio, battery, GCS)
│
├── System
│   ├── system.cpp             # init_ardupilot(), allocate_motors()
│   ├── AP_State.cpp           # Vehicle state flags
│   └── radio.cpp              # Radio input handling
│
├── Logging
│   └── Log.cpp                # MOTORI/MOTORO log messages
│
└── Build
    └── wscript                # Waf build configuration
```

### Key conventions

- The main class `Blimp` inherits from `AP_Vehicle`.
- `AP_MotorsBlimp` inherits from `AP_Motors` (not custom MotorMix).
- Mode implementations are in separate `mode_*.cpp` files.
- Parameters use `AP_GROUPINFO` macros in `Parameters.cpp`.
- The vehicle uses libraries from the upstream ArduPilot repo (e.g., `AP_HAL`, `AP_Motors`, `SRV_Channel`). Those are NOT modified here.

---

## 3. Architecture

### Data Flow (Manual Mode)

```
RC Receiver → radio.cpp → mode_manual.cpp → AP_MotorsBlimp → Motors
```

**No AHRS, EKF, GPS, or other sensors required for Manual mode.**

### Motor Control

`AP_MotorsBlimp` inherits from `AP_Motors` and provides:
- Configurable 4×4 mixing matrix (M1_ROLL..M4_THR parameters)
- Motor protocol selection via `MOTOR_PWM_TYPE` (DSHOT, OneShot, etc.)
- Bidirectional motor output
- Arming/disarm state management

### Flight Modes

| Mode | Description |
|------|-------------|
| **MANUAL** (1) | Direct RC passthrough to mixing matrix |
| **BRAKE** (0) | Emergency stop — zeros all motors, used for failsafe |

### API for Motor Control

```cpp
motors->set_roll(value);     // [-1, +1] rotational around X
motors->set_pitch(value);    // [-1, +1] rotational around Y
motors->set_yaw(value);      // [-1, +1] rotational around Z
motors->set_throttle(value); // [0, +1] linear along X (forward/backward)
```

### Arming

Button-based arming via AUX channel:
1. Set RC channel (5-8) to `AUX_FUNC=31` (ARMDISARM)
2. Toggle switch HIGH to arm, LOW to disarm

---

## 4. Coding Style

### C++

ArduPilot enforces style via [astyle](https://github.com/ArduPilot/ardupilot/blob/master/Tools/CodeStyle/astylerc). The key rules:

| Rule | Convention |
|---|---|
| **Indentation** | 4 spaces, no tabs |
| **Brace style** | Linux/K&R — opening brace on same line |
| **Line endings** | LF only (no CRLF) |
| **Header guards** | `#pragma once` (not `#ifndef`) |
| **Single-line blocks** | Always add braces |

#### Naming conventions

| Element | Convention | Example |
|---|---|---|
| Classes | `AP_` or `AC_` prefix, PascalCase | `AP_MotorsBlimp` |
| Methods | `snake_case` | `output_armed_stabilizing()` |
| Member variables  | `_singleton`, `_armed` |
| Constants/defines | `UPPER_SNAKE_CASE` | `AP_MOTORS_BLIMP_NUM_MOTORS` |

#### Other conventions

- Format only the part that is changed, not all the files, to not break git history and blame.
- Use the singleton pattern with `get_singleton()` where appropriate.
- Prefer `is_zero()`, `is_positive()`, `is_negative()` over direct float comparisons.
- Use `GCS_SEND_TEXT()` for user-facing messages.
- Use `AP_HAL::millis()` / `AP_HAL::micros()` instead of platform-specific time functions.

---

## 5. Build System

ArduPilot uses [Waf](https://waf.io/book/). To build Blimp:

```sh
# Configure for SITL (software-in-the-loop, used for development)
./waf configure --board sitl

# Build Blimp
./waf blimp
```

**Important:** Never run `waf` with `sudo`. Always call `./waf` from the repository root.

Note: This repo contains only the vehicle code. The full build requires the upstream ArduPilot repo structure with `libraries/`, `Tools/`, and `modules/`.

---

## 6. Parameter Documentation

Parameters are documented inline in C++ using `@` annotations above `AP_GROUPINFO` macros:

```cpp
// @Param: M1_ROLL
// @DisplayName: Motor 1 roll factor
// @Description: How much motor 1 contributes to roll rotation.
// @Range: -1 1
// @User: Standard
AP_GROUPINFO("M1_ROLL", 1, AP_MotorsBlimp, _roll_factor[0], 0),
```

### Available annotations

- `@Param:` — short name (max 16 chars total with group prefix)
- `@DisplayName:` — human-readable name
- `@Description:` — detailed description
- `@Values:` — `value:label` pairs, comma-separated
- `@Range:` — `min max`
- `@Units:` — unit string
- `@User:` — `Standard` or `Advanced`
- `@RebootRequired:` — `True` if reboot is needed after change

---

## 7. Commit Messages

This project uses [Conventional Commits](https://www.conventionalcommits.org/):

```text
type(scope): short description of the change

Optional longer description explaining the motivation,
what was changed, and why.
```

### Types

| Type | Description |
|---|---|
| `feat` | New feature or functionality |
| `fix` | Bug fix |
| `docs` | Documentation only changes |
| `style` | Code style changes (formatting, no logic change) |
| `refactor` | Code refactoring (no feature or fix) |
| `test` | Adding or updating tests |
| `chore` | Build process, CI, or auxiliary tool changes |

### Scope

Optional. Use the affected component: `motors`, `mode`, `params`, `gcs`, `failsafe`, etc.

### Rules

- Keep the first line under ~72 characters.
- **No merge commits** — always rebase onto the target branch.
- One logical change per commit. Split unrelated changes into separate commits.
- No emoji, no jokes.

### Examples

```text
feat(motors): add AP_MotorsBlimp with configurable mixing matrix
fix(failsafe): correct BRAKE mode failsafe action
docs(motors): update parameter documentation for MOTOR_PWM_TYPE
refactor(mode): simplify manual mode passthrough
```

---

## 8. What AI Should NOT Do

- **Do not fabricate**: Never invent APIs, parameters, MAVLink messages, or hardware interfaces that don't exist in the codebase. Always verify against actual ArduPilot source code.
- **Do not guess at safety-critical logic**: If you are uncertain about failsafe behavior, stop and flag it for human review.
- **Do not bypass compile-time guards**: Respect `#if AP_<FEATURE>_ENABLED` guards.
- **Do not change parameter indices**: Existing `AP_GROUPINFO` index numbers are baked into user configurations. Changing them breaks parameter storage.
- **Do not add unnecessary dependencies**: ArduPilot runs on constrained embedded hardware. Every byte of RAM and flash matters.
- **Do not add sensor dependencies to Manual mode**: Manual mode must work without AHRS, EKF, GPS, or any sensors.
- **Do not use `set_output_pwm()` for motor output**: Use `set_output_scaled()` which goes through SRV_Channel protocol layer.
- **Do not modify upstream libraries or tools**: This repo only contains vehicle code. Library changes belong in the upstream [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) repo.
- **Do not leave dead code**: If a function/variable is no longer used, delete it completely.
- **Do not add comments on all functions/lines**: Document only what was changed and useful for future reading.
