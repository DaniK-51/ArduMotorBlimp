# AI Contribution Guidelines for ArduMotorBlimp

This document provides guidelines for AI assistants contributing code to the ArduMotorBlimp project. This repository contains only the **Blimp vehicle code** extracted from the official [ArduPilot](https://github.com/ArduPilot/ardupilot) repository.

**Important**: This repo does NOT contain `libraries/`, `Tools/`, `modules/`, or other vehicle directories. Those are managed in the upstream [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) repo. We only modify vehicle-level code here.

---

## Table of Contents

- [1. Code of Conduct & Ethics](#1-code-of-conduct--ethics)
- [2. Repository Structure](#2-repository-structure)
- [3. Coding Style](#3-coding-style)
- [4. Build System](#4-build-system)
- [5. Testing](#5-testing)
- [6. Parameter Documentation](#6-parameter-documentation)
- [7. Commit Messages](#7-commit-messages)
- [8. Pull Request Guidelines](#8-pull-request-guidelines)
- [9. What AI Should NOT Do](#9-what-ai-should-not-do)

---

## 1. Code of Conduct & Ethics

- **Never** generate code that supports weaponization or code specific to the control of aircraft in control of human life.
- **Never** fabricate test results, log data, or claim testing that was not actually performed.

---

## 2. Repository Structure

This repo contains only the Blimp vehicle code:

```text
ArduMotorBlimp/             # Root — Blimp vehicle code
  Blimp.cpp / Blimp.h       # Main vehicle class (inherits AP_Vehicle)
  mode_*.cpp                # Flight mode implementations (mode_hold, mode_loiter, etc.)
  mode.cpp / mode.h         # Mode base class and management
  Parameters.cpp / .h       # Vehicle parameter definitions
  GCS_Blimp.cpp / .h        # Ground Control Station interface
  GCS_Mavlink.cpp / .h      # MAVLink message handling
  RC_Channel.cpp / .h       # RC channel handling
  AP_Arming.cpp / .h        # Arming checks
  Fins.cpp / Fins.h         # Motor/fin control
  Loiter.cpp / Loiter.h     # Loiter controller
  motors.cpp                # Motor mixing
  radio.cpp                 # Radio input handling
  sensors.cpp               # Sensor data
  failsafe.cpp              # Failsafe logic
  commands.cpp              # Command processing
  events.cpp                # Event handling
  Log.cpp                   # Logging
  AP_State.cpp              # Vehicle state
  system.cpp                # System initialization
  config.h / defines.h      # Configuration and defines
  version.h                 # Version info
  wscript                   # Build configuration
```

### Key conventions

- The main class `Blimp` inherits from `AP_Vehicle`.
- Mode implementations are in separate `mode_*.cpp` files.
- Parameters use `AP_GROUPINFO` macros in `Parameters.cpp`.
- The vehicle uses libraries from the upstream ArduPilot repo (e.g., `AP_HAL`, `AP_AHRS`, `AC_PID`). Those are NOT modified here.

---

## 3. Coding Style

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
| Classes | `AP_` or `AC_` prefix, PascalCase | `AP_GPS`, `AC_PID` |
| Methods | `snake_case` | `get_altitude()`, `update_state()` |
| Member variables  | `_singleton`, `_primary` |
| Constants/defines | `UPPER_SNAKE_CASE` | `AP_MOTORS_MOT_1` |
| Compile-time flags | `AP_<NAME>_ENABLED` | `AP_TERRAIN_AVAILABLE` |

#### Other conventions

- Format only the part that is changed, not all the files, to not break git history and blame.
- Use the singleton pattern with `get_singleton()` and `CLASS_NO_COPY()` where appropriate.
- Use `extern const AP_HAL::HAL& hal;` at the top of `.cpp` files that need hardware access.
- Prefer `is_zero()`, `is_positive()`, `is_negative()` over direct float comparisons.
- Use `GCS_SEND_TEXT()` for user-facing messages, not `printf` or `gcs().send_text`.
- Use `AP_HAL::millis()` / `AP_HAL::micros()` instead of platform-specific time functions.
- Wrap feature code in `#if AP_<FEATURE>_ENABLED` / `#endif // AP_<FEATURE>_ENABLED` guards.

---

## 4. Build System

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

## 5. Testing

### 5.1 SITL Autotest (Integration Tests)

Tests spawn a simulated vehicle and execute scripted flight scenarios.

```sh
# Run a specific autotest with rebuild
Tools/autotest/autotest.py build.Blimp test.Blimp
```

### 5.2 C++ Unit Tests (GTest)

Located in `libraries/<lib>/tests/` (upstream repo). Use `#include <AP_gtest.h>`.

---

## 6. Parameter Documentation

Parameters are documented inline in C++ using `@` annotations above `AP_GROUPINFO` macros:

```cpp
// @Param: ENABLE
// @DisplayName: Terrain data enable
// @Description: enable terrain data. This enables the vehicle storing
//   a database of terrain data on the SD card.
// @Values: 0:Disable,1:Enable
// @User: Advanced
AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Terrain, enable, 1, AP_PARAM_FLAG_ENABLE),
```

### Available annotations

- `@Param:` — short name
- `@DisplayName:` — human-readable name
- `@Description:` — detailed description (not too long)
- `@Values:` — `value:label` pairs, comma-separated
- `@Bitmask:` — `bit:label` pairs for bitmask parameters
- `@Range:` — `min max`
- `@Units:` — unit string (`m`, `Hz`, `deg`, `s`, etc.)
- `@Increment:` — UI step size
- `@User:` — `Standard` or `Advanced`
- `@RebootRequired:` — `True` if reboot is needed after change

### Special annotations

- `@Vehicles:` for vehicles-specific parameters

When adding or modifying parameters, always include all relevant annotations.
Parameters fullname max length is 16 characters.

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
| `perf` | Performance improvement |
| `ci` | CI configuration changes |
| `build` | Build system or dependency changes |

### Scope

Optional. Use the affected component: `fins`, `loiter`, `mode`, `params`, `gcs`, `motors`, etc.

### Rules

- Keep the first line under ~72 characters.
- **No merge commits** — always rebase onto the target branch.
- **No `fixup!` commits** — squash them before requesting review.
- One logical change per commit. Split unrelated changes into separate commits.
- No emoji, no jokes.
- Only adjust codestyle and cleanup on what's necessary and keep the file consistent with its current style.

### Examples

```text
feat(fins): add motor mix for quad-fin configuration
fix(loiter): correct drift in manual mode
docs(params): update parameter documentation for LOIT_SPEED
refactor(mode): simplify mode switching logic
```

---

## 8. Pull Request Guidelines

### Before opening a PR

1. **Fork and branch**: Work on a feature branch in your fork.
2. **Rebase on main**: Ensure your branch is up to date.
3. **Build locally**: `./waf configure --board sitl && ./waf blimp`.
4. **Run relevant tests**: At minimum, run SITL for Blimp.
5. **Check formatting**: Ensure code matches the existing style.
6. **Verify commit messages**: Every commit must follow `Subsystem: description`.

### PR description

- Clearly describe **what** the change does and **why**.
- Describe how it was tested.
- **Explicitly state that the contribution was AI-assisted**.
- Keep the description concise.

---

## 9. What AI Should NOT Do

- **Do not fabricate**: Never invent APIs, parameters, MAVLink messages, or hardware interfaces that don't exist in the codebase. Always verify against actual source code.
- **Do not guess at safety-critical logic**: If you are uncertain about control loop behavior, failsafe logic, or sensor fusion, stop and flag it for human review rather than guessing.
- **Do not bypass compile-time guards**: Respect `#if AP_<FEATURE>_ENABLED` guards. Do not remove them to "simplify" code.
- **Do not introduce platform-specific code** in shared libraries. Use the HAL abstraction layer.
- **Do not change parameter indices**: Existing `AP_GROUPINFO` index numbers are baked into user configurations. Changing them breaks parameter storage.
- **Do not add unnecessary dependencies**: ArduPilot runs on constrained embedded hardware. Every byte of RAM and flash matters.
- **Do not generate large speculative refactors**: Focus on minimal, targeted, well-tested changes.
- **Do not remove or weaken existing tests** unless there is a clear, documented reason.
- **Do not auto-generate commit messages**: Write meaningful messages that reflect the actual change.
- **Do not move functions around without goal**: Keep the original code structure as possible.
- **Do not add comments on all functions/lines**: Document only what was changed and useful for future reading.
- **Do not modify upstream libraries or tools**: This repo only contains vehicle code. Library changes belong in the upstream [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) repo.
