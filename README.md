# ArduMotorBlimp Vehicle

Custom blimp (Lighter-Than-Air) vehicle implementation for ArduPilot.

## Current Status

**Branch: `feat/manual-only`** — Minimal Manual + BRAKE mode build for first flight testing.

### Available Modes

| Mode | Description |
|------|-------------|
| **MANUAL** (1) | Direct RC passthrough to MotorMix mixing matrix |
| **BRAKE** (0) | Emergency stop — zeros all motors, used for failsafe |

### Motor Control

4 static motors with configurable mixing matrix (16 parameters: `M1_YAW`..`M4_X`).

```
MotorMix output = [yaw_out, pitch_out, roll_out, x_out] × mixing_matrix → [M1, M2, M3, M4]
```

## Build

```bash
# Requires full ArduPilot repo structure
./waf configure --board sitl
./waf blimp
```

## License

GNU General Public License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)
- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)
