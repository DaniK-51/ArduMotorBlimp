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

4 static motors with configurable mixing matrix via `AP_MotorsBlimp` (inherits `AP_Motors`).

Protocol selection via `MOTOR_PWM_TYPE` parameter:
| Value | Protocol |
|-------|----------|
| 0 | Normal PWM |
| 1 | OneShot |
| 2 | OneShot125 |
| 3 | Brushed |
| 4 | DShot150 |
| 5 | DShot300 |
| 6 | DShot600 |
| 7 | DShot1200 |

### Arming

Button-based arming via AUX channel switch:
1. Set an RC channel (5-8) to `AUX_FUNC=31` (ARMDISARM)
2. Toggle switch HIGH to arm, LOW to disarm

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
