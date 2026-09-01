# MotorBlimp controller tests

`test_motorblimp_control.cpp` exercises the production `MotorMixer.cpp` and
`FlightControl.cpp` against the ArduPilot 4.7 SITL libraries.  It covers the
canonical motor signs, allocator saturation, invalid input handling, quaternion
attitude error, compass-independent MANUAL body-yaw-rate control, mode compass
requirements, NED guidance, pitch limiting, forward/reverse hysteresis, and
waypoint acceptance.

From this repository, run:

```sh
python3 tests/run_controller_tests.py
```

The runner defaults to the sibling `work/ardupilot-4.7` checkout and to the
sibling `work/ardupilot-venv`.  Override them when needed:

```sh
ARDUPILOT_DIR=/path/to/ardupilot \
ARDUPILOT_PYTHON=/path/to/venv/bin/python \
python3 tests/run_controller_tests.py --configure
```

The runner creates a temporary test-only mount under `libraries/`, builds the
single gtest binary, runs it, and removes the mount even when the build fails.

The AUTO SITL test uses the normal arming path, waits for the one-tag beacon
and compass EKF, validates the published `HOME_POSITION`, uploads a
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` home plus one waypoint, flies it in
`AUTO=10`, and requires both mission completion and a stable final hold. It
also asserts that the common `SERIAL1_PROTOCOL` parameter is registered:

```sh
python3 tests/sitl_auto.py \
    --binary /path/to/ardupilot/build/sitl/bin/ardumotorblimp \
    --defaults /path/to/ardupilot/Tools/autotest/default_params/motorblimp.parm
```

`sitl_smoke.py --check-guided` additionally proves a real pitch-and-body-X
thrust climb against `SIM_STATE` truth, clears a persistent target on disarm,
and injects loss of the sole UWB backend. The loss gate requires motor neutral
within 0.5 seconds and matching CRITICAL/SYS_STATUS telemetry.

The normal smoke mixer session cold-starts with all simulated magnetometers
returning no data while `COMPASS_USE=1`; EKF yaw is explicitly unaided. It arms
through the ordinary MANUAL pre-arm path, requires `MAV_STATE_ACTIVE`, rejects
`HOLD=4`, `AUTO=10`, and `GUIDED=15` specifically because their heading
control requires a compass, and proves that the MANUAL yaw stick still
produces the expected motor differential and positive body yaw rate. A second
session proves those modes also remain blocked when the magnetometer frontend
is healthy but `EK3_SRC1_YAW=0` leaves the active EKF yaw unaided.
