# MotorBlimp controller tests

`test_motorblimp_control.cpp` exercises the production `MotorMixer.cpp` and
`FlightControl.cpp` against the ArduPilot 4.7 SITL libraries.  It covers the
canonical motor signs, allocator saturation, invalid input handling, quaternion
attitude error, manual compass-heading hold, NED guidance, pitch limiting,
forward/reverse hysteresis, and waypoint acceptance.

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
