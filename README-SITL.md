# ArduMotorBlimp native SITL

The SITL overlay is pinned to ArduPilot
`331c42a50c1f68b0065d4944e55eb688b62fe9c4` (`ArduPilot-4.7`). It adds a
native six-degree-of-freedom model for four reversible longitudinal motors.

## Apply and build

Clone this repository as `ArduMotorBlimp` inside the pinned ArduPilot checkout,
then run from the ArduPilot root:

```sh
./ArduMotorBlimp/scripts/patch-ardupilot.sh
./waf configure --board sitl
./waf build --target bin/ardumotorblimp
```

The patch script is safe to run repeatedly. It rejects another ArduPilot SHA
instead of attempting a fuzzy source rewrite.

## Run

The registered `sim_vehicle.py` entry is the normal entry point:

```sh
./Tools/autotest/sim_vehicle.py \
    -v ArduMotorBlimp -f motorblimp -w --console --map
```

The underlying binary can also be launched directly:

```sh
./build/sitl/bin/ardumotorblimp \
    --model motorblimp \
    --wipe \
    --defaults Tools/autotest/default_params/motorblimp.parm \
    --home 55.751244,37.618423,200,0
```

Connect MAVProxy from another terminal with:

```sh
mavproxy.py --master=tcp:127.0.0.1:5760 --console --map
```

## Smoke test

```sh
python3 ArduMotorBlimp/tests/sitl_smoke.py \
    --binary build/sitl/bin/ardumotorblimp \
    --defaults Tools/autotest/default_params/motorblimp.parm
```

The default gate uses normal arming (no force-arm bypass) and verifies the
airship heartbeat, neutral/reverse PWM convention, four-axis mixer signs,
forward motion, and positive roll/pitch/yaw response using `SIM_STATE` truth.

Run the automatic-navigation gate with:

```sh
python3 ArduMotorBlimp/tests/sitl_smoke.py \
    --binary build/sitl/bin/ardumotorblimp \
    --defaults Tools/autotest/default_params/motorblimp.parm \
    --check-guided
```

This waits for a healthy EKF using direct XYZ position from the simulated UWB
tag and yaw from the internal compass; GPS remains disabled. The direct tag-Z
measurement avoids the ambiguous height solution produced by range-only
fusion with coplanar anchors. The gate then verifies the `GUIDED=15` mode, a
horizontal target, and a cold-start five-metre-forward/one-metre-up climb. The
3D assertion requires nose-up pitch and positive longitudinal collective before
the first measured climb, then requires estimated NED-z and `SIM_STATE` truth
altitude to agree. It also disables the only beacon backend at runtime and
requires neutral within 0.5 seconds plus CRITICAL/unhealthy position telemetry.

The independent mission gate verifies that the common `SERIAL1_PROTOCOL`
parameter is registered, disarms long enough for the UWB/global origin to
establish and publish `HOME_POSITION`, then normally re-arms. It uploads a
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` home item and waypoint, selects `AUTO=10`,
and requires both `MISSION_ITEM_REACHED` and a stable final hold:

```sh
python3 ArduMotorBlimp/tests/sitl_auto.py \
    --binary build/sitl/bin/ardumotorblimp \
    --defaults Tools/autotest/default_params/motorblimp.parm
```

## Onboard logs

The normal ArduPilot logger lifecycle and `LOG_BITMASK` parameter are enabled.
The default mask records attitude plus EKF/AHR2/POS state, RC input and motor
output, roll/pitch/yaw PID terms, IMU/vibration, battery, scheduler performance,
and compass data. These records are intended to make a manual or autonomous
flight diagnosable without a live telemetry connection.

## Hardware output and parameter migration

The firmware default is ordinary PWM with 1500 microseconds as reversible-ESC
neutral. Verify neutral and both directions on a restrained bench setup before
installing propellers.

At runtime, autonomous control is stopped at neutral if the compass or fresh
direct UWB fix becomes unhealthy. `UWB_ERR_MAX` defaults to 1.0 metre; the
Nooploop backend requires at least four valid anchor blocks and its 300 ms
timeout is not refreshed by a stale coordinate. An independent 1 kHz main-loop
watchdog also commands explicit neutral and disarms after a 200 ms control-loop
stall; it does not depend on the stalled flight-control task being able to
clean itself up.

DShot is safe only when bidirectional/3D output is explicitly enabled for all
four motors. Set `SERVO_BLH_MASK=15`, `SERVO_BLH_3DMASK=15`, and
`SERVO_BLH_OTYPE=5` for DShot300 or `SERVO_BLH_OTYPE=6` for DShot600, then
repeat the restrained neutral/direction test. Without `SERVO_BLH_3DMASK=15`,
the 1500-centred command convention is not a stopped motor.

The completed firmware uses `FORMAT_VERSION=2` because the vehicle parameter
key layout changed. Back up a version-1 parameter set, wipe/reinitialise storage
when upgrading, and reconfigure/calibrate it against the version-2 parameter
names instead of reusing the old storage image blindly.

## Maintaining the pinned patch

The canonical new files are under `ArduMotorBlimp/sitl/`. To regenerate the
patch, start from a clean checkout of the pinned SHA, copy those files to:

```text
libraries/SITL/SIM_MotorBlimp.h
libraries/SITL/SIM_MotorBlimp.cpp
Tools/autotest/default_params/motorblimp.parm
```

Apply the integration edits described by the existing patch and produce the
artifact from the ArduPilot root with:

```sh
git add -N \
    libraries/SITL/SIM_MotorBlimp.h \
    libraries/SITL/SIM_MotorBlimp.cpp \
    Tools/autotest/default_params/motorblimp.parm
git diff --binary --no-ext-diff \
    --output=ArduMotorBlimp/patches/ardupilot-331c42a.patch
```

Running `patch-ardupilot.sh` performs byte-for-byte comparisons between the
canonical files and the copies embedded in the patch, catching drift.
