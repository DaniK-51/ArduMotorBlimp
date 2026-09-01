#!/bin/sh
# Apply the ArduMotorBlimp SITL integration to the exact ArduPilot revision it
# was generated against. This script intentionally uses git apply instead of
# platform-specific sed syntax.

set -eu

EXPECTED_SHA="331c42a50c1f68b0065d4944e55eb688b62fe9c4"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MOTORBLIMP_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
PATCH_FILE="$MOTORBLIMP_ROOT/patches/ardupilot-331c42a.patch"

usage()
{
    echo "Usage: $0 [ARDUPILOT_ROOT]" >&2
    echo "If omitted, run from an ArduPilot root or keep ArduMotorBlimp inside it." >&2
    exit 2
}

if [ "$#" -gt 1 ]; then
    usage
fi

if [ "$#" -eq 1 ]; then
    ARDUPILOT_ROOT=$1
elif [ -f "./wscript" ] && [ -d "./libraries/AP_HAL_SITL" ]; then
    ARDUPILOT_ROOT=.
else
    ARDUPILOT_ROOT="$MOTORBLIMP_ROOT/.."
fi

if [ ! -d "$ARDUPILOT_ROOT" ]; then
    echo "ArduPilot root does not exist: $ARDUPILOT_ROOT" >&2
    usage
fi

ARDUPILOT_ROOT=$(CDPATH= cd -- "$ARDUPILOT_ROOT" && pwd)

if [ ! -f "$ARDUPILOT_ROOT/wscript" ] || [ ! -d "$ARDUPILOT_ROOT/libraries/AP_HAL_SITL" ]; then
    echo "Not an ArduPilot checkout: $ARDUPILOT_ROOT" >&2
    exit 1
fi

if [ ! -f "$PATCH_FILE" ]; then
    echo "Missing pinned patch: $PATCH_FILE" >&2
    exit 1
fi

ACTUAL_SHA=$(git -C "$ARDUPILOT_ROOT" rev-parse HEAD)
if [ "$ACTUAL_SHA" != "$EXPECTED_SHA" ]; then
    echo "Unsupported ArduPilot revision." >&2
    echo "Expected: $EXPECTED_SHA" >&2
    echo "Actual:   $ACTUAL_SHA" >&2
    exit 1
fi

echo "ArduPilot root: $ARDUPILOT_ROOT"
echo "ArduPilot SHA:  $ACTUAL_SHA"

if git -C "$ARDUPILOT_ROOT" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1; then
    echo "SITL patch is already applied."
else
    if ! git -C "$ARDUPILOT_ROOT" apply --check "$PATCH_FILE"; then
        echo "Patch cannot be applied cleanly. The checkout may be partially patched" >&2
        echo "or contain overlapping local changes." >&2
        exit 1
    fi
    git -C "$ARDUPILOT_ROOT" apply "$PATCH_FILE"
    echo "Applied $(basename "$PATCH_FILE")."
fi

# Detect accidental drift between the canonical source files and the copies
# embedded in the generated ArduPilot patch.
if ! cmp -s "$MOTORBLIMP_ROOT/sitl/SIM_MotorBlimp.h" \
          "$ARDUPILOT_ROOT/libraries/SITL/SIM_MotorBlimp.h"; then
    echo "Patched SIM_MotorBlimp.h differs from sitl/SIM_MotorBlimp.h" >&2
    exit 1
fi
if ! cmp -s "$MOTORBLIMP_ROOT/sitl/SIM_MotorBlimp.cpp" \
          "$ARDUPILOT_ROOT/libraries/SITL/SIM_MotorBlimp.cpp"; then
    echo "Patched SIM_MotorBlimp.cpp differs from sitl/SIM_MotorBlimp.cpp" >&2
    exit 1
fi
if ! cmp -s "$MOTORBLIMP_ROOT/sitl/motorblimp.parm" \
          "$ARDUPILOT_ROOT/Tools/autotest/default_params/motorblimp.parm"; then
    echo "Patched motorblimp.parm differs from sitl/motorblimp.parm" >&2
    exit 1
fi

echo "SITL overlay verified. Build with:"
echo "  ./waf configure --board sitl"
echo "  ./waf build --target bin/ardumotorblimp"
echo "Run with:"
echo "  ./Tools/autotest/sim_vehicle.py -v ArduMotorBlimp -f motorblimp -w --console"
