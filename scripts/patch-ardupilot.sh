#!/bin/bash
# patch-ardupilot.sh — Register ArduMotorBlimp in ArduPilot's build system.
# Run from the ArduPilot root directory:
#   ./ArduMotorBlimp/scripts/patch-ardupilot.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "ArduPilot root: $ARDUPILOT_ROOT"

# --- 1. wscript: add 'motorblimp' to vehicles list ---
WSCRIPT="$ARDUPILOT_ROOT/wscript"
if grep -q "'motorblimp'" "$WSCRIPT"; then
    echo "[SKIP] 'motorblimp' already in wscript vehicles list"
else
    sed -i "s/vehicles = \['antennatracker', 'blimp', 'copter', 'heli', 'plane', 'rover', 'sub'\]/vehicles = ['antennatracker', 'blimp', 'copter', 'heli', 'motorblimp', 'plane', 'rover', 'sub']/" "$WSCRIPT"
    echo "[DONE] Added 'motorblimp' to wscript vehicles list"
fi

# --- 2. AP_Vehicle_Type.h: add APM_BUILD_ArduMotorBlimp ---
VTYPE="$ARDUPILOT_ROOT/libraries/AP_Vehicle/AP_Vehicle_Type.h"
if grep -q "APM_BUILD_ArduMotorBlimp" "$VTYPE"; then
    echo "[SKIP] APM_BUILD_ArduMotorBlimp already defined"
else
    sed -i '/#define APM_BUILD_Heli       13/a #define APM_BUILD_ArduMotorBlimp  14' "$VTYPE"
    echo "[DONE] Added APM_BUILD_ArduMotorBlimp to AP_Vehicle_Type.h"
fi

# --- 3. GCS_MAVLink_Parameters.cpp: add streamrates for ArduMotorBlimp ---
GCS_PARAMS="$ARDUPILOT_ROOT/libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp"
if grep -q "APM_BUILD_ArduMotorBlimp" "$GCS_PARAMS"; then
    echo "[SKIP] ArduMotorBlimp streamrates already defined"
else
    # Add ArduMotorBlimp to the Blimp branch (same streamrates: all zero)
    sed -i 's/APM_BUILD_TYPE(APM_BUILD_Blimp)/APM_BUILD_TYPE(APM_BUILD_Blimp) || APM_BUILD_TYPE(APM_BUILD_ArduMotorBlimp)/' "$GCS_PARAMS"
    echo "[DONE] Added ArduMotorBlimp streamrates to GCS_MAVLink_Parameters.cpp"
fi

echo ""
echo "Patch applied. Build with:"
echo "  ./waf configure --board sitl"
echo "  ./waf motorblimp"
