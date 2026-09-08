#pragma once

// MotorBlimp relies on the onboard Nooploop parser even on 2 MiB targets.
// The matching ArduPilot integration patch makes AP_Beacon vehicle-dependent
// so its library objects are built with the same feature selection.
#ifndef AP_BEACON_ENABLED
#define AP_BEACON_ENABLED 1
#endif

#ifndef AP_BEACON_NOOPLOOP_ENABLED
#define AP_BEACON_NOOPLOOP_ENABLED 1
#endif

#ifndef AP_MISSION_ENABLED
#define AP_MISSION_ENABLED 1
#endif

#ifndef DEFAULT_LOG_BITMASK
#define DEFAULT_LOG_BITMASK (MASK_LOG_ATTITUDE_MED | MASK_LOG_PM | \
                             MASK_LOG_RCIN | MASK_LOG_IMU | \
                             MASK_LOG_CURRENT | MASK_LOG_RCOUT | \
                             MASK_LOG_PID | MASK_LOG_COMPASS)
#endif
