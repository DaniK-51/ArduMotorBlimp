#pragma once

// Flight modes.  AUTO/GUIDED/HOLD use the conventional ArduPilot Rover
// numbers, which makes MAVLink tooling less surprising for this
// forward-thrust, waypoint-driven vehicle.
enum class Mode : uint8_t {
    MANUAL = 0,
    HOLD = 4,
    AUTO = 10,
    GUIDED = 15,
};

// Keep the conventional ArduPilot log bit assignments so Mission Planner,
// MAVExplorer and scheduler performance logging interpret LOG_BITMASK in the
// usual way.
#define MASK_LOG_ATTITUDE_MED  (1U << 1)
#define MASK_LOG_PM            (1U << 3)
#define MASK_LOG_RCIN          (1U << 6)
#define MASK_LOG_IMU           (1U << 7)
#define MASK_LOG_CURRENT       (1U << 9)
#define MASK_LOG_RCOUT         (1U << 10)
#define MASK_LOG_PID           (1U << 12)
#define MASK_LOG_COMPASS       (1U << 13)
