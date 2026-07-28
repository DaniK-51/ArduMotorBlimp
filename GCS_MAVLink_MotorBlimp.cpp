#include "GCS_MAVLink_MotorBlimp.h"
#include "ArduMotorBlimp.h"

uint8_t GCS_MAVLINK_MotorBlimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (motorblimp.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return _base_mode;
}

MAV_STATE GCS_MAVLINK_MotorBlimp::vehicle_system_status() const
{
    if (motorblimp.arming.is_armed()) {
        return MAV_STATE_ACTIVE;
    }
    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_MotorBlimp::send_nav_controller_output() const
{
}

void GCS_MAVLINK_MotorBlimp::send_pid_tuning()
{
}

uint8_t GCS_MAVLINK_MotorBlimp::send_available_mode(uint8_t index) const
{
    return 0;
}
