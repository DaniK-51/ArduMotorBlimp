#include "ArduMotorBlimp.h"
#include "GCS_MotorBlimp.h"

void GCS_MotorBlimp::update_vehicle_sensor_status_flags()
{
}

uint32_t GCS_MotorBlimp::custom_mode() const
{
    return (uint32_t)Mode::MANUAL;
}

MAV_TYPE GCS_MotorBlimp::frame_type() const
{
    return MAV_TYPE_AIRSHIP;
}
