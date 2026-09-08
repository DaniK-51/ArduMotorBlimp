#include "ArduMotorBlimp.h"
#include "GCS_MotorBlimp.h"

void GCS_MotorBlimp::update_vehicle_sensor_status_flags()
{
    static constexpr uint32_t attitude_control_flags =
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    static constexpr uint32_t yaw_position_flag =
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;
    static constexpr uint32_t position_control_flags =
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;

    control_sensors_present |= attitude_control_flags |
                               yaw_position_flag |
                               position_control_flags;

    const Mode mode = (Mode)motorblimp.get_mode();
    const bool position_control_enabled = mode == Mode::AUTO ||
                                          mode == Mode::GUIDED;

    const AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();
    const bool rate_control_ok = ins.get_gyro_health_all() &&
                                 ins.gyro_calibrated_ok_all();
    const bool attitude_ok = ahrs.initialised() && ahrs.healthy() &&
                             ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID);
    const bool yaw_ok = attitude_ok && motorblimp.compass_healthy();
    const bool navigation_ok = motorblimp.navigation_healthy();

    // MANUAL is stabilised as well: sticks command roll/pitch attitude and
    // yaw rate, while FlightControl closes the attitude and body-rate loops.
    control_sensors_enabled |= attitude_control_flags;
    if (mode_requires_compass(mode)) {
        control_sensors_enabled |= yaw_position_flag;
    }
    if (rate_control_ok) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
    }
    if (attitude_ok) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    }
    if (mode_requires_compass(mode) && yaw_ok) {
        control_sensors_health |= yaw_position_flag;
    }

    if (position_control_enabled) {
        control_sensors_enabled |= position_control_flags;
        if (navigation_ok) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        }
    }
}

uint32_t GCS_MotorBlimp::custom_mode() const
{
    return motorblimp.get_mode();
}

MAV_TYPE GCS_MotorBlimp::frame_type() const
{
    return MAV_TYPE_AIRSHIP;
}
