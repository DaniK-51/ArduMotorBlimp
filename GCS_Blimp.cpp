#include "GCS_Blimp.h"

#include "Blimp.h"

void GCS_Blimp::init()
{
    GCS::init();

    // Set up mission protocol for waypoint upload/download
    _mission_item_protocol = NEW_NOTHROW MissionItemProtocol_Waypoints(blimp.mission);
    if (_mission_item_protocol != nullptr) {
        missionitemprotocols[0] = _mission_item_protocol;
    }
}

uint8_t GCS_Blimp::sysid_this_mav() const
{
    return blimp.g.sysid_this_mav;
}

const char* GCS_Blimp::frame_string() const
{
    return blimp.get_frame_string();
}

void GCS_Blimp::update_vehicle_sensor_status_flags(void)
{
    // mode-specific flags:
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
}
