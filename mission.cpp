#include "Blimp.h"

// Start command callback for AP_Mission
bool Blimp::start_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
        do_nav_wp(cmd);
        break;
    case MAV_CMD_NAV_LAND:
        do_land(cmd);
        break;
    case MAV_CMD_NAV_TAKEOFF:
        do_takeoff(cmd);
        break;
    default:
        return false;
    }
    return true;
}

// Verify command callback for AP_Mission
bool Blimp::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
    case MAV_CMD_NAV_LAND:
        return false; // land doesn't verify
    case MAV_CMD_NAV_TAKEOFF:
        return false; // takeoff doesn't verify
    default:
        return true;
    }
}

// Mission complete callback
void Blimp::mission_complete()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Mission complete");
    AP_Notify::events.mission_complete = 1;
}

// NAV_WAYPOINT command
void Blimp::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // Convert mission command to Location
    Location dest_loc;
    if (!cmd.content.location.initialised()) {
        return;
    }
    dest_loc = cmd.content.location;

    // Set target in mode_auto
    if (control_mode == Mode::Number::AUTO) {
        mode_auto.set_wp_target(dest_loc);
    }
}

// Verify NAV_WAYPOINT reached
bool Blimp::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // Check if we've reached the waypoint
    Location target_loc;
    if (cmd.content.location.initialised()) {
        target_loc = cmd.content.location;
    } else {
        return false;
    }

    // Get EKF origin
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // Calculate NED position to target
    Vector3f target_ned;
    target_ned.x = (target_loc.lat - ekf_origin.lat) * 1.113195e-7f * 100.0f;
    target_ned.y = (target_loc.lng - ekf_origin.lng) * 1.113195e-7f * cosf(radians(ekf_origin.lat)) * 100.0f;
    target_ned.z = (target_loc.alt - ekf_origin.alt) * 0.01f;

    // Calculate distance
    Vector3f error = target_ned - pos_ned;
    float dist = error.length();

    // Check acceptance radius
    float accept_radius = cmd.p1 > 0 ? cmd.p1 : 1.0f;
    if (dist < accept_radius) {
        return true;
    }

    return false;
}

// NAV_LAND command
void Blimp::do_land(const AP_Mission::Mission_Command& cmd)
{
    // Switch to land mode
    set_mode(Mode::Number::LAND, ModeReason::MISSION_COMMAND);
}

// NAV_TAKEOFF command
void Blimp::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // For blimp, takeoff just means arm and start mission
    // The actual altitude target is handled by the mission
}
