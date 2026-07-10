#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

#define WP_RADIUS 1.0f  // meters

bool ModeAuto::init(bool ignore_checks)
{
    return true;
}

// Runs the main auto controller
void ModeAuto::run()
{
    if (!has_target()) {
        // No waypoint - hover in place
        motors->x_out = 0;
        motors->roll_out = 0;
        motors->pitch_out = 0;
        motors->yaw_out = 0;
        return;
    }

    // Get current waypoint
    const MissionItem& wp = waypoints[current_wp];

    // Convert target Location to NED position
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        // Can't get EKF origin, stop
        motors->x_out = 0;
        motors->roll_out = 0;
        motors->pitch_out = 0;
        motors->yaw_out = 0;
        return;
    }

    // Calculate NED position from EKF origin to waypoint
    Vector3f target_ned;
    target_ned.x = (wp.loc.lat - ekf_origin.lat) * 1.113195e-7f * 100.0f;  // meters
    target_ned.y = (wp.loc.lng - ekf_origin.lng) * 1.113195e-7f * cosf(radians(ekf_origin.lat)) * 100.0f;
    target_ned.z = (wp.loc.alt - ekf_origin.alt) * 0.01f;  // cm to meters

    float target_yaw = wp.yaw;

    // Use loiter controller for position hold
    blimp.loiter->run(target_ned, target_yaw, Vector4b{false,false,false,false});

    // Check if reached target
    Vector3f error = target_ned - blimp.pos_ned;
    float dist = error.length();
    if (dist < WP_RADIUS) {
        // Waypoint reached, advance to next
        advance_to_next();
    }
}

void ModeAuto::set_target(const Location& loc, float yaw)
{
    if (num_waypoints < MAX_WAYPOINTS) {
        waypoints[num_waypoints].loc = loc;
        waypoints[num_waypoints].yaw = yaw;
        waypoints[num_waypoints].valid = true;
        num_waypoints++;
    }
}

bool ModeAuto::has_target() const
{
    return (current_wp < num_waypoints && waypoints[current_wp].valid);
}

Location ModeAuto::get_target() const
{
    if (has_target()) {
        return waypoints[current_wp].loc;
    }
    return Location {};
}

void ModeAuto::advance_to_next()
{
    if (current_wp < num_waypoints) {
        waypoints[current_wp].valid = false;
        current_wp++;
    }
}

bool ModeAuto::mission_complete() const
{
    return (current_wp >= num_waypoints);
}

void ModeAuto::clear_mission()
{
    for (uint8_t i = 0; i < MAX_WAYPOINTS; i++) {
        waypoints[i].valid = false;
    }
    current_wp = 0;
    num_waypoints = 0;
}
