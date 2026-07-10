#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

#define WP_RADIUS 1.0f  // meters
#define HEADING_THRESHOLD_DEG 10.0f  // degrees - turn before fly threshold

bool ModeAuto::init(bool ignore_checks)
{
    // Start or resume mission
    if (blimp.mission.present() || ignore_checks) {
        blimp.mission.start_or_resume();
        return true;
    }
    return false;
}

// Runs the main auto controller
void ModeAuto::run()
{
    // Update mission
    blimp.mission.update();

    if (!has_target()) {
        // No waypoint target - hover in place
        blimp.motors->x_out = 0;
        blimp.motors->roll_out = 0;
        blimp.motors->pitch_out = 0;
        blimp.motors->yaw_out = 0;
        return;
    }

    // Get EKF origin
    Location ekf_origin;
    if (!blimp.ahrs.get_origin(ekf_origin)) {
        blimp.motors->x_out = 0;
        blimp.motors->roll_out = 0;
        blimp.motors->pitch_out = 0;
        blimp.motors->yaw_out = 0;
        return;
    }

    // Calculate NED position from EKF origin to waypoint
    Vector3f target_ned;
    target_ned.x = (_target_loc.lat - ekf_origin.lat) * 1.113195e-7f * 100.0f;
    target_ned.y = (_target_loc.lng - ekf_origin.lng) * 1.113195e-7f * cosf(radians(ekf_origin.lat)) * 100.0f;
    target_ned.z = (_target_loc.alt - ekf_origin.alt) * 0.01f;

    // Calculate bearing to target
    Vector3f error = target_ned - blimp.pos_ned;
    float dist_xy = sqrtf(error.x * error.x + error.y * error.y);
    float bearing_to_target = atan2f(error.y, error.x);
    float current_yaw = blimp.ahrs.get_yaw();
    float yaw_error = wrap_PI(bearing_to_target - current_yaw);

    // Turn-then-fly logic:
    // If we're not facing the target and we're not very close to it,
    // rotate first before flying forward
    if (dist_xy > WP_RADIUS * 2.0f && fabsf(degrees(yaw_error)) > HEADING_THRESHOLD_DEG) {
        // Phase 1: Turn to face target
        // Use yaw PID to turn toward the waypoint
        float target_yaw_vel = 0;
        blimp.pid_vel_yaw.set_target_rate(0);
        blimp.pid_vel_yaw.set_actual_rate(0);
        target_yaw_vel = blimp.pid_vel_yaw.update_error(yaw_error, blimp.G_Dt, false);
        target_yaw_vel = constrain_float(target_yaw_vel, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

        // Only yaw, no forward movement
        blimp.motors->x_out = 0;
        blimp.motors->roll_out = 0;
        blimp.motors->pitch_out = 0;
        blimp.motors->yaw_out = target_yaw_vel;
    } else {
        // Phase 2: Fly to target (use loiter controller)
        blimp.loiter->run(target_ned, _target_yaw, Vector4b{false,false,false,false});
    }
}

void ModeAuto::set_wp_target(const Location& loc)
{
    _target_loc = loc;
    _has_target = true;
}

bool ModeAuto::has_target() const
{
    return _has_target;
}

void ModeAuto::clear_target()
{
    _has_target = false;
}
