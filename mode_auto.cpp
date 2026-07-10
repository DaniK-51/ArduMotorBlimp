#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

#define WP_RADIUS 1.0f  // meters
#define HEADING_THRESHOLD_DEG 10.0f  // degrees - turn before fly threshold
#define PITCH_THRESHOLD_DEG 10.0f    // degrees - pitch align threshold

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
    const float dt = blimp.scheduler.get_last_loop_time_s();

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

    // Calculate errors
    Vector3f error = target_ned - blimp.pos_ned;
    float dist_xy = sqrtf(error.x * error.x + error.y * error.y);
    float dist_z = error.z;

    // Yaw: bearing to target in NE frame
    float bearing_to_target = atan2f(error.y, error.x);
    float current_yaw = blimp.ahrs.get_yaw();
    float yaw_error = wrap_PI(bearing_to_target - current_yaw);

    // Pitch: angle to target in vertical plane (XZ)
    // positive pitch_error = target is above us = need to pitch up
    float horizontal_dist = sqrtf(error.x * error.x + error.y * error.y);
    float pitch_to_target = atan2f(-dist_z, horizontal_dist);  // negative because NED z is down
    float current_pitch = 0;  // assume level for now; could use AHRS pitch
    float pitch_error = pitch_to_target - current_pitch;

    // Turn-then-fly logic:
    // Phase 1: Align yaw AND pitch simultaneously
    bool yaw_aligned = fabsf(degrees(yaw_error)) < HEADING_THRESHOLD_DEG;
    bool pitch_aligned = fabsf(degrees(pitch_error)) < PITCH_THRESHOLD_DEG;
    bool close_enough = dist_xy < WP_RADIUS * 2.0f;

    if (!close_enough && (!yaw_aligned || !pitch_aligned)) {
        // Phase 1: Turn/pitch to face target (no forward movement)
        float target_yaw_vel = 0;
        float target_pitch_vel = 0;

        if (!yaw_aligned) {
            // Position yaw PID -> target yaw velocity
            target_yaw_vel = blimp.pid_pos_yaw.update_error(yaw_error, dt, false);
            target_yaw_vel = constrain_float(target_yaw_vel, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);
            // Velocity yaw PID -> actuator output
            blimp.motors->yaw_out = blimp.pid_vel_yaw.update_all(target_yaw_vel, blimp.vel_yaw_filtd, dt, false);
        } else {
            blimp.motors->yaw_out = 0;
            blimp.pid_vel_yaw.set_integrator(0);
        }

        if (!pitch_aligned) {
            // Position pitch PID -> target pitch velocity
            target_pitch_vel = blimp.pid_pos_pitch.update_all(pitch_to_target, blimp.pos_ned.z, dt, false);
            target_pitch_vel = constrain_float(target_pitch_vel, -blimp.g.max_vel_pitch, blimp.g.max_vel_pitch);
            // Velocity pitch PID -> actuator output
            blimp.motors->pitch_out = blimp.pid_vel_pitch.update_all(target_pitch_vel, blimp.vel_ned_filtd.z, dt, false);
        } else {
            blimp.motors->pitch_out = 0;
            blimp.pid_vel_pitch.set_integrator(0);
        }

        // No forward or lateral movement during alignment
        blimp.motors->x_out = 0;
        blimp.motors->roll_out = 0;
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
