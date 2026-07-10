#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

// Runs the main velocity controller
void ModeVelocity::run()
{
    Vector3f target_vel;
    float target_vel_yaw;
    get_pilot_input(target_vel, target_vel_yaw);

    // Scale inputs
    target_vel.x *= g.max_vel_x;     // forward/backward velocity
    target_vel.y *= g.max_vel_roll;  // roll rate
    target_vel.z *= g.max_vel_pitch; // pitch rate
    target_vel_yaw *= g.max_vel_yaw;

    // Body-frame rotation for X and yaw
    if (g.simple_mode == 0) {
        blimp.rotate_BF_to_NE(target_vel.xy());
    }

    blimp.loiter->run_vel(target_vel, target_vel_yaw, Vector4b{false,false,false,false});
}
