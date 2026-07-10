#include "Blimp.h"
/*
 * Init and run calls for manual flight mode
 */

// Runs the main manual controller
void ModeManual::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    // Map RC channels to new axes:
    // Roll stick -> roll_out
    // Pitch stick -> pitch_out
    // Throttle stick -> x_out (forward/backward)
    // Yaw stick -> yaw_out
    motors->roll_out = pilot.y;
    motors->pitch_out = pilot.z;
    motors->x_out = pilot.x;
    motors->yaw_out = pilot_yaw;
}
