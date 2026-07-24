#include "Blimp.h"

void ModeManual::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    // pilot.x = [-1,1] → throttle [0,1]
    float throttle = (pilot.x + 1.0f) * 0.5f;

    motors->set_roll(pilot.y);
    motors->set_pitch(pilot.z);
    motors->set_yaw(pilot_yaw);
    motors->set_throttle(throttle);
}
