#include "Blimp.h"

void ModeBrake::run()
{
    // BRAKE mode: stop all movement and disable motors
    motors->yaw_out = 0;
    motors->pitch_out = 0;
    motors->roll_out = 0;
    motors->x_out = 0;
}

void Blimp::set_mode_brake_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::BRAKE, reason);
    AP_Notify::events.failsafe_mode_change = 1;
}
