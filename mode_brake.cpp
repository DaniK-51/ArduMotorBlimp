#include "Blimp.h"

void ModeBrake::run()
{
    motors->set_roll(0);
    motors->set_pitch(0);
    motors->set_yaw(0);
    motors->set_throttle(0);
}

void Blimp::set_mode_brake_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::BRAKE, reason);
    AP_Notify::events.failsafe_mode_change = 1;
}
