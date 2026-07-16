#include "Blimp.h"

void ModeLand::run()
{
    motors->yaw_out = 0;
    motors->pitch_out = 0;
    motors->roll_out = 0;
    motors->x_out = 0;
}

void Blimp::set_mode_land_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    AP_Notify::events.failsafe_mode_change = 1;
}
