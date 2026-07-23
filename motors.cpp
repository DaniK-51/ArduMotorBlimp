#include "Blimp.h"

// arm_motors_check - currently empty, arming is handled via RC AUX channel ARMDISARM function
// To arm: set an RC channel (5-8) to AUX_FUNC=31 (ARMDISARM)
// Toggle switch to arm/disarm
void Blimp::arm_motors_check()
{
    // arming is handled by RC_Channel AUX function ARMDISARM
    // no additional logic needed here
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Blimp::motors_output()
{
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();
    srv.cork();

    SRV_Channels::output_ch_all();

    motors->output();

    srv.push();
}
