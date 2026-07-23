#include "Blimp.h"

#include <SRV_Channel/SRV_Channel.h>

/*
  Motor mixing matrix parameters
 */
const AP_Param::GroupInfo MotorMix::var_info[] = {

    // @Param: M1_YAW
    // @DisplayName: Motor 1 yaw contribution
    // @Description: How much motor 1 contributes to yaw rotation. Positive = clockwise, negative = counter-clockwise.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_YAW", 1, MotorMix, motor_yaw[0], -1),

    // @Param: M1_PITCH
    // @DisplayName: Motor 1 pitch contribution
    // @Description: How much motor 1 contributes to pitch rotation. Positive = nose down, negative = nose up.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_PITCH", 2, MotorMix, motor_pitch[0], -1),

    // @Param: M1_ROLL
    // @DisplayName: Motor 1 roll contribution
    // @Description: How much motor 1 contributes to roll rotation. Positive = right roll, negative = left roll.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_ROLL", 3, MotorMix, motor_roll[0], 1),

    // @Param: M1_X
    // @DisplayName: Motor 1 X contribution
    // @Description: How much motor 1 contributes to forward/backward movement. Positive = forward, negative = backward.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_X", 4, MotorMix, motor_x[0], 1),

    // @Param: M2_YAW
    // @DisplayName: Motor 2 yaw contribution
    // @Description: How much motor 2 contributes to yaw rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M2_YAW", 5, MotorMix, motor_yaw[1], 1),

    // @Param: M2_PITCH
    // @DisplayName: Motor 2 pitch contribution
    // @Description: How much motor 2 contributes to pitch rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M2_PITCH", 6, MotorMix, motor_pitch[1], 1),

    // @Param: M2_ROLL
    // @DisplayName: Motor 2 roll contribution
    // @Description: How much motor 2 contributes to roll rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M2_ROLL", 7, MotorMix, motor_roll[1], 1),

    // @Param: M2_X
    // @DisplayName: Motor 2 X contribution
    // @Description: How much motor 2 contributes to forward/backward movement.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M2_X", 8, MotorMix, motor_x[1], 1),

    // @Param: M3_YAW
    // @DisplayName: Motor 3 yaw contribution
    // @Description: How much motor 3 contributes to yaw rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M3_YAW", 9, MotorMix, motor_yaw[2], 1),

    // @Param: M3_PITCH
    // @DisplayName: Motor 3 pitch contribution
    // @Description: How much motor 3 contributes to pitch rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M3_PITCH", 10, MotorMix, motor_pitch[2], -1),

    // @Param: M3_ROLL
    // @DisplayName: Motor 3 roll contribution
    // @Description: How much motor 3 contributes to roll rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M3_ROLL", 11, MotorMix, motor_roll[2], -1),

    // @Param: M3_X
    // @DisplayName: Motor 3 X contribution
    // @Description: How much motor 3 contributes to forward/backward movement.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M3_X", 12, MotorMix, motor_x[2], 1),

    // @Param: M4_YAW
    // @DisplayName: Motor 4 yaw contribution
    // @Description: How much motor 4 contributes to yaw rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M4_YAW", 13, MotorMix, motor_yaw[3], -1),

    // @Param: M4_PITCH
    // @DisplayName: Motor 4 pitch contribution
    // @Description: How much motor 4 contributes to pitch rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M4_PITCH", 14, MotorMix, motor_pitch[3], 1),

    // @Param: M4_ROLL
    // @DisplayName: Motor 4 roll contribution
    // @Description: How much motor 4 contributes to roll rotation.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M4_ROLL", 15, MotorMix, motor_roll[3], -1),

    // @Param: M4_X
    // @DisplayName: Motor 4 X contribution
    // @Description: How much motor 4 contributes to forward/backward movement.
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M4_X", 16, MotorMix, motor_x[3], 1),

    AP_GROUPEND
};

//constructor
MotorMix::MotorMix(uint16_t loop_rate) :
    _loop_rate(loop_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void MotorMix::setup_motors()
{
    // Configure motor channels for scaled output
    // Scale 1000 means: -1000=reverse, 0=stop, +1000=forward
    // SRV_Channel converts this to the actual protocol (PWM, DSHOT, etc.)
    for (int8_t i = 0; i < NUM_MOTORS; i++) {
        SRV_Channels::set_angle(SRV_Channel::get_motor_function(i), MOTOR_SCALE_MAX);
    }
}

void MotorMix::output()
{
    if (!_armed) {
        yaw_out = 0;
        pitch_out = 0;
        roll_out = 0;
        x_out = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_MOTORI(yaw_out, pitch_out, roll_out, x_out);
#endif

    // Constrain after logging so as to still show when sub-optimal tuning is causing massive overshoots.
    yaw_out = constrain_float(yaw_out, -1, 1);
    pitch_out = constrain_float(pitch_out, -1, 1);
    roll_out = constrain_float(roll_out, -1, 1);
    x_out = constrain_float(x_out, -1, 1);

    // Apply mixing matrix: motor_output[m] = sum(axis_weight[m][a] * axis_out[a])
    float motor_outputs[NUM_MOTORS];
    for (int8_t i = 0; i < NUM_MOTORS; i++) {
        motor_outputs[i] = motor_yaw[i] * yaw_out
                         + motor_pitch[i] * pitch_out
                         + motor_roll[i] * roll_out
                         + motor_x[i] * x_out;
        motor_outputs[i] = constrain_float(motor_outputs[i], -1, 1);
    }

    // Output to servos via set_output_scaled (supports PWM, DSHOT, etc.)
    for (int8_t i = 0; i < NUM_MOTORS; i++) {
        SRV_Channels::set_output_scaled(SRV_Channel::get_motor_function(i), motor_outputs[i] * MOTOR_SCALE_MAX);
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_MOTORO(motor_outputs);
#endif
}

void MotorMix::output_min()
{
    yaw_out = 0;
    pitch_out = 0;
    roll_out = 0;
    x_out = 0;
    MotorMix::output();
}
