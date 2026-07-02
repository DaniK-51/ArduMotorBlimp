#include "Blimp.h"

#include <SRV_Channel/SRV_Channel.h>

const AP_Param::GroupInfo Fins::var_info[] = {

    // @Param: FREQ_HZ
    // @DisplayName: Fins frequency
    // @Description: This is the oscillation frequency of the fins
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("FREQ_HZ", 1, Fins, freq_hz, 3),

    // @Param: TURBO_MODE
    // @DisplayName: Enable turbo mode
    // @Description: Enables double speed on high offset (finned blimp only).
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TURBO_MODE", 2, Fins, turbo_mode, 0),

    // @Param: THR_MAX
    // @DisplayName: Maximum throttle
    // @Description: Maximum throttle allowed. Constrains any throttle input to this value (negative and positive) Set it to 1 to disable (i.e. allow max throttle).
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 3, Fins, thr_max, 1),

    AP_GROUPEND
};

//constructor
Fins::Fins(uint16_t loop_rate) :
    _loop_rate(loop_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
    switch ((Fins::motor_frame)blimp.g2.frame_class.get()) {
        case Fins::MOTOR_FRAME_ROTARY_BLIMP:
            _frame = Fins::MOTOR_FRAME_ROTARY_BLIMP;
            _initialised_ok = true;
            break;
        case Fins::MOTOR_FRAME_UNDEFINED:
        default:
            _frame = Fins::MOTOR_FRAME_UNDEFINED;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Bad frame class. Motors not initialised.");
            _initialised_ok = false;
            break;
    }   
}

void Fins::setup_finsmotors()
{
    if (!_initialised_ok) {
        return;
    }

    switch (_frame) {
        case Fins::MOTOR_FRAME_ROTARY_BLIMP:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting up RotaryBlimp.");
            setup_rotary();
            break;
        case Fins::MOTOR_FRAME_UNDEFINED:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Bad frame class.");
            break;
    }
}

void Fins::setup_rotary()
{
    //   motor#   forward pitch roll  yaw
    add_motor(0,  1,  1,  0,   1);  //FrontLeft  — forward + pitch + yaw
    add_motor(1,  1,  1,  0,  -1);  //FrontRight — forward + pitch - yaw
    add_motor(2,  0, -1,  0,   0);  //Up         — pitch only
    add_motor(3,  0,  0,  1,   0);  //Right      — roll only

    SRV_Channels::set_angle(SRV_Channel::k_motor1, INPUT_AND_OUTPUT_SCALING);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, INPUT_AND_OUTPUT_SCALING);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, INPUT_AND_OUTPUT_SCALING);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, INPUT_AND_OUTPUT_SCALING);
}

void Fins::add_motor(int8_t fin_num, float forward_amp_fac, float pitch_amp_fac, float roll_amp_fac, float yaw_amp_fac)
{
    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {
        _forward_amp_factor[fin_num] = forward_amp_fac;
        _pitch_amp_factor[fin_num] = pitch_amp_fac;
        _roll_amp_factor[fin_num] = roll_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;
    }
}

void Fins::output()
{
    if (!_initialised_ok) {
        return;
    }

    if (!_armed) {
        // set everything to zero so fins stop moving
        forward_out = 0;
        pitch_out   = 0;
        roll_out    = 0;
        yaw_out     = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINI(forward_out, pitch_out, roll_out, yaw_out);
#endif

    //Constrain after logging so as to still show when sub-optimal tuning is causing massive overshoots.
    forward_out = constrain_float(forward_out, -thr_max, thr_max);
    pitch_out = constrain_float(pitch_out, -thr_max, thr_max);
    roll_out = constrain_float(roll_out, -thr_max, thr_max);
    yaw_out = constrain_float(yaw_out, -thr_max, thr_max);

    switch (_frame) {
        case Fins::MOTOR_FRAME_ROTARY_BLIMP:
            output_rotary();
            break;
        case Fins::MOTOR_FRAME_UNDEFINED:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Bad frame class.");
            break;
    }
}

void Fins::output_rotary()
{
    for (int8_t i=0; i<NUM_FINS; i++) {
        //Calculate throttle for each motor
        _thrpos[i] = constrain_float(_forward_amp_factor[i]*forward_out + _pitch_amp_factor[i]*pitch_out + _roll_amp_factor[i]*roll_out + _yaw_amp_factor[i]*yaw_out, -thr_max, thr_max);

        //Set output
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrpos[i] * INPUT_AND_OUTPUT_SCALING);
    }
}

void Fins::output_min()
{
    forward_out = 0;
    pitch_out   = 0;
    roll_out    = 0;
    yaw_out     = 0;
    Fins::output();
}

const char* Fins::get_frame_string()
{
    switch (_frame) {
        case Fins::MOTOR_FRAME_ROTARY_BLIMP:
            return "ROTARYBLIMP";
        case Fins::MOTOR_FRAME_UNDEFINED:
            break;
    }
    return "NOFRAME";
}
