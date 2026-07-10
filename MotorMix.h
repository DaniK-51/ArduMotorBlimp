// Motor mixing class for 4 static motors
// Converts control axis outputs to motor PWM via a configurable mixing matrix
#pragma once
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

#define NUM_MOTORS 4
#define MOTOR_SCALE_MAX 1000

class MotorMix
{
public:
    friend class Blimp;
    friend class Loiter;

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_MIXED = 1,
    };
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_MIXED = 1,
    };

    //constructor
    MotorMix(uint16_t loop_rate);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    bool initialised_ok() const
    {
        return true;
    }

    void armed(bool arm)
    {
        if (arm != _armed) {
            _armed = arm;
            AP_Notify::flags.armed = arm;
        }
    }
    bool armed() const
    {
        return _armed;
    }

protected:
    const uint16_t      _loop_rate;
    bool _armed;

    // Mixing matrix parameters: motor_yaw[i], motor_pitch[i], motor_roll[i], motor_x[i]
    AP_Float            motor_yaw[NUM_MOTORS];
    AP_Float            motor_pitch[NUM_MOTORS];
    AP_Float            motor_roll[NUM_MOTORS];
    AP_Float            motor_x[NUM_MOTORS];

public:
    float               yaw_out;    // [-1, +1] rotational around Z
    float               pitch_out;  // [-1, +1] rotational around Y
    float               roll_out;   // [-1, +1] rotational around X
    float               x_out;      // [-1, +1] linear along X

    void output_min();

    void setup_motors();

    void output();

    float get_throttle()
    {
        return fmaxf(fmaxf(fabsf(x_out), fabsf(roll_out)), fmaxf(fabsf(pitch_out), fabsf(yaw_out)));
    }
};
