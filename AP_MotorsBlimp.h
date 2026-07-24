#pragma once

#include <AP_Motors/AP_Motors_Class.h>

#define AP_MOTORS_BLIMP_NUM_MOTORS 4

class AP_MotorsBlimp : public AP_Motors
{
public:
    AP_MotorsBlimp(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    static AP_MotorsBlimp *get_singleton() { return _singleton; }

    // AP_Motors pure virtual implementations
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void output() override;
    void output_min() override;
    void set_desired_spool_state(DesiredSpoolState spool) override;
    float get_throttle_hover() const override { return 0.0f; }
    uint32_t get_motor_mask() override;
    void set_frame_class_and_type(motor_frame_class, motor_frame_type) override {}

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    void output_armed_stabilizing() override;
    void update_throttle_filter() override;
    const char* _get_frame_string() const override { return "BLIMP"; }
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // Mixing matrix factors (per motor)
    AP_Float _roll_factor[AP_MOTORS_BLIMP_NUM_MOTORS];
    AP_Float _pitch_factor[AP_MOTORS_BLIMP_NUM_MOTORS];
    AP_Float _yaw_factor[AP_MOTORS_BLIMP_NUM_MOTORS];
    AP_Float _throttle_factor[AP_MOTORS_BLIMP_NUM_MOTORS];  // X axis

private:
    static AP_MotorsBlimp *_singleton;
};
