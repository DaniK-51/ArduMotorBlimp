#include "AP_MotorsBlimp.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

AP_MotorsBlimp *AP_MotorsBlimp::_singleton;

const AP_Param::GroupInfo AP_MotorsBlimp::var_info[] = {
    // Motor 1 factors
    // @Param: M1_ROLL
    // @DisplayName: Motor 1 roll factor
    // @Description: How much motor 1 contributes to roll
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_ROLL", 1, AP_MotorsBlimp, _roll_factor[0], 0),
    // @Param: M1_PITCH
    // @DisplayName: Motor 1 pitch factor
    // @Description: How much motor 1 contributes to pitch
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_PITCH", 2, AP_MotorsBlimp, _pitch_factor[0], 0),
    // @Param: M1_YAW
    // @DisplayName: Motor 1 yaw factor
    // @Description: How much motor 1 contributes to yaw
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_YAW", 3, AP_MotorsBlimp, _yaw_factor[0], 0),
    // @Param: M1_THR
    // @DisplayName: Motor 1 throttle factor
    // @Description: How much motor 1 contributes to throttle (X axis)
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("M1_THR", 4, AP_MotorsBlimp, _throttle_factor[0], 0),

    // Motor 2 factors
    // @Param: M2_ROLL
    // @DisplayName: Motor 2 roll factor
    AP_GROUPINFO("M2_ROLL", 5, AP_MotorsBlimp, _roll_factor[1], 0),
    // @Param: M2_PITCH
    // @DisplayName: Motor 2 pitch factor
    AP_GROUPINFO("M2_PITCH", 6, AP_MotorsBlimp, _pitch_factor[1], 0),
    // @Param: M2_YAW
    // @DisplayName: Motor 2 yaw factor
    AP_GROUPINFO("M2_YAW", 7, AP_MotorsBlimp, _yaw_factor[1], 0),
    // @Param: M2_THR
    // @DisplayName: Motor 2 throttle factor
    AP_GROUPINFO("M2_THR", 8, AP_MotorsBlimp, _throttle_factor[1], 0),

    // Motor 3 factors
    // @Param: M3_ROLL
    // @DisplayName: Motor 3 roll factor
    AP_GROUPINFO("M3_ROLL", 9, AP_MotorsBlimp, _roll_factor[2], 0),
    // @Param: M3_PITCH
    // @DisplayName: Motor 3 pitch factor
    AP_GROUPINFO("M3_PITCH", 10, AP_MotorsBlimp, _pitch_factor[2], 0),
    // @Param: M3_YAW
    // @DisplayName: Motor 3 yaw factor
    AP_GROUPINFO("M3_YAW", 11, AP_MotorsBlimp, _yaw_factor[2], 0),
    // @Param: M3_THR
    // @DisplayName: Motor 3 throttle factor
    AP_GROUPINFO("M3_THR", 12, AP_MotorsBlimp, _throttle_factor[2], 0),

    // Motor 4 factors
    // @Param: M4_ROLL
    // @DisplayName: Motor 4 roll factor
    AP_GROUPINFO("M4_ROLL", 13, AP_MotorsBlimp, _roll_factor[3], 0),
    // @Param: M4_PITCH
    // @DisplayName: Motor 4 pitch factor
    AP_GROUPINFO("M4_PITCH", 14, AP_MotorsBlimp, _pitch_factor[3], 0),
    // @Param: M4_YAW
    // @DisplayName: Motor 4 yaw factor
    AP_GROUPINFO("M4_YAW", 15, AP_MotorsBlimp, _yaw_factor[3], 0),
    // @Param: M4_THR
    // @DisplayName: Motor 4 throttle factor
    AP_GROUPINFO("M4_THR", 16, AP_MotorsBlimp, _throttle_factor[3], 0),

    AP_GROUPEND
};

AP_MotorsBlimp::AP_MotorsBlimp(uint16_t speed_hz) :
    AP_Motors(speed_hz)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MotorsBlimp::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup motor mask
    for (uint8_t i = 0; i < AP_MOTORS_BLIMP_NUM_MOTORS; i++) {
        add_motor_num(i);
    }

    // set update rate
    set_update_rate(_speed_hz);

    // set initial spool state
    _spool_state = SpoolState::SHUT_DOWN;

    set_initialised_ok(true);
}

uint32_t AP_MotorsBlimp::get_motor_mask()
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_BLIMP_NUM_MOTORS; i++) {
        mask |= (1U << i);
    }
    return mask;
}

void AP_MotorsBlimp::output()
{
    update_throttle_filter();

    // Simple spool logic for blimp
    if (!armed() || !get_interlock()) {
        _spool_state = SpoolState::SHUT_DOWN;
    } else if (_spool_desired == DesiredSpoolState::SHUT_DOWN) {
        _spool_state = SpoolState::SHUT_DOWN;
    } else {
        _spool_state = SpoolState::THROTTLE_UNLIMITED;
    }

    // output motors if spooled up
    if (_spool_state == SpoolState::THROTTLE_UNLIMITED) {
        output_armed_stabilizing();
    } else {
        output_min();
    }
}

void AP_MotorsBlimp::output_min()
{
    set_desired_spool_state(DesiredSpoolState::SHUT_DOWN);
    _spool_state = SpoolState::SHUT_DOWN;
    output();
}

void AP_MotorsBlimp::output_armed_stabilizing()
{
    // Apply mixing matrix
    for (uint8_t i = 0; i < AP_MOTORS_BLIMP_NUM_MOTORS; i++) {
        float output = _roll_factor[i] * _roll_in
                     + _pitch_factor[i] * _pitch_in
                     + _yaw_factor[i] * _yaw_in
                     + _throttle_factor[i] * _throttle_in;
        output = constrain_float(output, 0.0f, 1.0f);
        // Convert [0,1] to PWM [1000,2000] and write via rc_write
        uint16_t pwm = 1000 + (uint16_t)(output * 1000);
        rc_write(i, pwm);
    }
}

void AP_MotorsBlimp::set_desired_spool_state(DesiredSpoolState desired)
{
    if (!armed() || !get_interlock()) {
        _spool_desired = DesiredSpoolState::SHUT_DOWN;
        return;
    }
    _spool_desired = desired;
}

void AP_MotorsBlimp::update_throttle_filter()
{
    if (armed()) {
        _throttle_filter.apply(_throttle_in, _dt);
        _throttle_filter.get();
    } else {
        _throttle_filter.reset(0.0f);
    }
}

void AP_MotorsBlimp::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    if (motor_seq >= 1 && motor_seq <= AP_MOTORS_BLIMP_NUM_MOTORS) {
        rc_write(motor_seq - 1, pwm);
    }
}
