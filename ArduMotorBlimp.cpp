#include "ArduMotorBlimp.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) \
    SCHED_TASK_CLASS(ArduMotorBlimp, &motorblimp, func, rate_hz, max_time_micros, priority)

const AP_Scheduler::Task ArduMotorBlimp::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,           100,    130,   3),
    SCHED_TASK(motors_output,     400,    100,   6),
    SCHED_TASK(one_hz_loop,         1,    100,   9),
};

ArduMotorBlimp::ArduMotorBlimp(void)
{
}

void ArduMotorBlimp::init_ardupilot()
{
    rc().init();
}

void ArduMotorBlimp::load_parameters()
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);
}

bool ArduMotorBlimp::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    return true;
}

uint8_t ArduMotorBlimp::get_mode() const
{
    return (uint8_t)Mode::MANUAL;
}

void ArduMotorBlimp::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                          uint8_t &task_count,
                                          uint32_t &log_bit)
{
    tasks = scheduler_tasks;
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = 0;
}

void ArduMotorBlimp::rc_loop()
{
    rc().read_input();

    const RC_Channel &ch_throttle = rc().get_throttle_channel();
    const RC_Channel &ch_roll = rc().get_roll_channel();
    const RC_Channel &ch_pitch = rc().get_pitch_channel();
    const RC_Channel &ch_yaw = rc().get_yaw_channel();

    rc_in.forward = ch_throttle.norm_input();
    rc_in.roll = ch_roll.norm_input();
    rc_in.pitch = ch_pitch.norm_input();
    rc_in.yaw = ch_yaw.norm_input();
}

void ArduMotorBlimp::motors_output()
{
    // For now — pass-through RC to outputs
    // Motor mixing will be added later
    if (!arming.is_armed()) {
        SRV_Channels::set_output_norm(SRV_Channel::k_motor1, 0);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor2, 0);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor3, 0);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor4, 0);
    } else {
        // Direct pass-through for testing
        SRV_Channels::set_output_norm(SRV_Channel::k_motor1, rc_in.forward);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor2, rc_in.forward);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor3, rc_in.pitch);
        SRV_Channels::set_output_norm(SRV_Channel::k_motor4, rc_in.roll);
    }

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
}

void ArduMotorBlimp::one_hz_loop()
{
}

void ArduMotorBlimp::handle_battery_failsafe(const char* type_str, const int8_t action)
{
}

// Stubs for symbols required by linked libraries
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
AP_Avoidance *AP::ap_avoidance() { return nullptr; }

ArduMotorBlimp motorblimp;
AP_Vehicle& vehicle = motorblimp;
AP_HAL_MAIN_CALLBACKS(&motorblimp);
