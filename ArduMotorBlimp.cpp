#include "ArduMotorBlimp.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) \
    SCHED_TASK_CLASS(ArduMotorBlimp, &motorblimp, func, rate_hz, max_time_micros, priority)

const AP_Scheduler::Task ArduMotorBlimp::scheduler_tasks[] = {
    SCHED_TASK(one_hz_loop, 1, 100, 5),
};

ArduMotorBlimp::ArduMotorBlimp(void)
{
}

void ArduMotorBlimp::init_ardupilot()
{
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
