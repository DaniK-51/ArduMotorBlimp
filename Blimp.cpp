#include "Blimp.h"

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Blimp, &blimp, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Blimp, &blimp, func)

const AP_Scheduler::Task Blimp::scheduler_tasks[] = {
    FAST_TASK_CLASS(AP_InertialSensor, &blimp.ins, update),
    FAST_TASK(motors_output),
    FAST_TASK(update_flight_mode),

    SCHED_TASK(rc_loop,              100,    130,   3),
    SCHED_TASK(throttle_loop,         50,     75,   6),
    SCHED_TASK(arm_motors_check,      10,     50,  18),
    SCHED_TASK_CLASS(AP_Notify,            &blimp.notify,              update,          50,  90,  36),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_receive, 400, 180,  51),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_send,    400, 550,  54),
};

void Blimp::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Blimp::_failsafe_priorities[4];

void Blimp::rc_loop()
{
    read_radio();
    rc().read_mode_switch();
}

void Blimp::throttle_loop()
{
    update_auto_armed();
}

Blimp::Blimp(void)
    :
      flight_modes(&g.flight_mode1),
      control_mode(Mode::Number::MANUAL),
      rc_throttle_control_in_filter(1.0f),
      param_loader(var_info),
      flightmode(&mode_manual)
{
}

Blimp blimp;
AP_Vehicle& vehicle = blimp;

AP_HAL_MAIN_CALLBACKS(&blimp);
