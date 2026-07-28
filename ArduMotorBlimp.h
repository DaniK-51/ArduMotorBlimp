#pragma once

#include <AP_Vehicle/AP_Vehicle.h>
#include <SRV_Channel/SRV_Channel.h>

#include "config.h"
#include "defines.h"
#include "Parameters.h"
#include "GCS_MotorBlimp.h"
#include "RC_Channel_MotorBlimp.h"
#include "AP_Arming_MotorBlimp.h"

class ArduMotorBlimp : public AP_Vehicle {
    friend class GCS_MAVLINK_MotorBlimp;
    friend class GCS_MotorBlimp;
    friend class Parameters;

public:
    ArduMotorBlimp(void);

    // AP_Vehicle overrides
    void init_ardupilot() override;
    void load_parameters() override;
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    uint8_t get_mode() const override;
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    static const AP_Param::Info var_info[];

private:
    Parameters g;

    static const AP_Scheduler::Task scheduler_tasks[];

    RC_Channels_MotorBlimp rc_channels;
    SRV_Channels servo_channels;

    GCS_MotorBlimp _gcs;
    GCS_MotorBlimp &gcs() { return _gcs; }

    AP_BattMonitor battery{0,
                           FUNCTOR_BIND_MEMBER(&ArduMotorBlimp::handle_battery_failsafe, void, const char*, const int8_t),
                           nullptr};

    void handle_battery_failsafe(const char* type_str, const int8_t action);

    AP_Arming_MotorBlimp arming;

    AP_Param param_loader{var_info};

    // RC input — normalized values [-1, 1]
    struct {
        float forward;  // throttle stick
        float roll;     // roll stick
        float pitch;    // pitch stick
        float yaw;      // yaw stick
    } rc_in;

    // Scheduler tasks
    void rc_loop();
    void motors_output();
    void one_hz_loop();
};

extern ArduMotorBlimp motorblimp;
