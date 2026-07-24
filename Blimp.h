/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Arming/AP_Arming.h>

#include "defines.h"
#include "config.h"
#include "AP_MotorsBlimp.h"
#include "RC_Channel.h"
#include "GCS_Blimp.h"
#include "Parameters.h"
#include "mode.h"

class Blimp : public AP_Vehicle
{
public:
    friend class GCS_Blimp;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Blimp;
    friend class RC_Channel_Blimp;
    friend class RC_Channels_Blimp;
    friend class Mode;
    friend class ModeManual;
    friend class ModeBrake;
    friend class AP_MotorsBlimp;

    Blimp(void);

private:

    Parameters g;
    ParametersG2 g2;

    RC_Channel *channel_right;
    RC_Channel *channel_front;
    RC_Channel *channel_up;
    RC_Channel *channel_yaw;

    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;

    AP_Arming_Blimp arming;

    GCS_Blimp _gcs;
    GCS_Blimp &gcs() { return _gcs; }

    typedef union {
        struct {
            uint8_t pre_arm_rc_check        : 1;
            uint8_t pre_arm_check           : 1;
            uint8_t auto_armed              : 1;
            uint8_t logging_started         : 1;
            uint8_t land_complete           : 1;
            uint8_t new_radio_frame         : 1;
            uint8_t rc_receiver_present_unused : 1;
            uint8_t compass_mot             : 1;
            uint8_t motor_test              : 1;
            uint8_t initialised             : 1;
            uint8_t land_complete_maybe     : 1;
            uint8_t throttle_zero           : 1;
            uint8_t gps_glitching           : 1;
            uint8_t in_arming_delay         : 1;
            uint8_t initialised_params      : 1;
        };
        uint32_t value;
    } ap_t;

    ap_t ap;
    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t must be uint32_t");

    Mode::Number control_mode;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;

    RCMapper rcmap;

    struct {
        int8_t radio_counter;
        uint8_t radio               : 1;
        uint8_t gcs                 : 1;
        uint8_t ekf                 : 1;
    } failsafe;

    bool any_failsafe_triggered() const
    {
        return failsafe.radio || failsafe.gcs || failsafe.ekf;
    }

    AP_MotorsBlimp *motors;

    uint32_t last_radio_update_ms;
    uint32_t arm_time_ms;

    AP_Param param_loader;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    enum Failsafe_Action {
        Failsafe_Action_None           = 0,
        Failsafe_Action_BRAKE          = 1,
        Failsafe_Action_Terminate      = 5
    };

    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),
        GCS_CONTINUE_IF_AUTO            = (1<<1),
        RC_CONTINUE_IF_GUIDED           = (1<<2),
        CONTINUE_IF_LANDING             = (1<<3),
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),
        RELEASE_GRIPPER                 = (1<<5),
    };

    static constexpr int8_t _failsafe_priorities[] = {
        Failsafe_Action_Terminate,
        Failsafe_Action_BRAKE,
        Failsafe_Action_None,
        -1
    };

    // AP_State.cpp
    void set_auto_armed(bool b);
    void set_failsafe_radio(bool b);
    void set_failsafe_gcs(bool b);

    // Blimp.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void rc_loop();
    void throttle_loop();

    // events.cpp
    bool failsafe_option(FailsafeOption opt) const;
    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void failsafe_gcs_check();
    bool should_disarm_on_failsafe();
    void do_failsafe_action(Failsafe_Action action, ModeReason reason);

    // failsafe.cpp
    void failsafe_enable();
    void failsafe_disable();

#if HAL_LOGGING_ENABLED
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override { return log_structure; }
    uint8_t get_num_log_structures() const override;
    void Log_Write_Vehicle_Startup_Messages();
    void Write_MOTORI(float yaw, float pitch, float roll, float x);
    void Write_MOTORO(float *outputs);
#endif

    // mode.cpp
    bool set_mode(Mode::Number mode, ModeReason reason);
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    uint8_t get_mode() const override { return (uint8_t)control_mode; }
    void update_flight_mode();
    void notify_flight_mode();

    // mode_brake.cpp
    void set_mode_brake_failsafe(ModeReason reason);

    // motors.cpp
    void arm_motors_check();
    void motors_output();

    // Parameters.cpp
    void load_parameters(void) override;

    // radio.cpp
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);

    // system.cpp
    void init_ardupilot() override;
    MAV_TYPE get_frame_mav_type();
    const char* get_frame_string();
    void allocate_motors(void);

    Mode *flightmode;
    ModeManual mode_manual;
    ModeBrake mode_brake;

    Mode *mode_from_mode_num(const Mode::Number mode);
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

public:
    void failsafe_check();
};

extern Blimp blimp;

using AP_HAL::millis;
using AP_HAL::micros;
