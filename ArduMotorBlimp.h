#pragma once

#include "config.h"

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Mission/AP_Mission_ChangeDetector.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Arming_MotorBlimp.h"
#include "defines.h"
#include "FlightControl.h"
#include "GCS_MotorBlimp.h"
#include "MotorMixer.h"
#include "Parameters.h"
#include "RC_Channel_MotorBlimp.h"

class ArduMotorBlimp : public AP_Vehicle {
    friend class AP_Arming_MotorBlimp;
    friend class GCS_MAVLINK_MotorBlimp;
    friend class GCS_MotorBlimp;
    friend class Parameters;
    friend class ParametersG2;

public:
    ArduMotorBlimp();

    // AP_Vehicle overrides
    void init_ardupilot() override;
    void load_parameters() override;
    bool set_mode(uint8_t new_mode, ModeReason reason) override;
    uint8_t get_mode() const override { return static_cast<uint8_t>(control_mode); }
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location &location, bool lock) override WARN_IF_UNUSED;
    bool current_mode_requires_mission() const override {
        return control_mode == Mode::AUTO;
    }
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    static const AP_Param::Info var_info[];
    static const LogStructure log_structure[];

    const LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }

private:
    static constexpr float MOTOR_SCALE = 1000.0f;
    static constexpr uint32_t WAYPOINT_SETTLE_MS = 1000U;

    Parameters g;
    ParametersG2 g2;

    RC_Channels_MotorBlimp rc_channels;
    SRV_Channels servo_channels;

    GCS_MotorBlimp _gcs;
    GCS_MotorBlimp &gcs() { return _gcs; }

    AP_BattMonitor battery{
        MASK_LOG_CURRENT,
        FUNCTOR_BIND_MEMBER(&ArduMotorBlimp::handle_battery_failsafe,
                            void, const char *, const int8_t),
        nullptr};
    AP_Arming_MotorBlimp arming;
    RCMapper rcmap;

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ArduMotorBlimp::start_mission_command,
                            bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ArduMotorBlimp::verify_mission_command,
                            bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ArduMotorBlimp::mission_complete, void)};
    AP_Mission_ChangeDetector mission_change_detector;
    AP_Beacon beacon;

    MotorMixer motor_mixer{g2};
    FlightControl flight_control{g2};

    AP_Param param_loader{var_info};

    static const AP_Scheduler::Task scheduler_tasks[];

    Mode control_mode = Mode::MANUAL;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;

    struct RCInput {
        float collective = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        bool valid = false;
    } rc_in;

    struct EstimatedState {
        Quaternion attitude_body_to_ned;
        Vector3f position_ned_m;
        Vector3f velocity_ned_mps;
        bool attitude_valid = false;
        bool position_valid = false;
        bool velocity_valid = false;
    } state;

    FlightControl::AttitudeTarget hold_target;
    Vector3f logged_attitude_target_deg;
    bool control_active = false;
    bool waypoint_reached = false;
    uint32_t waypoint_reached_since_ms = 0;
    bool guided_reached_reported = false;
    bool navigation_failsafe_active = false;
    bool navigation_failsafe_announced = false;
    bool rc_failsafe_announced = false;
    bool compass_failsafe_announced = false;

    // Scheduler tasks
    void ahrs_update();
    void control_loop();
    void motors_output();
    void rc_loop();
    void update_battery_compass();
    void update_barometer();
    void beacon_update();
    void mission_update();
    void one_hz_loop();
#if HAL_LOGGING_ENABLED
    void logging_25hz();
    void logging_10hz();
#endif

    void configure_rc_input();
    void configure_motor_output();
    void reset_control_state(bool clear_navigation_target);
    void stop_control();
    bool attitude_healthy() const;
    bool compass_healthy() const;
    bool navigation_healthy() const;
    bool set_hold_target_from_attitude();
    bool set_guided_target_ned(const Vector3f &target_ned);
    bool mission_is_supported() const;

    bool start_mission_command(const AP_Mission::Mission_Command &command);
    bool verify_mission_command(const AP_Mission::Mission_Command &command);
    void mission_complete();

    void handle_battery_failsafe(const char *type_str, int8_t action);

    // Independent 1 kHz watchdog callback.  It must be static to match the
    // HAL timer API and delegates to the single vehicle instance.
    static void failsafe_check_static();
    void failsafe_check();
    void output_motor_neutral();

    // Main-loop stall tolerated before the watchdog acts.  Copter uses 2 s;
    // 200 ms tripped on routine parameter saves and log flushes.
    static constexpr uint32_t CPU_FAILSAFE_TIMEOUT_US = 1000000U;
    uint16_t failsafe_last_ticks = 0;
    uint32_t failsafe_last_timestamp_us = 0;
    // set by the timer thread, cleared by control_loop()
    volatile bool cpu_failsafe_active = false;
};

extern ArduMotorBlimp motorblimp;
