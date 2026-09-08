#include "ArduMotorBlimp.h"

#include <AP_Logger/AP_Logger.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) \
    SCHED_TASK_CLASS(ArduMotorBlimp, &motorblimp, func, rate_hz, \
                     max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(ArduMotorBlimp, &motorblimp, func)

const AP_Scheduler::Task ArduMotorBlimp::scheduler_tasks[] = {
    // The estimator and controller must run in this order on every main loop.
    FAST_TASK_CLASS(AP_InertialSensor, &motorblimp.ins, update),
    FAST_TASK(ahrs_update),
    FAST_TASK(control_loop),
    FAST_TASK(motors_output),

    SCHED_TASK(rc_loop, 100, 130, 3),
    SCHED_TASK_CLASS(AP_GPS, &motorblimp.gps, update, 50, 200, 9),
    SCHED_TASK(beacon_update, 50, 120, 12),
    SCHED_TASK(update_battery_compass, 10, 160, 15),
    SCHED_TASK(update_barometer, 10, 100, 18),
    SCHED_TASK(mission_update, 10, 200, 27),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(logging_25hz, 25, 220, 33),
#endif
    SCHED_TASK_CLASS(AP_Notify, &motorblimp.notify, update, 50, 90, 36),
    SCHED_TASK(one_hz_loop, 1, 100, 39),
    SCHED_TASK_CLASS(GCS, (GCS *)&motorblimp._gcs,
                     update_receive, 400, 180, 51),
    SCHED_TASK_CLASS(GCS, (GCS *)&motorblimp._gcs,
                     update_send, 400, 550, 54),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(logging_10hz, 10, 350, 57),
    SCHED_TASK_CLASS(AP_Logger, &motorblimp.logger,
                     periodic_tasks, 400, 300, 63),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &motorblimp.ins,
                     periodic, 400, 50, 66),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &motorblimp.scheduler,
                     update_logging, 0.1, 75, 69),
#endif
};

const LogStructure ArduMotorBlimp::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

uint8_t ArduMotorBlimp::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

ArduMotorBlimp::ArduMotorBlimp() = default;

void ArduMotorBlimp::load_parameters()
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // Motor commands are signed around a real neutral.  These are defaults
    // only: SRV_Channels::set_digital_outputs() re-trims DShot channels at
    // boot and forces TRIM to 1000 unless the channel is in
    // SERVO_BLH_3DMASK, so the pre-arm check in AP_Arming_MotorBlimp
    // refuses to arm until every motor output is back on 1000/1500/2000.
    AP_Param::set_default_by_name("SCHED_LOOP_RATE", 400);
    AP_Param::set_default_by_name("SERVO1_MIN", 1000);
    AP_Param::set_default_by_name("SERVO1_TRIM", 1500);
    AP_Param::set_default_by_name("SERVO1_MAX", 2000);
    AP_Param::set_default_by_name("SERVO2_MIN", 1000);
    AP_Param::set_default_by_name("SERVO2_TRIM", 1500);
    AP_Param::set_default_by_name("SERVO2_MAX", 2000);
    AP_Param::set_default_by_name("SERVO3_MIN", 1000);
    AP_Param::set_default_by_name("SERVO3_TRIM", 1500);
    AP_Param::set_default_by_name("SERVO3_MAX", 2000);
    AP_Param::set_default_by_name("SERVO4_MIN", 1000);
    AP_Param::set_default_by_name("SERVO4_TRIM", 1500);
    AP_Param::set_default_by_name("SERVO4_MAX", 2000);
}

void ArduMotorBlimp::configure_rc_input()
{
    RC_Channel &roll = rc().get_roll_channel();
    RC_Channel &pitch = rc().get_pitch_channel();
    RC_Channel &collective = rc().get_throttle_channel();
    RC_Channel &yaw = rc().get_yaw_channel();

    // All four controls are centred, signed inputs.  In particular, channel 3
    // is not a conventional zero-to-full multicopter throttle.
    roll.set_angle(1000);
    pitch.set_angle(1000);
    collective.set_angle(1000);
    yaw.set_angle(1000);

    roll.set_default_dead_zone(20);
    pitch.set_default_dead_zone(20);
    collective.set_default_dead_zone(30);
    yaw.set_default_dead_zone(20);
}

void ArduMotorBlimp::configure_motor_output()
{
    ParametersG2::set_default_motor_functions();

    SRV_Channels::set_angle(SRV_Channel::k_motor1, MOTOR_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, MOTOR_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, MOTOR_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, MOTOR_SCALE);

    AP::srv().enable_aux_servos();
    SRV_Channels::update_aux_servo_function();

    // An IO failsafe must command the reversible ESC neutral, never MIN
    // (which would be full reverse with this actuator convention).
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_motor1,
                                     SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_motor2,
                                     SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_motor3,
                                     SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_motor4,
                                     SRV_Channel::Limit::TRIM);
}

void ArduMotorBlimp::init_ardupilot()
{
    hal.console->printf("ArduMotorBlimp: initialising\n");

    notify.init();
    battery.init();
    barometer.init();

    gcs().setup_uarts();

    configure_rc_input();
    rc().init();
    configure_motor_output();

    // This callback runs independently of the main loop.  A lockup must not
    // leave the last signed motor command latched on the ESCs.
    failsafe_last_timestamp_us = AP_HAL::micros();
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    gps.init();
    AP::compass().init();
    barometer.calibrate();
    beacon.init();
    mission.init();

    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    Quaternion initial_attitude;
    ahrs.get_quat_body_to_ned(initial_attitude);
    flight_control.reset(initial_attitude);
    motor_mixer.reset();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "MotorBlimp ready: MANUAL/HOLD/AUTO/GUIDED");
}

void ArduMotorBlimp::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                         uint8_t &task_count,
                                         uint32_t &log_bit)
{
    tasks = scheduler_tasks;
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

bool ArduMotorBlimp::attitude_healthy() const
{
    return state.attitude_valid && ahrs.initialised() && ahrs.healthy() &&
           ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID);
}

bool ArduMotorBlimp::compass_healthy() const
{
    const Compass &vehicle_compass = AP::compass();
    return vehicle_compass.available() && vehicle_compass.healthy() &&
           vehicle_compass.use_for_yaw() && AP::ahrs().use_compass();
}

bool ArduMotorBlimp::navigation_healthy() const
{
    // This vehicle has exactly one absolute-position source.  Do not rely on
    // the EKF validity flags alone here: after the tag disappears the EKF can
    // legitimately coast in dead reckoning for several seconds.  AUTO and
    // GUIDED must instead stop as soon as AP_Beacon times out the direct XYZ
    // fix (AP_BEACON_TIMEOUT_MS).
    Vector3f beacon_position;
    float beacon_accuracy = 0.0f;
    const bool beacon_position_fresh =
        beacon.enabled() && beacon.healthy() &&
        beacon.get_vehicle_position_ned(beacon_position, beacon_accuracy) &&
        !beacon_position.is_nan() && !beacon_position.is_inf() &&
        isfinite(beacon_accuracy) && beacon_accuracy >= 0.0f &&
        beacon_accuracy <= MAX(g2.uwb_accuracy_max_m.get(), 0.0f);

    const bool horizontal_position =
        ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL) ||
        ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
    const bool vertical_position = ahrs.has_status(AP_AHRS::Status::VERT_POS);
    const bool vertical_velocity = ahrs.has_status(AP_AHRS::Status::VERT_VEL);

    return beacon_position_fresh && attitude_healthy() && compass_healthy() &&
           ahrs.have_inertial_nav() &&
           state.position_valid && state.velocity_valid &&
           horizontal_position && vertical_position && vertical_velocity &&
           !ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE);
}

bool ArduMotorBlimp::mission_is_supported() const
{
    if (!mission.present()) {
        return false;
    }

    for (uint16_t index = AP_MISSION_FIRST_REAL_COMMAND;
         index < mission.num_commands(); index++) {
        AP_Mission::Mission_Command command;
        if (!mission.read_cmd_from_storage(index, command) ||
            command.id != MAV_CMD_NAV_WAYPOINT) {
            return false;
        }
    }
    return true;
}

bool ArduMotorBlimp::set_home_to_current_location(bool lock)
{
    Location current_location;
    if (!ahrs.get_location(current_location)) {
        return false;
    }
    return set_home(current_location, lock);
}

bool ArduMotorBlimp::set_home(const Location &location, bool lock)
{
    // A beacon origin is the global reference for both HOME_POSITION and
    // GLOBAL_RELATIVE_ALT mission items.  Do not accept a home until that EKF
    // origin exists.
    Location origin;
    if (!ahrs.get_origin(origin) || !ahrs.set_home(location)) {
        return false;
    }
    if (lock) {
        ahrs.lock_home();
    }
    return true;
}

bool ArduMotorBlimp::set_hold_target_from_attitude()
{
    hold_target = FlightControl::AttitudeTarget {};
    if (!state.attitude_valid) {
        return false;
    }
    hold_target.attitude_body_to_ned = state.attitude_body_to_ned;
    hold_target.collective = 0.0f;
    hold_target.valid = true;
    return true;
}

bool ArduMotorBlimp::set_mode(uint8_t new_mode_value, ModeReason reason)
{
    const Mode new_mode = static_cast<Mode>(new_mode_value);
    switch (new_mode) {
    case Mode::MANUAL:
    case Mode::HOLD:
    case Mode::AUTO:
    case Mode::GUIDED:
        break;
    default:
        notify_no_such_mode(new_mode_value);
        return false;
    }

    if (new_mode == control_mode) {
        return true;
    }

    if ((new_mode == Mode::MANUAL || new_mode == Mode::HOLD) &&
        !attitude_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Mode rejected: attitude invalid");
        return false;
    }
    if (mode_requires_compass(new_mode) && !compass_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Mode rejected: compass invalid");
        return false;
    }
    if ((new_mode == Mode::AUTO || new_mode == Mode::GUIDED) &&
        !navigation_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Mode rejected: UWB/EKF/compass invalid");
        return false;
    }
    if (new_mode == Mode::AUTO && !ahrs.home_is_set() &&
        !set_home_to_current_location(false)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "AUTO rejected: home unavailable");
        return false;
    }
    if (new_mode == Mode::AUTO && !mission_is_supported()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "AUTO rejected: waypoint-only mission required");
        return false;
    }

    if (control_mode == Mode::AUTO) {
        mission.stop();
    }

    control_mode = new_mode;
    control_mode_reason = reason;
    reset_control_state(new_mode == Mode::GUIDED);

    if (new_mode == Mode::HOLD && !set_hold_target_from_attitude()) {
        // Fall back to MANUAL and announce it like any other mode change so
        // the GCS, notify outputs and the log do not keep reporting HOLD.
        control_mode = Mode::MANUAL;
        control_mode_reason = ModeReason::UNAVAILABLE;
        AP_Notify::flags.autopilot_mode = false;
        AP_Notify::flags.flight_mode = uint8_t(Mode::MANUAL);
#if HAL_LOGGING_ENABLED
        AP::logger().Write_Mode(uint8_t(Mode::MANUAL), ModeReason::UNAVAILABLE);
#endif
        gcs().send_message(MSG_HEARTBEAT);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "HOLD rejected: no attitude target, staying in MANUAL");
        return false;
    }
    if (new_mode == Mode::AUTO) {
        flight_control.clear_position_target();
        mission.start_or_resume();
        // Seed the detector after start_or_resume has selected the active
        // command.  Subsequent in-place uploads are then distinguished from
        // ordinary mission progress.
        IGNORE_RETURN(mission_change_detector.check_for_mission_change());
    }

    AP_Notify::flags.autopilot_mode =
        new_mode == Mode::AUTO || new_mode == Mode::GUIDED;
    AP_Notify::flags.flight_mode = new_mode_value;
#if HAL_LOGGING_ENABLED
    AP::logger().Write_Mode(new_mode_value, reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode %u", unsigned(new_mode_value));
    return true;
}

void ArduMotorBlimp::reset_control_state(bool clear_navigation_target)
{
    flight_control.reset(state.attitude_body_to_ned);
    if (clear_navigation_target) {
        flight_control.clear_position_target();
    }
    motor_mixer.reset();
    hold_target = FlightControl::AttitudeTarget {};
    waypoint_reached = false;
    waypoint_reached_since_ms = 0;
    guided_reached_reported = false;
    navigation_failsafe_active = false;
    navigation_failsafe_announced = false;
    rc_failsafe_announced = false;
    compass_failsafe_announced = false;
    control_active = false;
}

void ArduMotorBlimp::stop_control()
{
    if (control_active) {
        flight_control.reset_rate_controllers();
    }
    motor_mixer.reset();
    waypoint_reached = false;
    waypoint_reached_since_ms = 0;
    control_active = false;
}

void ArduMotorBlimp::ahrs_update()
{
    // INS was updated by the preceding FAST_TASK.
    ahrs.update(true);

    Quaternion attitude;
    ahrs.get_quat_body_to_ned(attitude);
    state.attitude_valid = FlightControl::quaternion_is_valid(attitude) &&
                           ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID);
    if (state.attitude_valid) {
        attitude.normalize();
        state.attitude_body_to_ned = attitude;
    }

    Vector3f position;
    state.position_valid =
        ahrs.get_relative_position_NED_origin_float(position) &&
        FlightControl::vector_is_finite(position);
    if (state.position_valid) {
        state.position_ned_m = position;
    }

    Vector3f velocity;
    state.velocity_valid = ahrs.get_velocity_NED(velocity) &&
                           FlightControl::vector_is_finite(velocity);
    if (state.velocity_valid) {
        state.velocity_ned_mps = velocity;
    }
}

void ArduMotorBlimp::rc_loop()
{
    rc().read_input();
    rc().read_aux_all();

    rc_in.valid = rc().has_valid_input();
    if (!rc_in.valid) {
        rc_in.collective = 0.0f;
        rc_in.roll = 0.0f;
        rc_in.pitch = 0.0f;
        rc_in.yaw = 0.0f;
        return;
    }

    rc_in.collective = rc().get_throttle_channel().norm_input_dz();
    rc_in.roll = rc().get_roll_channel().norm_input_dz();
    rc_in.pitch = rc().get_pitch_channel().norm_input_dz();
    rc_in.yaw = rc().get_yaw_channel().norm_input_dz();
}

void ArduMotorBlimp::control_loop()
{
    // The timer-thread watchdog only cuts the HAL soft-arm and writes
    // neutral; the full disarm bookkeeping (state reset, logging, GCS text)
    // is not timer-thread safe and is completed here once the loop runs.
    if (cpu_failsafe_active) {
        cpu_failsafe_active = false;
        if (arming.is_armed()) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU,
                               LogErrorCode::FAILSAFE_OCCURRED);
            IGNORE_RETURN(arming.disarm(AP_Arming::Method::CPUFAILSAFE, false));
        }
    }

    // Do not let rate integrators or the allocator continue winding up while
    // the hardware safety switch or emergency stop is suppressing outputs.
    if (!arming.is_armed_and_safety_off() ||
        SRV_Channels::get_emergency_stop() ||
        battery.has_failsafed() || !attitude_healthy() ||
        (mode_requires_compass(control_mode) && !compass_healthy())) {
        stop_control();
        return;
    }

    if (!control_active) {
        flight_control.reset(state.attitude_body_to_ned);
        if (control_mode == Mode::HOLD && !set_hold_target_from_attitude()) {
            stop_control();
            return;
        }
        control_active = true;
    }

    const float dt = scheduler.get_last_loop_time_s();
    FlightControl::AttitudeTarget attitude_target;

    if (control_mode == Mode::MANUAL) {
        // Never retain a stale stick command after an RC timeout.
        if (!rc_in.valid || rc().in_rc_failsafe()) {
            stop_control();
            return;
        }

        const FlightControl::ManualInput input {
            rc_in.collective, rc_in.roll, rc_in.pitch, rc_in.yaw
        };
        if (!flight_control.build_manual_target(
                input, state.attitude_body_to_ned, dt, attitude_target)) {
            stop_control();
            return;
        }
    } else if (control_mode == Mode::HOLD) {
        attitude_target = hold_target;
        attitude_target.collective = 0.0f;
    } else {
        // AUTO deliberately ignores RC/GCS loss, but it cannot fly blind.
        if (!navigation_healthy() || !flight_control.has_position_target()) {
            navigation_failsafe_active = true;
            stop_control();
            return;
        }

        const FlightControl::NavigationState navigation_state {
            state.position_ned_m,
            state.velocity_ned_mps,
            state.attitude_body_to_ned,
            state.position_valid,
            state.velocity_valid,
            state.attitude_valid,
        };
        const FlightControl::GuidanceOutput guidance =
            flight_control.update_guidance(navigation_state, dt);
        if (!guidance.valid) {
            stop_control();
            return;
        }
        navigation_failsafe_active = false;
        navigation_failsafe_announced = false;
        attitude_target = guidance.attitude;

        if (guidance.waypoint_reached) {
            if (waypoint_reached_since_ms == 0) {
                waypoint_reached_since_ms = AP_HAL::millis();
            }
            waypoint_reached =
                AP_HAL::millis() - waypoint_reached_since_ms >= WAYPOINT_SETTLE_MS;
        } else {
            waypoint_reached = false;
            waypoint_reached_since_ms = 0;
        }

        if (control_mode == Mode::GUIDED && waypoint_reached &&
            !guided_reached_reported) {
            gcs().send_mission_item_reached_message(0);
            guided_reached_reported = true;
        }
    }

    const FlightControl::AttitudeControlOutput control =
        flight_control.update_attitude(
            attitude_target,
            state.attitude_body_to_ned,
            ahrs.get_gyro(),
            dt,
            motor_mixer.result().attitude_limited);
    if (!control.valid) {
        stop_control();
        return;
    }

    attitude_target.attitude_body_to_ned.to_euler(logged_attitude_target_deg);
    logged_attitude_target_deg *= RAD_TO_DEG;

    const MotorMixer::Command command {
        control.collective,
        control.torque.x,
        control.torque.y,
        control.torque.z,
    };
    if (!motor_mixer.allocate(command).valid) {
        stop_control();
    }
}

void ArduMotorBlimp::motors_output()
{
    const MotorMixer::Result &allocation = motor_mixer.result();
    const bool enabled = arming.is_armed_and_safety_off() &&
                         !SRV_Channels::get_emergency_stop() && control_active &&
                         allocation.valid && !battery.has_failsafed() &&
                         (!mode_requires_compass(control_mode) ||
                          compass_healthy());

    // A reversible DShot ESC is stopped only at exactly 1500 us; anything
    // else is at least minimum throttle.  Snap small commands to neutral so
    // the attitude loops cannot keep the motors idling around zero.
    const float deadband = constrain_float(g2.mix_deadband.get(), 0.0f, 0.3f);
    const auto motor_output = [&](float command) {
        return (enabled && fabsf(command) >= deadband) ? command * MOTOR_SCALE
                                                        : 0.0f;
    };

    SRV_Channels::set_output_scaled(
        SRV_Channel::k_motor1, motor_output(allocation.motor[0]));
    SRV_Channels::set_output_scaled(
        SRV_Channel::k_motor2, motor_output(allocation.motor[1]));
    SRV_Channels::set_output_scaled(
        SRV_Channel::k_motor3, motor_output(allocation.motor[2]));
    SRV_Channels::set_output_scaled(
        SRV_Channel::k_motor4, motor_output(allocation.motor[3]));

    SRV_Channels::calc_pwm();

    // Cork so all four motor channels leave in a single DShot frame per
    // loop; uncorked writes trigger a separate send per channel, which
    // breaks the frame cadence the ESCs lock onto.
    auto &srv = AP::srv();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
}

void ArduMotorBlimp::output_motor_neutral()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, 0.0f);
    SRV_Channels::calc_pwm();

    // cork/push here as well: if the main loop stalled between its own
    // cork() and push(), an uncorked write would only land in the corked
    // buffer and the last thrust command would stay latched on the ESCs.
    auto &srv = AP::srv();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
}

void ArduMotorBlimp::failsafe_check_static()
{
    motorblimp.failsafe_check();
}

void ArduMotorBlimp::failsafe_check()
{
    const uint32_t now_us = AP_HAL::micros();
    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // main loop is running; cpu_failsafe_active is consumed by control_loop()
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp_us = now_us;
        return;
    }

    if (arming.is_armed() &&
        now_us - failsafe_last_timestamp_us > CPU_FAILSAFE_TIMEOUT_US) {
        // Timer-thread context, mirror Copter::failsafe_check(): only cut the
        // HAL soft-arm (DShot then carries zero throttle) and write neutral,
        // and repeat once per second while the stall persists.  The explicit
        // SRV write is still required because a reversible ESC's safe value
        // is TRIM, not MIN or an absent pulse.  Everything else happens in
        // control_loop() when the main loop is back.
        failsafe_last_timestamp_us = now_us;
        cpu_failsafe_active = true;
        hal.util->set_soft_armed(false);
        output_motor_neutral();
    }
}

void ArduMotorBlimp::update_battery_compass()
{
    battery.read();
    if (AP::compass().available()) {
        AP::compass().set_voltage(battery.voltage());
        AP::compass().read();
    }
}

void ArduMotorBlimp::update_barometer()
{
    barometer.update();
}

void ArduMotorBlimp::beacon_update()
{
    beacon.update();
}

void ArduMotorBlimp::mission_update()
{
    if (control_mode != Mode::AUTO) {
        return;
    }

    if (mission_change_detector.check_for_mission_change()) {
        // Never continue towards a cached target after the GCS changes or
        // replaces the mission.  AP_Mission will invoke our start callback
        // again while restarting, which installs the updated NED target.
        flight_control.clear_position_target();
        stop_control();

        bool restarted = false;
        if (mission.present()) {
            if (mission.state() == AP_Mission::MISSION_RUNNING) {
                restarted = mission.restart_current_nav_cmd();
            } else {
                mission.start_or_resume();
                restarted = mission.state() == AP_Mission::MISSION_RUNNING;
            }
        }

        if (restarted && flight_control.has_position_target()) {
            navigation_failsafe_active = false;
            navigation_failsafe_announced = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "AUTO mission changed: command restarted");
        } else {
            mission.stop();
            navigation_failsafe_active = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "AUTO mission changed: no safe command");
            return;
        }
    }

    mission.update();
}

#if HAL_LOGGING_ENABLED
void ArduMotorBlimp::logging_25hz()
{
    if (logger.should_log(MASK_LOG_ATTITUDE_MED)) {
        ahrs.Write_Attitude(logged_attitude_target_deg);
    }
    if (logger.should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, g2.rate_roll_pid.get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, g2.rate_pitch_pid.get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, g2.rate_yaw_pid.get_pid_info());
    }
    if (logger.should_log(MASK_LOG_IMU)) {
        ins.Write_IMU();
    }
}

void ArduMotorBlimp::logging_10hz()
{
    if (logger.should_log(MASK_LOG_ATTITUDE_MED)) {
        // Includes EKF, AHR2 and POS records in addition to ATT above.
        ahrs.Log_Write();
    }
    if (logger.should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (logger.should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (logger.should_log(MASK_LOG_IMU)) {
        ins.Write_Vibration();
    }
    if (logger.should_log(MASK_LOG_COMPASS)) {
        logger.Write_Compass();
    }
}
#endif

bool ArduMotorBlimp::set_guided_target_ned(const Vector3f &target_ned)
{
    if (control_mode != Mode::GUIDED || !navigation_healthy() ||
        !flight_control.set_position_target(target_ned)) {
        return false;
    }

    waypoint_reached = false;
    waypoint_reached_since_ms = 0;
    guided_reached_reported = false;
    navigation_failsafe_active = false;
    navigation_failsafe_announced = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GUIDED target accepted");
    return true;
}

bool ArduMotorBlimp::start_mission_command(
    const AP_Mission::Mission_Command &command)
{
    if (control_mode != Mode::AUTO || command.id != MAV_CMD_NAV_WAYPOINT) {
        return false;
    }

    Vector3f target_ned;
    if (!command.content.location.get_vector_from_origin_NED_m(target_ned) ||
        !flight_control.set_position_target(target_ned)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Mission waypoint has no valid EKF origin");
        return false;
    }

    waypoint_reached = false;
    waypoint_reached_since_ms = 0;
    navigation_failsafe_active = false;
    navigation_failsafe_announced = false;
    return true;
}

bool ArduMotorBlimp::verify_mission_command(
    const AP_Mission::Mission_Command &command)
{
    if (control_mode != Mode::AUTO || command.id != MAV_CMD_NAV_WAYPOINT) {
        return false;
    }
    if (waypoint_reached) {
        gcs().send_mission_item_reached_message(command.index);
        return true;
    }
    return false;
}

void ArduMotorBlimp::mission_complete()
{
    // Keep holding the final waypoint.  No ground stream is required after
    // mission upload, including after the last item completes.
    if (control_mode == Mode::AUTO) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Mission complete: holding final waypoint");
    }
}

void ArduMotorBlimp::one_hz_loop()
{
    AP::srv().enable_aux_servos();

    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = arming.gps_checks(false);
    AP_Notify::flags.armed = arming.is_armed();
    AP_Notify::flags.flying = arming.is_armed_and_safety_off();

    // Establish a conventional takeoff/home reference once the UWB EKF has a
    // global origin.  This makes standard GLOBAL_RELATIVE_ALT mission items
    // interoperable with QGC while keeping HOME fixed throughout a flight.
    if (!arming.is_armed() && !ahrs.home_is_set() && navigation_healthy()) {
        IGNORE_RETURN(set_home_to_current_location(false));
    }

    const bool rc_failsafe_active =
        arming.is_armed() && control_mode == Mode::MANUAL &&
        (!rc_in.valid || rc().in_rc_failsafe());
    if (rc_failsafe_active && !rc_failsafe_announced) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RC failsafe: motor neutral");
        rc_failsafe_announced = true;
    } else if (!rc_failsafe_active) {
        rc_failsafe_announced = false;
    }

    const bool compass_failsafe_active =
        arming.is_armed() && mode_requires_compass(control_mode) &&
        !compass_healthy();
    if (compass_failsafe_active && !compass_failsafe_announced) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Compass failsafe: motor neutral");
        compass_failsafe_announced = true;
    } else if (!compass_failsafe_active) {
        compass_failsafe_announced = false;
    }

    const bool autonomous_navigation_failsafe =
        arming.is_armed() &&
        (control_mode == Mode::AUTO || control_mode == Mode::GUIDED) &&
        navigation_failsafe_active;
    if (autonomous_navigation_failsafe &&
        !navigation_failsafe_announced) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Navigation failsafe: motor neutral");
        navigation_failsafe_announced = true;
    } else if (!autonomous_navigation_failsafe) {
        navigation_failsafe_announced = false;
    }
}

void ArduMotorBlimp::handle_battery_failsafe(const char *type_str,
                                              int8_t action)
{
    (void)action;
    stop_control();
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Battery failsafe: %s",
                  type_str != nullptr ? type_str : "unknown");
}

// Stubs required by optional linked vehicle libraries.
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>

bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate,
                                        const char *reason)
{
    (void)should_terminate;
    (void)reason;
    return false;
}

AP_AdvancedFailsafe *AP::advancedfailsafe()
{
    return nullptr;
}

AP_Avoidance *AP::ap_avoidance()
{
    return nullptr;
}

ArduMotorBlimp motorblimp;
AP_Vehicle &vehicle = motorblimp;
AP_HAL_MAIN_CALLBACKS(&motorblimp);
