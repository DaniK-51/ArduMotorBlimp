#include "Blimp.h"

bool AP_Arming_Blimp::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

bool AP_Arming_Blimp::run_pre_arm_checks(bool display_failure)
{
    if (blimp.motors->armed()) {
        return true;
    }

    if (!hal.scheduler->is_system_initialized()) {
        check_failed(display_failure, "System not initialised");
        return false;
    }

    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP)) {
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        return false;
    }

    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

    return parameter_checks(display_failure)
           && motor_checks(display_failure)
           && gcs_failsafe_check(display_failure);
}

bool AP_Arming_Blimp::barometer_checks(bool display_failure)
{
    return true;
}

bool AP_Arming_Blimp::ins_checks(bool display_failure)
{
    // No INS checks in manual-only build
    return true;
}

bool AP_Arming_Blimp::board_voltage_checks(bool display_failure)
{
    return AP_Arming::board_voltage_checks(display_failure);
}

bool AP_Arming_Blimp::parameter_checks(bool display_failure)
{
    if (check_enabled(ARMING_CHECK_PARAMETERS)) {
        if (blimp.g.failsafe_throttle) {
            if (blimp.channel_up->get_radio_min() <= blimp.g.failsafe_throttle_value+10 || blimp.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
    }
    return true;
}

bool AP_Arming_Blimp::motor_checks(bool display_failure)
{
    if (!blimp.motors->initialised_ok()) {
        check_failed(display_failure, "Check firmware or FRAME_CLASS");
        return false;
    }
    return true;
}

bool AP_Arming_Blimp::rc_calibration_checks(bool display_failure)
{
    return true;
}

bool AP_Arming_Blimp::gps_checks(bool display_failure)
{
    // No GPS checks in manual-only build
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

bool AP_Arming_Blimp::mandatory_gps_checks(bool display_failure)
{
    return true;
}

bool AP_Arming_Blimp::gcs_failsafe_check(bool display_failure)
{
    if (blimp.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

bool AP_Arming_Blimp::alt_checks(bool display_failure)
{
    // No altitude checks in manual-only build
    return true;
}

bool AP_Arming_Blimp::arm_checks(AP_Arming::Method method)
{
    return AP_Arming::arm_checks(method);
}

bool AP_Arming_Blimp::mandatory_checks(bool display_failure)
{
    bool result = true;

    if (!alt_checks(display_failure)) {
        result = false;
    }

    if (!motor_checks(display_failure)) {
        result = false;
    }

    return result;
}

void AP_Arming_Blimp::set_pre_arm_check(bool b)
{
    blimp.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}

bool AP_Arming_Blimp::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    if (blimp.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(true);
#endif

    AP_Notify::flags.armed = true;
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    send_arm_disarm_statustext("Arming motors");

    hal.util->set_soft_armed(true);

    blimp.motors->armed(true);

#if HAL_LOGGING_ENABLED
    AP::logger().Write_Mode((uint8_t)blimp.control_mode, blimp.control_mode_reason);
#endif

    AP::scheduler().perf_info.ignore_this_loop();

    in_arm_motors = false;
    blimp.arm_time_ms = millis();
    blimp.ap.in_arming_delay = true;

    return true;
}

bool AP_Arming_Blimp::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (!blimp.motors->armed()) {
        return true;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    send_arm_disarm_statustext("Disarming motors");

    blimp.motors->armed(false);

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    hal.util->set_soft_armed(false);

    blimp.ap.in_arming_delay = false;

    return true;
}
