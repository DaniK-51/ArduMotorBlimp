#include "ArduMotorBlimp.h"

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

bool AP_Arming_MotorBlimp::gps_checks(const bool report)
{
    // AP_Arming's generic GPS check always requires a GPS-derived home.  This
    // vehicle deliberately has GPS disabled: MANUAL needs only attitude/RC,
    // while autonomous modes use the UWB/compass EKF state instead.
    const Mode mode = static_cast<Mode>(motorblimp.get_mode());
    if ((mode == Mode::AUTO || mode == Mode::GUIDED) &&
        !motorblimp.navigation_healthy()) {
        check_failed(Check::GPS, report, "UWB/EKF/compass invalid");
        return false;
    }
    return true;
}

bool AP_Arming_MotorBlimp::pre_arm_checks(bool report)
{
    bool passed = AP_Arming::pre_arm_checks(report);

    if (!motorblimp.attitude_healthy()) {
        check_failed(Check::INS, report, "Attitude estimate invalid");
        passed = false;
    }

    // Every implemented mode closes an absolute yaw loop.  Generic ArduPilot
    // compass checks permit COMPASS_USE=0, but that configuration cannot fly
    // this controller and would otherwise arm only to remain motor-neutral.
    if (!motorblimp.compass_healthy()) {
        check_failed(Check::COMPASS, report,
                     "Compass unavailable/not used for yaw");
        passed = false;
    }

    const Mode mode = static_cast<Mode>(motorblimp.get_mode());
    if (mode == Mode::MANUAL && rc().in_rc_failsafe()) {
        check_failed(Check::RC, report, "No valid centred RC input");
        passed = false;
    }

    return passed;
}

bool AP_Arming_MotorBlimp::arm(const Method method, const bool do_arming_checks)
{
    if (is_armed()) {
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        return false;
    }

    hal.util->set_soft_armed(true);
    AP_Notify::flags.armed = true;
#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(true);
#endif
    send_arm_disarm_statustext("Arming motors");
    return true;
}

bool AP_Arming_MotorBlimp::disarm(const Method method, const bool do_disarm_checks)
{
    if (!is_armed()) {
        return true;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    // A GUIDED setpoint is intentionally persistent during flight, but must
    // never survive a disarm/re-arm cycle.  The vehicle may have been carried
    // while disarmed, so replaying the old coordinate would cause an
    // unexpected motor start.  AUTO mission state is kept separately and may
    // be resumed deliberately in the usual ArduPilot manner.
    const bool clear_navigation_target =
        static_cast<Mode>(motorblimp.get_mode()) == Mode::GUIDED;
    motorblimp.reset_control_state(clear_navigation_target);

    // Write the reversible-ESC stop value synchronously, while the HAL is
    // still soft-armed and will pass the pulse through.  Waiting for the next
    // main-loop motors_output() leaves a narrow stale-PWM race if the loop
    // locks immediately after this disarm request.
    motorblimp.output_motor_neutral();

    hal.util->set_soft_armed(false);
    AP_Notify::flags.armed = false;
#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif
    send_arm_disarm_statustext("Disarming motors");
    return true;
}
