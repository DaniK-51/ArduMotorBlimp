#include "Blimp.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    blimp.failsafe_check();
}

void Blimp::init_ardupilot()
{
    // initialise notify system
    notify.init();
    notify_flight_mode();

    // setup telem slots with serial ports
    gcs().setup_uarts();

    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    allocate_motors();

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // sets up motors and output to escs
    init_rc_out();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // attempt to switch to MANUAL
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        set_mode(Mode::Number::MANUAL, ModeReason::UNAVAILABLE);
    } else {
        AP_Notify::events.failsafe_mode_change = 1;
    }

    // flag that initialisation has completed
    ap.initialised = true;
}


// return MAV_TYPE corresponding to frame class
MAV_TYPE Blimp::get_frame_mav_type()
{
    return MAV_TYPE_AIRSHIP;
}

// return string corresponding to frame_class
const char* Blimp::get_frame_string()
{
    return "MIXED";
}

/*
  allocate the motors class
 */
void Blimp::allocate_motors(void)
{
    motors = NEW_NOTHROW AP_MotorsBlimp(blimp.scheduler.get_loop_rate_hz());
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("Failed to allocate AP_MotorsBlimp");
    }
    AP_Param::load_object_from_eeprom(motors, AP_MotorsBlimp::var_info);

    AP_Param::reload_defaults_file(true);
    AP_Param::invalidate_count();
}
