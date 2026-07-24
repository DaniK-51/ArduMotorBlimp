#include "Blimp.h"

bool Blimp::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

void Blimp::failsafe_radio_on_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    Failsafe_Action desired_action;
    switch (g.failsafe_throttle) {
    case FS_THR_DISABLED:
        desired_action = Failsafe_Action_None;
        break;
    default:
        desired_action = Failsafe_Action_BRAKE;
    }

    if (should_disarm_on_failsafe()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = Failsafe_Action_None;
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe");
    }

    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

void Blimp::failsafe_radio_off_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

void Blimp::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    Failsafe_Action desired_action = Failsafe_Action_BRAKE;

    if (should_disarm_on_failsafe()) {
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = Failsafe_Action_None;
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe - Disarming");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe");
    }

    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);
}

void Blimp::failsafe_gcs_check()
{
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        return;
    }

    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        set_failsafe_gcs(true);
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe");
        do_failsafe_action(Failsafe_Action_BRAKE, ModeReason::GCS_FAILSAFE);
    } else if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        set_failsafe_gcs(false);
    }
}

bool Blimp::should_disarm_on_failsafe()
{
    if (ap.in_arming_delay) {
        return true;
    }
    return !motors->armed();
}

void Blimp::do_failsafe_action(Failsafe_Action action, ModeReason reason)
{
    switch (action) {
    case Failsafe_Action_None:
        return;
    case Failsafe_Action_BRAKE:
        set_mode_brake_failsafe(reason);
        break;
    case Failsafe_Action_Terminate:
        arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
        break;
    }
}
