#include "GCS_MAVLink_MotorBlimp.h"
#include "ArduMotorBlimp.h"

extern const AP_HAL::HAL& hal;

namespace {

constexpr uint16_t POS_IGNORE_MASK =
    POSITION_TARGET_TYPEMASK_X_IGNORE |
    POSITION_TARGET_TYPEMASK_Y_IGNORE |
    POSITION_TARGET_TYPEMASK_Z_IGNORE;
constexpr uint16_t VEL_IGNORE_MASK =
    POSITION_TARGET_TYPEMASK_VX_IGNORE |
    POSITION_TARGET_TYPEMASK_VY_IGNORE |
    POSITION_TARGET_TYPEMASK_VZ_IGNORE;
constexpr uint16_t ACC_IGNORE_MASK =
    POSITION_TARGET_TYPEMASK_AX_IGNORE |
    POSITION_TARGET_TYPEMASK_AY_IGNORE |
    POSITION_TARGET_TYPEMASK_AZ_IGNORE;
constexpr uint16_t YAW_IGNORE_MASK = POSITION_TARGET_TYPEMASK_YAW_IGNORE;
constexpr uint16_t YAW_RATE_IGNORE_MASK = POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
constexpr uint16_t FORCE_SET_MASK = POSITION_TARGET_TYPEMASK_FORCE_SET;

} // namespace

uint8_t GCS_MAVLINK_MotorBlimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    switch ((Mode)motorblimp.get_mode()) {
    case Mode::MANUAL:
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;

    case Mode::HOLD:
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;

    case Mode::AUTO:
    case Mode::GUIDED:
        // ArduPilot deliberately uses GUIDED_ENABLED for modes which
        // follow an externally supplied or mission-supplied target.
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
                      MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    }

    if (motorblimp.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return _base_mode;
}

MAV_STATE GCS_MAVLINK_MotorBlimp::vehicle_system_status() const
{
    if (!hal.scheduler->is_system_initialized()) {
        return MAV_STATE_BOOT;
    }

    if (AP::ins().calibrating()) {
        return MAV_STATE_CALIBRATING;
    }

    if (!motorblimp.arming.is_armed()) {
        return MAV_STATE_STANDBY;
    }

    const Mode mode = (Mode)motorblimp.get_mode();
    const AP_AHRS &ahrs = AP::ahrs();
    const bool attitude_ok = ahrs.initialised() && ahrs.healthy() &&
                             ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID);
    const bool yaw_ok = attitude_ok && motorblimp.compass_healthy();
    bool critical = AP::battery().has_failsafed();
    switch (mode) {
    case Mode::MANUAL:
        critical |= rc().in_rc_failsafe() || !attitude_ok || !yaw_ok;
        break;
    case Mode::HOLD:
        critical |= !attitude_ok || !yaw_ok;
        break;
    case Mode::AUTO:
    case Mode::GUIDED:
        // Use the same one-tag freshness predicate as the control loop.  EKF
        // position flags can remain valid briefly while dead reckoning, which
        // must not be reported as an ACTIVE autonomous vehicle after the
        // only UWB fix has timed out.
        critical |= !motorblimp.navigation_healthy();
        break;
    }

    if (critical) {
        return MAV_STATE_CRITICAL;
    }

    return MAV_STATE_ACTIVE;
}

void GCS_MAVLINK_MotorBlimp::send_nav_controller_output() const
{
}

void GCS_MAVLINK_MotorBlimp::send_pid_tuning()
{
}

uint8_t GCS_MAVLINK_MotorBlimp::send_available_mode(uint8_t index) const
{
    struct ModeInfo {
        Mode number;
        const char *name;
    };

    static const ModeInfo modes[] {
        { Mode::MANUAL, "MANUAL" },
        { Mode::HOLD,   "HOLD" },
        { Mode::AUTO,   "AUTO" },
        { Mode::GUIDED, "GUIDED" },
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);
    if (index == 0 || index > mode_count) {
        return mode_count;
    }

    const ModeInfo &mode = modes[index - 1];
    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        (uint32_t)mode.number,
        0, // MAV_MODE_PROPERTY bitmask: all four modes are user-selectable
        mode.name);

    return mode_count;
}

uint64_t GCS_MAVLINK_MotorBlimp::capabilities() const
{
    return MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
           MAV_PROTOCOL_CAPABILITY_MISSION_INT |
           MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
           MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
           GCS_MAVLINK::capabilities();
}

bool GCS_MAVLINK_MotorBlimp::try_send_message(const enum ap_message id)
{
    switch (id) {
    case MSG_WIND:
        // MotorBlimp has no wind estimator.  WIND is nevertheless part of
        // the generic EXTRA3 stream requested by MAVProxy; consume the
        // request so SITL does not treat it as an unknown vehicle message.
        return true;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
}

void GCS_MAVLINK_MotorBlimp::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

void GCS_MAVLINK_MotorBlimp::handle_set_position_target_local_ned(const mavlink_message_t &msg)
{
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // The initial GUIDED contract is intentionally small and deterministic:
    // one persistent position target in EKF-origin-relative local NED.
    if ((Mode)motorblimp.get_mode() != Mode::GUIDED ||
        packet.coordinate_frame != MAV_FRAME_LOCAL_NED) {
        return;
    }

    static constexpr uint16_t required_ignore_mask =
        VEL_IGNORE_MASK |
        ACC_IGNORE_MASK |
        YAW_IGNORE_MASK |
        YAW_RATE_IGNORE_MASK;

    if ((packet.type_mask & POS_IGNORE_MASK) != 0 ||
        (packet.type_mask & required_ignore_mask) != required_ignore_mask ||
        (packet.type_mask & FORCE_SET_MASK) != 0) {
        return;
    }

    const Vector3f target_ned {packet.x, packet.y, packet.z};
    if (target_ned.is_nan() || target_ned.is_inf()) {
        return;
    }

    if (!motorblimp.set_guided_target_ned(target_ned)) {
        send_text(MAV_SEVERITY_WARNING, "GUIDED target rejected");
    }
}
