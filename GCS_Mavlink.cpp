#include "Blimp.h"

#include "GCS_Mavlink.h"

MAV_TYPE GCS_Blimp::frame_type() const
{
    return blimp.get_frame_mav_type();
}

MAV_MODE GCS_MAVLINK_Blimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (blimp.motors != nullptr && blimp.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Blimp::custom_mode() const
{
    return (uint32_t)blimp.control_mode;
}

MAV_STATE GCS_MAVLINK_Blimp::vehicle_system_status() const
{
    if (blimp.any_failsafe_triggered()) return MAV_STATE_CRITICAL;
    if (blimp.ap.land_complete) return MAV_STATE_STANDBY;
    if (!blimp.ap.initialised) return MAV_STATE_BOOT;
    return MAV_STATE_ACTIVE;
}

void GCS_MAVLINK_Blimp::send_position_target_global_int()
{
    Location target;
    if (!blimp.flightmode->get_wp(target)) return;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | 0xF000;
    mavlink_msg_position_target_global_int_send(chan, AP_HAL::millis(), MAV_FRAME_GLOBAL, TYPE_MASK,
        target.lat, target.lng, target.alt * 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void GCS_MAVLINK_Blimp::send_nav_controller_output() const {}
float GCS_MAVLINK_Blimp::vfr_hud_airspeed() const { return 0.0f; }
int16_t GCS_MAVLINK_Blimp::vfr_hud_throttle() const { return blimp.motors ? (int16_t)(blimp.motors->get_throttle() * 100) : 0; }
void GCS_MAVLINK_Blimp::send_pid_tuning() {}
uint8_t GCS_MAVLINK_Blimp::sysid_my_gcs() const { return blimp.g.sysid_my_gcs; }
bool GCS_MAVLINK_Blimp::sysid_enforce() const { return blimp.g2.sysid_enforce; }
uint32_t GCS_MAVLINK_Blimp::telem_delay() const { return (uint32_t)(blimp.g.telem_delay); }
bool GCS_Blimp::vehicle_initialised() const { return blimp.ap.initialised; }

bool GCS_MAVLINK_Blimp::try_send_message(enum ap_message id)
{
    switch (id) {
    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
    case MSG_ADSB_VEHICLE:
        break;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0], 0),
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1], 0),
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2], 0),
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3], 0),
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4], 0),
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5], 0),
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6], 0),
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7], 0),
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8], 0),
    AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = { MSG_RAW_IMU };
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = { MSG_SYS_STATUS, MSG_POSITION_TARGET_GLOBAL_INT };
static const ap_message STREAM_POSITION_msgs[] = { MSG_LOCATION, MSG_LOCAL_POSITION };
static const ap_message STREAM_RC_CHANNELS_msgs[] = { MSG_SERVO_OUTPUT_RAW, MSG_RC_CHANNELS };
static const ap_message STREAM_EXTRA1_msgs[] = { MSG_ATTITUDE };
static const ap_message STREAM_EXTRA2_msgs[] = { MSG_VFR_HUD };
static const ap_message STREAM_EXTRA3_msgs[] = { MSG_SYSTEM_TIME };
static const ap_message STREAM_PARAMS_msgs[] = { MSG_NEXT_PARAM };
static const ap_message STREAM_ADSB_msgs[] = {};

// THIS IS REQUIRED - without it GCS base class doesn't know what messages to send
const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR
};

void GCS_MAVLINK_Blimp::packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg)
{
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Blimp::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) return true;
    return blimp.ap.initialised_params;
}

void GCS_MAVLINK_Blimp::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", blimp.get_frame_string());
}

MAV_RESULT GCS_MAVLINK_Blimp::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_do_set_roi(const Location &roi_loc) { return MAV_RESULT_ACCEPTED; }

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_NAV_TAKEOFF: return MAV_RESULT_ACCEPTED;
    default: return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
bool GCS_MAVLINK_Blimp::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    if (packet_command == MAV_CMD_NAV_TAKEOFF) { frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; return true; }
    return GCS_MAVLINK::mav_frame_for_command_long(frame, packet_command);
}
#endif

void GCS_MAVLINK_Blimp::handle_message(const mavlink_message_t &msg) { GCS_MAVLINK::handle_message(msg); }

MAV_RESULT GCS_MAVLINK_Blimp::handle_flight_termination(const mavlink_command_int_t &packet)
{
    if (packet.param1 > 0.5f) { blimp.arming.disarm(AP_Arming::Method::TERMINATION); return MAV_RESULT_ACCEPTED; }
    return MAV_RESULT_FAILED;
}

float GCS_MAVLINK_Blimp::vfr_hud_alt() const { return 0.0f; }

uint64_t GCS_MAVLINK_Blimp::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT | MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT | MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            GCS_MAVLINK::capabilities());
}

MAV_LANDED_STATE GCS_MAVLINK_Blimp::landed_state() const
{
    if (blimp.ap.land_complete) return MAV_LANDED_STATE_ON_GROUND;
    return MAV_LANDED_STATE_IN_AIR;
}

void GCS_MAVLINK_Blimp::send_wind() const {}

#if HAL_HIGH_LATENCY2_ENABLED
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_speed() const { return 0; }
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_direction() const { return 0; }
#endif
