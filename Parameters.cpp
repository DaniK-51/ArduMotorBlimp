#include "ArduMotorBlimp.h"

#include <SRV_Channel/SRV_Channel.h>

const AP_Param::Info ArduMotorBlimp::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Onboard log bitmask
    // @Description: Selects attitude, scheduler, RC, IMU, battery, PID and compass records written to the onboard log
    // @Bitmask: 1:Medium Attitude,3:System Performance,6:RC Input,7:IMU,9:Battery Monitor,10:RC Output,12:PID,13:Compass
    // @User: Standard
    GSCALAR(log_bitmask, "LOG_BITMASK", DEFAULT_LOG_BITMASK),

    // Library objects
    GOBJECT(barometer, "BARO", AP_Baro),
    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(ins, "INS", AP_InertialSensor),
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
    GOBJECT(gps, "GPS", AP_GPS),
    GOBJECT(battery, "BATT", AP_BattMonitor),
    GOBJECT(notify, "NTF_", AP_Notify),
    GOBJECT(rc_channels, "RC", RC_Channels_MotorBlimp),
    GOBJECT(rcmap, "RCMAP_", RCMapper),
    GOBJECT(servo_channels, "SERVO", SRV_Channels),
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // EKF
#if HAL_NAVEKF2_AVAILABLE
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),

    // GCS
    GOBJECT(_gcs, "MAV", GCS),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming, "ARMING_", AP_Arming_MotorBlimp),

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_", AP_Mission),

    // @Group: BCN_
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    // AP_Beacon leaf names begin with an underscore (for example _TYPE), so
    // the object prefix deliberately has no trailing underscore.
    GOBJECT(beacon, "BCN", AP_Beacon),

    // Motor mixer and flight-control parameters.  The individual group names
    // carry their own MIX_, ATC_, MAN_, PSC_, NAV_ and WP_ prefixes.
    GOBJECT(g2, "", ParametersG2),

    // AP_Vehicle common parameters include the serial manager required to
    // configure a real Nooploop tag (SERIALx_PROTOCOL=13), logging, statistics
    // and other vehicle-wide services.
    PARAM_VEHICLE_INFO,

    AP_VAREND
};

const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // @Param: MIX_M1_FWD
    // @DisplayName: Motor 1 forward coefficient
    // @Description: Mixing coefficient for motor 1 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M1_FWD", 1, ParametersG2, mix_m1_fwd, 1.0f),

    // @Param: MIX_M1_RLL
    // @DisplayName: Motor 1 roll coefficient
    // @Description: Mixing coefficient for motor 1 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M1_RLL", 2, ParametersG2, mix_m1_roll, 1.0f),

    // @Param: MIX_M1_PIT
    // @DisplayName: Motor 1 pitch coefficient
    // @Description: Mixing coefficient for motor 1 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M1_PIT", 3, ParametersG2, mix_m1_pitch, -1.0f),

    // @Param: MIX_M1_YAW
    // @DisplayName: Motor 1 yaw coefficient
    // @Description: Mixing coefficient for motor 1 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M1_YAW", 4, ParametersG2, mix_m1_yaw, 0.0f),

    // @Param: MIX_M2_FWD
    // @DisplayName: Motor 2 forward coefficient
    // @Description: Mixing coefficient for motor 2 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M2_FWD", 5, ParametersG2, mix_m2_fwd, 1.0f),

    // @Param: MIX_M2_RLL
    // @DisplayName: Motor 2 roll coefficient
    // @Description: Mixing coefficient for motor 2 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M2_RLL", 6, ParametersG2, mix_m2_roll, -1.0f),

    // @Param: MIX_M2_PIT
    // @DisplayName: Motor 2 pitch coefficient
    // @Description: Mixing coefficient for motor 2 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M2_PIT", 7, ParametersG2, mix_m2_pitch, 0.0f),

    // @Param: MIX_M2_YAW
    // @DisplayName: Motor 2 yaw coefficient
    // @Description: Mixing coefficient for motor 2 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M2_YAW", 8, ParametersG2, mix_m2_yaw, -1.0f),

    // @Param: MIX_M3_FWD
    // @DisplayName: Motor 3 forward coefficient
    // @Description: Mixing coefficient for motor 3 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M3_FWD", 9, ParametersG2, mix_m3_fwd, 1.0f),

    // @Param: MIX_M3_RLL
    // @DisplayName: Motor 3 roll coefficient
    // @Description: Mixing coefficient for motor 3 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M3_RLL", 10, ParametersG2, mix_m3_roll, 1.0f),

    // @Param: MIX_M3_PIT
    // @DisplayName: Motor 3 pitch coefficient
    // @Description: Mixing coefficient for motor 3 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M3_PIT", 11, ParametersG2, mix_m3_pitch, 1.0f),

    // @Param: MIX_M3_YAW
    // @DisplayName: Motor 3 yaw coefficient
    // @Description: Mixing coefficient for motor 3 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M3_YAW", 12, ParametersG2, mix_m3_yaw, 0.0f),

    // @Param: MIX_M4_FWD
    // @DisplayName: Motor 4 forward coefficient
    // @Description: Mixing coefficient for motor 4 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M4_FWD", 13, ParametersG2, mix_m4_fwd, 1.0f),

    // @Param: MIX_M4_RLL
    // @DisplayName: Motor 4 roll coefficient
    // @Description: Mixing coefficient for motor 4 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M4_RLL", 14, ParametersG2, mix_m4_roll, -1.0f),

    // @Param: MIX_M4_PIT
    // @DisplayName: Motor 4 pitch coefficient
    // @Description: Mixing coefficient for motor 4 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M4_PIT", 15, ParametersG2, mix_m4_pitch, 0.0f),

    // @Param: MIX_M4_YAW
    // @DisplayName: Motor 4 yaw coefficient
    // @Description: Mixing coefficient for motor 4 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("MIX_M4_YAW", 16, ParametersG2, mix_m4_yaw, 1.0f),

    // @Param: ATC_ANG_RLL_P
    // @DisplayName: Roll angle P gain
    // @Description: Converts quaternion roll error to a body roll-rate target
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ATC_ANG_RLL_P", 17, ParametersG2, att_angle_roll_p, 2.5f),

    // @Param: ATC_ANG_PIT_P
    // @DisplayName: Pitch angle P gain
    // @Description: Converts quaternion pitch error to a body pitch-rate target
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ATC_ANG_PIT_P", 18, ParametersG2, att_angle_pitch_p, 2.5f),

    // @Param: ATC_ANG_YAW_P
    // @DisplayName: Yaw angle P gain
    // @Description: Converts quaternion yaw error to a body yaw-rate target
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ATC_ANG_YAW_P", 19, ParametersG2, att_angle_yaw_p, 2.0f),

    // @Param: ATC_RAT_RLL_MAX
    // @DisplayName: Maximum roll rate
    // @Description: Maximum body roll-rate target generated by the attitude controller
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard
    AP_GROUPINFO("ATC_RAT_RLL_MAX", 20, ParametersG2, att_rate_roll_max_dps, 60.0f),

    // @Param: ATC_RAT_PIT_MAX
    // @DisplayName: Maximum pitch rate
    // @Description: Maximum body pitch-rate target generated by the attitude controller
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard
    AP_GROUPINFO("ATC_RAT_PIT_MAX", 21, ParametersG2, att_rate_pitch_max_dps, 60.0f),

    // @Param: ATC_RAT_YAW_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: Maximum body yaw-rate target generated by the attitude controller
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard
    AP_GROUPINFO("ATC_RAT_YAW_MAX", 22, ParametersG2, att_rate_yaw_max_dps, 45.0f),

    // @Param: MAN_RLL_MAX
    // @DisplayName: Manual maximum roll angle
    // @Description: Maximum absolute roll-angle target commanded by the manual roll stick
    // @Units: deg
    // @Range: 0 85
    // @User: Standard
    AP_GROUPINFO("MAN_RLL_MAX", 23, ParametersG2, manual_roll_max_deg, 30.0f),

    // @Param: MAN_PIT_MAX
    // @DisplayName: Manual maximum pitch angle
    // @Description: Maximum absolute pitch-angle target commanded by the manual pitch stick
    // @Units: deg
    // @Range: 0 85
    // @User: Standard
    AP_GROUPINFO("MAN_PIT_MAX", 24, ParametersG2, manual_pitch_max_deg, 60.0f),

    // @Param: MAN_YAW_RATE
    // @DisplayName: Manual maximum yaw rate
    // @Description: Maximum body yaw-rate commanded directly by the manual yaw stick
    // @Units: deg/s
    // @Range: 0 180
    // @User: Standard
    AP_GROUPINFO("MAN_YAW_RATE", 25, ParametersG2, manual_yaw_rate_max_dps, 45.0f),

    // @Param: MAN_FWD_MAX
    // @DisplayName: Manual maximum signed collective
    // @Description: Maximum magnitude of the signed forward or reverse collective command in manual mode
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("MAN_FWD_MAX", 26, ParametersG2, manual_collective_max, 1.0f),

    // @Param: PSC_POS_P
    // @DisplayName: Position P gain
    // @Description: Converts NED position error to a velocity target
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PSC_POS_P", 27, ParametersG2, position_p, 0.4f),

    // @Param: PSC_VEL_P
    // @DisplayName: Velocity P gain
    // @Description: Converts NED velocity error to an acceleration request
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PSC_VEL_P", 28, ParametersG2, velocity_p, 1.0f),

    // @Param: PSC_VEL_I
    // @DisplayName: Velocity I gain
    // @Description: Compensates steady wind and non-neutral buoyancy
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("PSC_VEL_I", 29, ParametersG2, velocity_i, 0.1f),

    // @Param: PSC_VEL_IMAX
    // @DisplayName: Velocity integrator maximum
    // @Description: Maximum magnitude of the three-dimensional velocity-controller integrator
    // @Units: m/s/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("PSC_VEL_IMAX", 30, ParametersG2, velocity_imax, 0.2f),

    // @Param: NAV_VEL_MAX
    // @DisplayName: Maximum navigation velocity
    // @Description: Maximum magnitude of the three-dimensional automatic velocity target
    // @Units: m/s
    // @Range: 0.05 5
    // @User: Standard
    AP_GROUPINFO("NAV_VEL_MAX", 31, ParametersG2, nav_velocity_max_mps, 0.5f),

    // @Param: NAV_ACC_MAX
    // @DisplayName: Maximum navigation acceleration
    // @Description: Maximum magnitude of the three-dimensional automatic acceleration request
    // @Units: m/s/s
    // @Range: 0.01 5
    // @User: Standard
    AP_GROUPINFO("NAV_ACC_MAX", 32, ParametersG2, nav_accel_max_mss, 0.3f),

    // @Param: NAV_FWD_MAX
    // @DisplayName: Maximum automatic signed collective
    // @Description: Maximum magnitude of the signed forward or reverse collective command in automatic flight
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("NAV_FWD_MAX", 33, ParametersG2, nav_collective_max, 0.7f),

    // @Param: NAV_PIT_MAX
    // @DisplayName: Maximum navigation pitch angle
    // @Description: Maximum absolute pitch angle used to point the body axis at a navigation acceleration request
    // @Units: deg
    // @Range: 0 85
    // @User: Standard
    AP_GROUPINFO("NAV_PIT_MAX", 34, ParametersG2, nav_pitch_max_deg, 60.0f),

    // @Param: NAV_ACC_MIN
    // @DisplayName: Direction update acceleration threshold
    // @Description: Below this acceleration request the last attitude target is retained
    // @Units: m/s/s
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("NAV_ACC_MIN", 35, ParametersG2, nav_accel_min_mss, 0.02f),

    // @Param: NAV_REV_HYST
    // @DisplayName: Forward reverse selection hysteresis
    // @Description: Prevents direction chatter when choosing forward or reverse thrust
    // @Range: 0 0.9
    // @User: Advanced
    AP_GROUPINFO("NAV_REV_HYST", 36, ParametersG2, nav_reverse_hysteresis, 0.15f),

    // @Param: NAV_THR_ANGLE
    // @DisplayName: Full thrust attitude error angle
    // @Description: Automatic collective is zero above this pointing error and ramps to full at zero error
    // @Units: deg
    // @Range: 1 90
    // @User: Advanced
    AP_GROUPINFO("NAV_THR_ANGLE", 37, ParametersG2, nav_thrust_angle_deg, 30.0f),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint radius
    // @Description: Maximum three-dimensional position error for waypoint acceptance
    // @Units: m
    // @Range: 0.05 5
    // @User: Standard
    AP_GROUPINFO("WP_RADIUS", 38, ParametersG2, waypoint_radius_m, 0.4f),

    // @Param: WP_SPEED
    // @DisplayName: Waypoint speed tolerance
    // @Description: Maximum vehicle speed for waypoint acceptance
    // @Units: m/s
    // @Range: 0.01 2
    // @User: Standard
    AP_GROUPINFO("WP_SPEED", 39, ParametersG2, waypoint_speed_mps, 0.15f),

    // @Group: ATC_RAT_RLL_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(rate_roll_pid, "ATC_RAT_RLL_", 40, ParametersG2, AC_PID),

    // @Group: ATC_RAT_PIT_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(rate_pitch_pid, "ATC_RAT_PIT_", 41, ParametersG2, AC_PID),

    // @Group: ATC_RAT_YAW_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(rate_yaw_pid, "ATC_RAT_YAW_", 42, ParametersG2, AC_PID),

    // @Param: UWB_ERR_MAX
    // @DisplayName: Maximum UWB position error
    // @Description: AUTO and GUIDED stop at neutral when the direct UWB tag position reports a larger one-sigma error
    // @Units: m
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("UWB_ERR_MAX", 43, ParametersG2, uwb_accuracy_max_m, 1.0f),

    // @Param: MIX_DEADBAND
    // @DisplayName: Motor output deadband
    // @Description: Motor commands smaller than this fraction of full thrust are output as exact neutral (1500 us). A reversible DShot ESC treats only exactly 1500 as stopped, so without a deadband the attitude loops keep the motors idling around neutral. Keep it well below MAN_FWD_MAX, otherwise small stick and attitude corrections are lost.
    // @Range: 0 0.3
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIX_DEADBAND", 44, ParametersG2, mix_deadband, 0.02f),

    AP_GROUPEND
};

void ParametersG2::set_default_motor_functions()
{
    SRV_Channels::set_default_function(0, SRV_Channel::k_motor1);
    SRV_Channels::set_default_function(1, SRV_Channel::k_motor2);
    SRV_Channels::set_default_function(2, SRV_Channel::k_motor3);
    SRV_Channels::set_default_function(3, SRV_Channel::k_motor4);
}
