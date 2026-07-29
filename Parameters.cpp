#include "ArduMotorBlimp.h"

const AP_Param::Info ArduMotorBlimp::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // Library objects
    GOBJECT(barometer, "BARO", AP_Baro),
    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(ins, "INS", AP_InertialSensor),
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
    GOBJECT(gps, "GPS", AP_GPS),
    GOBJECT(battery, "BATT", AP_BattMonitor),
    GOBJECT(notify, "NTF_", AP_Notify),
    GOBJECT(rc_channels, "RC", RC_Channels_MotorBlimp),
    GOBJECT(servo_channels, "SERVO", SRV_Channels),
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

    // EKF
    GOBJECTN(ahrs.ekf2.EKF2, NavEKF2, "EK2_", NavEKF2),
    GOBJECTN(ahrs.ekf3.EKF3, NavEKF3, "EK3_", NavEKF3),

    // GCS
    GOBJECT(_gcs, "MAV", GCS),

    // Motor mixing parameters
    GOBJECT(g2, "MIX_", ParametersG2),

    AP_VAREND
};

const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // @Param: M1_FWD
    // @DisplayName: Motor 1 forward coefficient
    // @Description: Mixing coefficient for motor 1 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M1_FWD", 1, ParametersG2, mix_m1_fwd, 0.0f),

    // @Param: M1_RLL
    // @DisplayName: Motor 1 roll coefficient
    // @Description: Mixing coefficient for motor 1 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M1_RLL", 2, ParametersG2, mix_m1_roll, 0.0f),

    // @Param: M1_PIT
    // @DisplayName: Motor 1 pitch coefficient
    // @Description: Mixing coefficient for motor 1 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M1_PIT", 3, ParametersG2, mix_m1_pitch, 0.0f),

    // @Param: M1_YAW
    // @DisplayName: Motor 1 yaw coefficient
    // @Description: Mixing coefficient for motor 1 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M1_YAW", 4, ParametersG2, mix_m1_yaw, 0.0f),

    // @Param: M2_FWD
    // @DisplayName: Motor 2 forward coefficient
    // @Description: Mixing coefficient for motor 2 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M2_FWD", 5, ParametersG2, mix_m2_fwd, 0.0f),

    // @Param: M2_RLL
    // @DisplayName: Motor 2 roll coefficient
    // @Description: Mixing coefficient for motor 2 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M2_RLL", 6, ParametersG2, mix_m2_roll, 0.0f),

    // @Param: M2_PIT
    // @DisplayName: Motor 2 pitch coefficient
    // @Description: Mixing coefficient for motor 2 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M2_PIT", 7, ParametersG2, mix_m2_pitch, 0.0f),

    // @Param: M2_YAW
    // @DisplayName: Motor 2 yaw coefficient
    // @Description: Mixing coefficient for motor 2 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M2_YAW", 8, ParametersG2, mix_m2_yaw, 0.0f),

    // @Param: M3_FWD
    // @DisplayName: Motor 3 forward coefficient
    // @Description: Mixing coefficient for motor 3 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M3_FWD", 9, ParametersG2, mix_m3_fwd, 0.0f),

    // @Param: M3_RLL
    // @DisplayName: Motor 3 roll coefficient
    // @Description: Mixing coefficient for motor 3 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M3_RLL", 10, ParametersG2, mix_m3_roll, 0.0f),

    // @Param: M3_PIT
    // @DisplayName: Motor 3 pitch coefficient
    // @Description: Mixing coefficient for motor 3 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M3_PIT", 11, ParametersG2, mix_m3_pitch, 0.0f),

    // @Param: M3_YAW
    // @DisplayName: Motor 3 yaw coefficient
    // @Description: Mixing coefficient for motor 3 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M3_YAW", 12, ParametersG2, mix_m3_yaw, 0.0f),

    // @Param: M4_FWD
    // @DisplayName: Motor 4 forward coefficient
    // @Description: Mixing coefficient for motor 4 forward contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M4_FWD", 13, ParametersG2, mix_m4_fwd, 0.0f),

    // @Param: M4_RLL
    // @DisplayName: Motor 4 roll coefficient
    // @Description: Mixing coefficient for motor 4 roll contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M4_RLL", 14, ParametersG2, mix_m4_roll, 0.0f),

    // @Param: M4_PIT
    // @DisplayName: Motor 4 pitch coefficient
    // @Description: Mixing coefficient for motor 4 pitch contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M4_PIT", 15, ParametersG2, mix_m4_pitch, 0.0f),

    // @Param: M4_YAW
    // @DisplayName: Motor 4 yaw coefficient
    // @Description: Mixing coefficient for motor 4 yaw contribution
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("M4_YAW", 16, ParametersG2, mix_m4_yaw, 0.0f),

    AP_GROUPEND
};
