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

    AP_VAREND
};
