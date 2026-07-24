#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// bit options for DEV_OPTIONS parameter
enum DevOptions {
    DevOptionADSBMAVLink = 1,
    DevOptionVFR_HUDRelativeAlt = 2,
    DevOptionSetAttitudeTarget_ThrustAsThrust = 4,
};

//  Logging parameters
enum LoggingParameters {
    LOG_CONTROL_TUNING_MSG,
    LOG_DATA_INT16_MSG,
    LOG_DATA_UINT16_MSG,
    LOG_DATA_INT32_MSG,
    LOG_DATA_UINT32_MSG,
    LOG_DATA_FLOAT_MSG,
    LOG_PARAMTUNE_MSG,
    LOG_MOTORI_MSG,
    LOG_MOTORO_MSG,
};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_RCIN                   (1<<6)
#define MASK_LOG_RCOUT                  (1<<10)
#define MASK_LOG_PID                    (1<<12)
#define MASK_LOG_ANY                    0xFFFF

// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                 0
#define FS_THR_ENABLED_ALWAYS_BRAKE     1

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED                 0
#define FS_GCS_ENABLED_ALWAYS_BRAKE     1

// EKF failsafe definitions (FS_EKF_ACTION parameter)
#define FS_EKF_ACTION_BRAKE             1
#define FS_EKF_ACTION_BRAKE_EVEN_MANUAL 3

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
