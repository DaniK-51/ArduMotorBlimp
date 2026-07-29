#pragma once

#define AP_PARAM_VEHICLE_NAME motorblimp

#include <AP_Param/AP_Param.h>

class Parameters {
public:
    static const uint16_t k_format_version = 1;

    enum {
        k_param_format_version = 0,
        k_param_barometer = 1,
        k_param_compass = 2,
        k_param_ins = 3,
        k_param_ahrs = 4,
        k_param_gps = 5,
        k_param_battery = 6,
        k_param_notify = 7,
        k_param_rc_channels = 8,
        k_param_servo_channels = 9,
        k_param_scheduler = 10,
        k_param_BoardConfig = 11,
        k_param_NavEKF2 = 12,
        k_param_NavEKF3 = 13,
        k_param__gcs = 14,
        k_param_g2 = 15,
    };

    AP_Int16 format_version;
};

class ParametersG2 {
public:
    static const AP_Param::GroupInfo var_info[];

    // Motor mixing coefficients: motor[i] = sum(coeff[i][j] * axis[j])
    // 4 motors x 4 axes = 16 parameters
    AP_Float mix_m1_fwd;
    AP_Float mix_m1_roll;
    AP_Float mix_m1_pitch;
    AP_Float mix_m1_yaw;

    AP_Float mix_m2_fwd;
    AP_Float mix_m2_roll;
    AP_Float mix_m2_pitch;
    AP_Float mix_m2_yaw;

    AP_Float mix_m3_fwd;
    AP_Float mix_m3_roll;
    AP_Float mix_m3_pitch;
    AP_Float mix_m3_yaw;

    AP_Float mix_m4_fwd;
    AP_Float mix_m4_roll;
    AP_Float mix_m4_pitch;
    AP_Float mix_m4_yaw;
};
