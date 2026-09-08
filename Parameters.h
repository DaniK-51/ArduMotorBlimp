#pragma once

#define AP_PARAM_VEHICLE_NAME motorblimp

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>

class Parameters {
public:
    // Version 2 moves the former k_param_g2=15 block and introduces arming,
    // mission, beacon, SITL, RC mapping, logging and AP_Vehicle groups.  Treat
    // a version-1 EEPROM as incompatible so old mixer bytes can never be
    // interpreted as arming policy.
    static const uint16_t k_format_version = 2;

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
        k_param_arming = 15,
        k_param_mission = 16,
        k_param_beacon = 17,
        k_param_g2 = 18,
        k_param_sitl = 19,
        k_param_rcmap = 20,
        k_param_log_bitmask = 21,

        // Reserved by ArduPilot for the AP_Vehicle common parameter block.
        // Keep this fixed key so SERIALx_, LOG_, STAT_ and the other common
        // groups use the same storage layout as the upstream vehicles.
        k_param_vehicle = 257,
    };

    AP_Int16 format_version;
    AP_Int32 log_bitmask;
};

class ParametersG2 {
public:
    static const AP_Param::GroupInfo var_info[];

    ParametersG2()
    {
        // Unlike most ArduPilot library parameter objects, ParametersG2 is a
        // vehicle-local group.  It therefore has to initialise its own leaf
        // defaults; registering the group in the top-level var_info table is
        // not sufficient to populate unset AP_Float values.
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Must be called after the SRV_Channels object has been constructed.
    // ParametersG2 is constructed before SRV_Channels in ArduMotorBlimp, so
    // calling this from a ParametersG2 constructor would dereference the
    // uninitialised SRV_Channels singleton.
    static void set_default_motor_functions();

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

    // Quaternion attitude-error P gains.  Their output is a body-rate target
    // in rad/s.
    AP_Float att_angle_roll_p;
    AP_Float att_angle_pitch_p;
    AP_Float att_angle_yaw_p;

    // Body-rate limits in degrees/s.
    AP_Float att_rate_roll_max_dps;
    AP_Float att_rate_pitch_max_dps;
    AP_Float att_rate_yaw_max_dps;

    // Stabilised MANUAL-mode limits.
    AP_Float manual_roll_max_deg;
    AP_Float manual_pitch_max_deg;
    AP_Float manual_yaw_rate_max_dps;
    AP_Float manual_collective_max;

    // Position and velocity controller gains.
    AP_Float position_p;
    AP_Float velocity_p;
    AP_Float velocity_i;
    AP_Float velocity_imax;

    // Guidance limits.
    AP_Float nav_velocity_max_mps;
    AP_Float nav_accel_max_mss;
    AP_Float nav_collective_max;
    AP_Float nav_pitch_max_deg;
    AP_Float nav_accel_min_mss;
    AP_Float nav_reverse_hysteresis;
    AP_Float nav_thrust_angle_deg;

    // Waypoint acceptance limits.
    AP_Float waypoint_radius_m;
    AP_Float waypoint_speed_mps;

    // Maximum accepted direct UWB position error for autonomous flight.
    AP_Float uwb_accuracy_max_m;
    AP_Float mix_deadband;
    AP_Float mix_output_max;

    // Body-rate PID controllers.  Outputs are normalised torque requests.
    AC_PID rate_roll_pid{0.25f, 0.05f, 0.003f, 0.0f, 0.30f, 10.0f, 10.0f, 5.0f};
    AC_PID rate_pitch_pid{0.25f, 0.05f, 0.003f, 0.0f, 0.30f, 10.0f, 10.0f, 5.0f};
    AC_PID rate_yaw_pid{0.25f, 0.05f, 0.003f, 0.0f, 0.30f, 10.0f, 10.0f, 5.0f};
};
