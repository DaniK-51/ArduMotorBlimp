#pragma once

#include <AP_Math/AP_Math.h>

#include "Parameters.h"

// Shared stabilisation and point-to-point guidance for MANUAL, GUIDED and
// AUTO.  All attitudes are body-to-NED quaternions and angular rates are body
// rates in rad/s.
class FlightControl {
public:
    struct ManualInput {
        float collective = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;

        constexpr ManualInput(float collective_in = 0.0f,
                              float roll_in = 0.0f,
                              float pitch_in = 0.0f,
                              float yaw_in = 0.0f) :
            collective(collective_in),
            roll(roll_in),
            pitch(pitch_in),
            yaw(yaw_in)
        {}
    };

    struct AttitudeTarget {
        Quaternion attitude_body_to_ned;
        float collective = 0.0f;
        bool valid = false;
    };

    struct AttitudeControlOutput {
        Vector3f attitude_error_rad;
        Vector3f rate_target_rads;
        Vector3f torque;
        float collective = 0.0f;
        bool rate_limited = false;
        bool valid = false;
    };

    struct NavigationState {
        Vector3f position_ned_m;
        Vector3f velocity_ned_mps;
        Quaternion attitude_body_to_ned;
        bool position_valid = false;
        bool velocity_valid = false;
        bool attitude_valid = false;

        NavigationState(const Vector3f &position_ned_m_in = Vector3f {},
                        const Vector3f &velocity_ned_mps_in = Vector3f {},
                        const Quaternion &attitude_body_to_ned_in = Quaternion {},
                        bool position_valid_in = false,
                        bool velocity_valid_in = false,
                        bool attitude_valid_in = false) :
            position_ned_m(position_ned_m_in),
            velocity_ned_mps(velocity_ned_mps_in),
            attitude_body_to_ned(attitude_body_to_ned_in),
            position_valid(position_valid_in),
            velocity_valid(velocity_valid_in),
            attitude_valid(attitude_valid_in)
        {}
    };

    struct GuidanceOutput {
        AttitudeTarget attitude;
        Vector3f position_error_m;
        Vector3f velocity_target_mps;
        Vector3f velocity_error_mps;
        Vector3f acceleration_request_mss;
        Vector3f axis_target_ned;
        int8_t thrust_sign = 1;
        bool acceleration_limited = false;
        bool pitch_limited = false;
        bool waypoint_reached = false;
        bool valid = false;
    };

    explicit FlightControl(ParametersG2 &params) : _params(params) {}

    // Reset all target state and controller history to the measured attitude.
    void reset(const Quaternion &current_attitude_body_to_ned);
    void reset_rate_controllers();
    void reset_navigation(const Quaternion &current_attitude_body_to_ned,
                          bool clear_position_target = false);

    // Stabilised manual target: roll/pitch sticks command angles, yaw commands
    // a rate integrated into an absolute heading, and collective remains signed.
    bool build_manual_target(const ManualInput &input,
                             const Quaternion &current_attitude_body_to_ned,
                             float dt,
                             AttitudeTarget &target);

    // Quaternion outer loop followed by three configurable AC_PID rate loops.
    // allocator_attitude_limited should be the previous mixer cycle's attitude
    // saturation flag and is used to stop PID integrator growth.
    AttitudeControlOutput update_attitude(
        const AttitudeTarget &target,
        const Quaternion &current_attitude_body_to_ned,
        const Vector3f &gyro_body_rads,
        float dt,
        bool allocator_attitude_limited = false);

    // GUIDED/AUTO latch a waypoint with set_position_target() and call
    // update_guidance() at the control-loop rate.
    bool set_position_target(const Vector3f &position_ned_m);
    void clear_position_target();
    bool has_position_target() const { return _position_target_valid; }
    const Vector3f &position_target() const { return _position_target_ned_m; }
    GuidanceOutput update_guidance(const NavigationState &state, float dt);

    // Pure helpers intended for unit tests.
    static bool vector_is_finite(const Vector3f &vector);
    static bool quaternion_is_valid(const Quaternion &quaternion);
    static Vector3f quaternion_error_body(
        const Quaternion &current_body_to_ned,
        const Quaternion &target_body_to_ned);
    static Vector3f limit_vector_length(const Vector3f &vector,
                                        float maximum_length,
                                        bool &limited);
    static bool waypoint_is_reached(const Vector3f &position_error_m,
                                    const Vector3f &velocity_ned_mps,
                                    float radius_m,
                                    float speed_mps);
    static bool attitude_from_axis_ned(const Vector3f &axis_ned,
                                       float pitch_limit_rad,
                                       float yaw_fallback_rad,
                                       Quaternion &attitude_body_to_ned,
                                       Vector3f &limited_axis_ned,
                                       bool &pitch_limited);
    static float thrust_alignment_scale(const Vector3f &current_axis_ned,
                                        const Vector3f &target_axis_ned,
                                        float full_error_angle_rad);

private:
    static bool manual_input_is_finite(const ManualInput &input);
    static Quaternion normalised_quaternion(const Quaternion &quaternion);

    ParametersG2 &_params;

    float _manual_yaw_target_rad = 0.0f;
    bool _manual_target_initialised = false;
    bool _last_rate_output_limited = false;

    Vector3f _position_target_ned_m;
    bool _position_target_valid = false;
    Vector3f _velocity_integrator_ned_mss;

    Quaternion _navigation_attitude_target;
    bool _navigation_attitude_valid = false;
    int8_t _navigation_thrust_sign = 1;
    bool _navigation_thrust_sign_initialised = false;
};
