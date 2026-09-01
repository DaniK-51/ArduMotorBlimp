#include "FlightControl.h"

#include <cmath>

namespace {

constexpr float CONTROL_EPSILON = 1.0e-6f;
constexpr float MAX_CONTROL_DT = 0.1f;
constexpr float MAX_PITCH_RAD = radians(85.0f);

float bounded_dt(float dt)
{
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return 0.0f;
    }
    return MIN(dt, MAX_CONTROL_DT);
}

float bounded_unit_input(float input)
{
    return constrain_float(input, -1.0f, 1.0f);
}

} // namespace

bool FlightControl::vector_is_finite(const Vector3f &vector)
{
    return !vector.is_nan() && !vector.is_inf();
}

bool FlightControl::quaternion_is_valid(const Quaternion &quaternion)
{
    return !quaternion.is_nan() &&
           std::isfinite(quaternion.length_squared()) &&
           quaternion.length_squared() > CONTROL_EPSILON;
}

Quaternion FlightControl::normalised_quaternion(const Quaternion &quaternion)
{
    Quaternion normalised = quaternion;
    normalised.normalize();
    return normalised;
}

Vector3f FlightControl::quaternion_error_body(
    const Quaternion &current_body_to_ned,
    const Quaternion &target_body_to_ned)
{
    if (!quaternion_is_valid(current_body_to_ned) ||
        !quaternion_is_valid(target_body_to_ned)) {
        return Vector3f {};
    }

    const Quaternion current = normalised_quaternion(current_body_to_ned);
    const Quaternion target = normalised_quaternion(target_body_to_ned);
    Quaternion error = current.inverse() * target;
    error.normalize();

    // q and -q describe the same rotation.  Positive scalar part selects the
    // shortest rotation and avoids a discontinuity at 180 degrees.
    if (error.q1 < 0.0f) {
        error.q1 = -error.q1;
        error.q2 = -error.q2;
        error.q3 = -error.q3;
        error.q4 = -error.q4;
    }

    Vector3f rotation_vector;
    error.to_axis_angle(rotation_vector);
    if (!vector_is_finite(rotation_vector)) {
        rotation_vector.zero();
    }
    return rotation_vector;
}

Vector3f FlightControl::limit_vector_length(const Vector3f &vector,
                                             float maximum_length,
                                             bool &limited)
{
    limited = false;
    if (!vector_is_finite(vector) || !std::isfinite(maximum_length) ||
        maximum_length <= 0.0f) {
        limited = !vector.is_zero();
        return Vector3f {};
    }

    const float length = vector.length();
    if (!std::isfinite(length) || length <= maximum_length ||
        length <= CONTROL_EPSILON) {
        return vector;
    }

    limited = true;
    return vector * (maximum_length / length);
}

bool FlightControl::waypoint_is_reached(const Vector3f &position_error_m,
                                         const Vector3f &velocity_ned_mps,
                                         float radius_m,
                                         float speed_mps)
{
    if (!vector_is_finite(position_error_m) ||
        !vector_is_finite(velocity_ned_mps) ||
        !std::isfinite(radius_m) || !std::isfinite(speed_mps)) {
        return false;
    }

    return position_error_m.length() <= MAX(radius_m, 0.0f) &&
           velocity_ned_mps.length() <= MAX(speed_mps, 0.0f);
}

bool FlightControl::attitude_from_axis_ned(const Vector3f &axis_ned,
                                            float pitch_limit_rad,
                                            float yaw_fallback_rad,
                                            Quaternion &attitude_body_to_ned,
                                            Vector3f &limited_axis_ned,
                                            bool &pitch_limited)
{
    pitch_limited = false;
    if (!vector_is_finite(axis_ned) || axis_ned.length() <= CONTROL_EPSILON ||
        !std::isfinite(pitch_limit_rad) || !std::isfinite(yaw_fallback_rad)) {
        return false;
    }

    const Vector3f axis = axis_ned.normalized();
    const float horizontal_length = sqrtf(sq(axis.x) + sq(axis.y));
    const float yaw = horizontal_length > CONTROL_EPSILON ?
        atan2f(axis.y, axis.x) : yaw_fallback_rad;
    const float requested_pitch = atan2f(-axis.z, horizontal_length);
    const float pitch_limit = constrain_float(fabsf(pitch_limit_rad),
                                               0.0f,
                                               MAX_PITCH_RAD);
    const float pitch = constrain_float(requested_pitch,
                                        -pitch_limit,
                                        pitch_limit);
    pitch_limited = fabsf(pitch - requested_pitch) > CONTROL_EPSILON;

    attitude_body_to_ned.from_euler(0.0f, pitch, wrap_PI(yaw));
    attitude_body_to_ned.normalize();
    limited_axis_ned = attitude_body_to_ned * Vector3f {1.0f, 0.0f, 0.0f};
    return quaternion_is_valid(attitude_body_to_ned) &&
           vector_is_finite(limited_axis_ned);
}

float FlightControl::thrust_alignment_scale(const Vector3f &current_axis_ned,
                                             const Vector3f &target_axis_ned,
                                             float full_error_angle_rad)
{
    if (!vector_is_finite(current_axis_ned) ||
        !vector_is_finite(target_axis_ned) ||
        current_axis_ned.length() <= CONTROL_EPSILON ||
        target_axis_ned.length() <= CONTROL_EPSILON ||
        !std::isfinite(full_error_angle_rad)) {
        return 0.0f;
    }

    const float maximum_error = constrain_float(fabsf(full_error_angle_rad),
                                                 radians(1.0f),
                                                 radians(90.0f));
    const float cosine_limit = cosf(maximum_error);
    const float alignment = constrain_float(current_axis_ned.normalized() *
                                            target_axis_ned.normalized(),
                                            -1.0f,
                                            1.0f);
    if (alignment <= cosine_limit) {
        return 0.0f;
    }
    return constrain_float((alignment - cosine_limit) /
                           (1.0f - cosine_limit), 0.0f, 1.0f);
}

bool FlightControl::manual_input_is_finite(const ManualInput &input)
{
    return std::isfinite(input.collective) &&
           std::isfinite(input.roll) &&
           std::isfinite(input.pitch) &&
           std::isfinite(input.yaw);
}

void FlightControl::reset_rate_controllers()
{
    _params.rate_roll_pid.reset_I();
    _params.rate_pitch_pid.reset_I();
    _params.rate_yaw_pid.reset_I();
    _params.rate_roll_pid.reset_filter();
    _params.rate_pitch_pid.reset_filter();
    _params.rate_yaw_pid.reset_filter();
    _last_rate_output_limited = false;
}

void FlightControl::reset_navigation(
    const Quaternion &current_attitude_body_to_ned,
    bool clear_target)
{
    _velocity_integrator_ned_mss.zero();
    _navigation_thrust_sign = 1;
    _navigation_thrust_sign_initialised = false;
    _navigation_attitude_valid = quaternion_is_valid(current_attitude_body_to_ned);
    if (_navigation_attitude_valid) {
        _navigation_attitude_target =
            normalised_quaternion(current_attitude_body_to_ned);
    } else {
        _navigation_attitude_target.initialise();
    }
    if (clear_target) {
        clear_position_target();
    }
}

void FlightControl::reset(const Quaternion &current_attitude_body_to_ned)
{
    reset_rate_controllers();
    reset_navigation(current_attitude_body_to_ned, false);

    _manual_target_initialised = quaternion_is_valid(current_attitude_body_to_ned);
    if (_manual_target_initialised) {
        _manual_yaw_target_rad =
            normalised_quaternion(current_attitude_body_to_ned).get_euler_yaw();
    } else {
        _manual_yaw_target_rad = 0.0f;
    }
}

bool FlightControl::build_manual_target(
    const ManualInput &input,
    const Quaternion &current_attitude_body_to_ned,
    float dt,
    AttitudeTarget &target)
{
    target = AttitudeTarget {};
    const float control_dt = bounded_dt(dt);
    if (!manual_input_is_finite(input) || control_dt <= 0.0f ||
        !quaternion_is_valid(current_attitude_body_to_ned)) {
        return false;
    }

    if (!_manual_target_initialised) {
        _manual_yaw_target_rad =
            normalised_quaternion(current_attitude_body_to_ned).get_euler_yaw();
        _manual_target_initialised = true;
    }

    const float roll_limit = radians(constrain_float(
        fabsf(_params.manual_roll_max_deg.get()), 0.0f, 85.0f));
    const float pitch_limit = radians(constrain_float(
        fabsf(_params.manual_pitch_max_deg.get()), 0.0f, 85.0f));
    const float yaw_rate_limit = radians(constrain_float(
        fabsf(_params.manual_yaw_rate_max_dps.get()), 0.0f, 360.0f));

    const float roll_target = bounded_unit_input(input.roll) * roll_limit;
    const float pitch_target = bounded_unit_input(input.pitch) * pitch_limit;
    _manual_yaw_target_rad = wrap_PI(
        _manual_yaw_target_rad +
        bounded_unit_input(input.yaw) * yaw_rate_limit * control_dt);

    target.attitude_body_to_ned.from_euler(roll_target,
                                            pitch_target,
                                            _manual_yaw_target_rad);
    target.attitude_body_to_ned.normalize();
    target.collective = bounded_unit_input(input.collective) *
        constrain_float(fabsf(_params.manual_collective_max.get()), 0.0f, 1.0f);
    target.valid = quaternion_is_valid(target.attitude_body_to_ned) &&
                   std::isfinite(target.collective);
    return target.valid;
}

FlightControl::AttitudeControlOutput FlightControl::update_attitude(
    const AttitudeTarget &target,
    const Quaternion &current_attitude_body_to_ned,
    const Vector3f &gyro_body_rads,
    float dt,
    bool allocator_attitude_limited)
{
    AttitudeControlOutput output {};
    const float control_dt = bounded_dt(dt);
    if (!target.valid || control_dt <= 0.0f ||
        !quaternion_is_valid(target.attitude_body_to_ned) ||
        !quaternion_is_valid(current_attitude_body_to_ned) ||
        !vector_is_finite(gyro_body_rads) ||
        !std::isfinite(target.collective)) {
        return output;
    }

    output.attitude_error_rad = quaternion_error_body(
        current_attitude_body_to_ned, target.attitude_body_to_ned);
    output.rate_target_rads.x = output.attitude_error_rad.x *
        MAX(_params.att_angle_roll_p.get(), 0.0f);
    output.rate_target_rads.y = output.attitude_error_rad.y *
        MAX(_params.att_angle_pitch_p.get(), 0.0f);
    output.rate_target_rads.z = output.attitude_error_rad.z *
        MAX(_params.att_angle_yaw_p.get(), 0.0f);

    const Vector3f rate_limit_rads {
        radians(MAX(_params.att_rate_roll_max_dps.get(), 0.0f)),
        radians(MAX(_params.att_rate_pitch_max_dps.get(), 0.0f)),
        radians(MAX(_params.att_rate_yaw_max_dps.get(), 0.0f)),
    };
    output.rate_target_rads.x = constrain_float(output.rate_target_rads.x,
                                                -rate_limit_rads.x,
                                                rate_limit_rads.x);
    output.rate_target_rads.y = constrain_float(output.rate_target_rads.y,
                                                -rate_limit_rads.y,
                                                rate_limit_rads.y);
    output.rate_target_rads.z = constrain_float(output.rate_target_rads.z,
                                                -rate_limit_rads.z,
                                                rate_limit_rads.z);

    const bool integrator_limited = allocator_attitude_limited ||
                                    _last_rate_output_limited;
    const Vector3f raw_torque {
        _params.rate_roll_pid.update_all(output.rate_target_rads.x,
                                         gyro_body_rads.x,
                                         control_dt,
                                         integrator_limited),
        _params.rate_pitch_pid.update_all(output.rate_target_rads.y,
                                          gyro_body_rads.y,
                                          control_dt,
                                          integrator_limited),
        _params.rate_yaw_pid.update_all(output.rate_target_rads.z,
                                        gyro_body_rads.z,
                                        control_dt,
                                        integrator_limited),
    };
    if (!vector_is_finite(raw_torque)) {
        reset_rate_controllers();
        return AttitudeControlOutput {};
    }

    output.torque.x = constrain_float(raw_torque.x, -1.0f, 1.0f);
    output.torque.y = constrain_float(raw_torque.y, -1.0f, 1.0f);
    output.torque.z = constrain_float(raw_torque.z, -1.0f, 1.0f);
    output.rate_limited = fabsf(raw_torque.x - output.torque.x) > CONTROL_EPSILON ||
                          fabsf(raw_torque.y - output.torque.y) > CONTROL_EPSILON ||
                          fabsf(raw_torque.z - output.torque.z) > CONTROL_EPSILON;
    _last_rate_output_limited = output.rate_limited;
    output.collective = constrain_float(target.collective, -1.0f, 1.0f);
    output.valid = true;
    return output;
}

bool FlightControl::set_position_target(const Vector3f &position_ned_m)
{
    if (!vector_is_finite(position_ned_m)) {
        return false;
    }
    _position_target_ned_m = position_ned_m;
    _position_target_valid = true;
    _velocity_integrator_ned_mss.zero();
    return true;
}

void FlightControl::clear_position_target()
{
    _position_target_ned_m.zero();
    _position_target_valid = false;
    _velocity_integrator_ned_mss.zero();
}

FlightControl::GuidanceOutput FlightControl::update_guidance(
    const NavigationState &state,
    float dt)
{
    GuidanceOutput output {};
    const float control_dt = bounded_dt(dt);
    if (!_position_target_valid || control_dt <= 0.0f ||
        !state.position_valid || !state.velocity_valid || !state.attitude_valid ||
        !vector_is_finite(state.position_ned_m) ||
        !vector_is_finite(state.velocity_ned_mps) ||
        !quaternion_is_valid(state.attitude_body_to_ned)) {
        return output;
    }

    const Quaternion current_attitude =
        normalised_quaternion(state.attitude_body_to_ned);
    if (!_navigation_attitude_valid) {
        _navigation_attitude_target = current_attitude;
        _navigation_attitude_valid = true;
    }

    output.position_error_m = _position_target_ned_m - state.position_ned_m;
    bool velocity_limited = false;
    output.velocity_target_mps = limit_vector_length(
        output.position_error_m * MAX(_params.position_p.get(), 0.0f),
        MAX(_params.nav_velocity_max_mps.get(), 0.0f),
        velocity_limited);
    output.velocity_error_mps = output.velocity_target_mps -
                                state.velocity_ned_mps;

    const float velocity_i = MAX(_params.velocity_i.get(), 0.0f);
    _velocity_integrator_ned_mss += output.velocity_error_mps *
                                    (velocity_i * control_dt);
    bool integrator_limited = false;
    _velocity_integrator_ned_mss = limit_vector_length(
        _velocity_integrator_ned_mss,
        MAX(_params.velocity_imax.get(), 0.0f),
        integrator_limited);

    const Vector3f acceleration_unlimited =
        output.velocity_error_mps * MAX(_params.velocity_p.get(), 0.0f) +
        _velocity_integrator_ned_mss;
    output.acceleration_request_mss = limit_vector_length(
        acceleration_unlimited,
        MAX(_params.nav_accel_max_mss.get(), 0.0f),
        output.acceleration_limited);

    output.waypoint_reached = waypoint_is_reached(
        output.position_error_m,
        state.velocity_ned_mps,
        _params.waypoint_radius_m.get(),
        _params.waypoint_speed_mps.get());

    const float acceleration_magnitude = output.acceleration_request_mss.length();
    const float acceleration_min = MAX(_params.nav_accel_min_mss.get(), 0.0f);
    float collective = 0.0f;
    if (acceleration_magnitude > MAX(acceleration_min, CONTROL_EPSILON)) {
        const Vector3f acceleration_axis =
            output.acceleration_request_mss / acceleration_magnitude;
        const Vector3f current_axis_ned =
            current_attitude * Vector3f {1.0f, 0.0f, 0.0f};
        const float direction_dot = acceleration_axis * current_axis_ned;
        const float reverse_hysteresis = constrain_float(
            fabsf(_params.nav_reverse_hysteresis.get()), 0.0f, 0.9f);

        if (!_navigation_thrust_sign_initialised) {
            _navigation_thrust_sign = direction_dot >= 0.0f ? 1 : -1;
            _navigation_thrust_sign_initialised = true;
        } else if (_navigation_thrust_sign * direction_dot <
                   -reverse_hysteresis) {
            _navigation_thrust_sign = -_navigation_thrust_sign;
        }

        const Vector3f requested_axis = acceleration_axis *
                                        float(_navigation_thrust_sign);
        const float yaw_fallback = _navigation_attitude_target.get_euler_yaw();
        if (!attitude_from_axis_ned(
                requested_axis,
                radians(constrain_float(fabsf(_params.nav_pitch_max_deg.get()),
                                        0.0f, 85.0f)),
                yaw_fallback,
                _navigation_attitude_target,
                output.axis_target_ned,
                output.pitch_limited)) {
            return GuidanceOutput {};
        }

        const float accel_max = MAX(_params.nav_accel_max_mss.get(),
                                    CONTROL_EPSILON);
        const float collective_max = constrain_float(
            fabsf(_params.nav_collective_max.get()), 0.0f, 1.0f);
        collective = float(_navigation_thrust_sign) *
            constrain_float(acceleration_magnitude / accel_max, 0.0f, 1.0f) *
            collective_max;
        collective *= thrust_alignment_scale(
            current_axis_ned,
            output.axis_target_ned,
            radians(_params.nav_thrust_angle_deg.get()));
    } else {
        output.axis_target_ned =
            _navigation_attitude_target * Vector3f {1.0f, 0.0f, 0.0f};
    }

    output.thrust_sign = _navigation_thrust_sign;
    output.attitude.attitude_body_to_ned = _navigation_attitude_target;
    output.attitude.collective = collective;
    output.attitude.valid = _navigation_attitude_valid &&
                            quaternion_is_valid(_navigation_attitude_target);
    output.valid = output.attitude.valid &&
                   vector_is_finite(output.acceleration_request_mss) &&
                   vector_is_finite(output.axis_target_ned) &&
                   std::isfinite(collective);
    return output;
}
