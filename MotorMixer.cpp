#include "MotorMixer.h"

#include <cmath>

namespace {

constexpr float MIX_EPSILON = 1.0e-6f;

float command_axis(const MotorMixer::Command &command, MotorMixer::Axis axis)
{
    switch (axis) {
    case MotorMixer::AXIS_COLLECTIVE:
        return command.collective;
    case MotorMixer::AXIS_ROLL:
        return command.roll;
    case MotorMixer::AXIS_PITCH:
        return command.pitch;
    case MotorMixer::AXIS_YAW:
        return command.yaw;
    }
    return 0.0f;
}

} // namespace

MotorMixer::Coefficients MotorMixer::canonical_coefficients()
{
    // Rows are M1 top, M2 right, M3 bottom, M4 left.  Columns are
    // collective, roll, pitch, yaw.
    Coefficients coefficients {};
    const float canonical[MOTOR_COUNT][AXIS_COUNT] {
        { 1.0f,  1.0f, -1.0f,  0.0f },
        { 1.0f, -1.0f,  0.0f, -1.0f },
        { 1.0f,  1.0f,  1.0f,  0.0f },
        { 1.0f, -1.0f,  0.0f,  1.0f },
    };

    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            coefficients.motor[motor][axis] = canonical[motor][axis];
        }
    }
    return coefficients;
}

MotorMixer::Coefficients MotorMixer::coefficients_from_parameters(const ParametersG2 &params)
{
    Coefficients coefficients {};

    coefficients.motor[0][AXIS_COLLECTIVE] = params.mix_m1_fwd.get();
    coefficients.motor[0][AXIS_ROLL] = params.mix_m1_roll.get();
    coefficients.motor[0][AXIS_PITCH] = params.mix_m1_pitch.get();
    coefficients.motor[0][AXIS_YAW] = params.mix_m1_yaw.get();

    coefficients.motor[1][AXIS_COLLECTIVE] = params.mix_m2_fwd.get();
    coefficients.motor[1][AXIS_ROLL] = params.mix_m2_roll.get();
    coefficients.motor[1][AXIS_PITCH] = params.mix_m2_pitch.get();
    coefficients.motor[1][AXIS_YAW] = params.mix_m2_yaw.get();

    coefficients.motor[2][AXIS_COLLECTIVE] = params.mix_m3_fwd.get();
    coefficients.motor[2][AXIS_ROLL] = params.mix_m3_roll.get();
    coefficients.motor[2][AXIS_PITCH] = params.mix_m3_pitch.get();
    coefficients.motor[2][AXIS_YAW] = params.mix_m3_yaw.get();

    coefficients.motor[3][AXIS_COLLECTIVE] = params.mix_m4_fwd.get();
    coefficients.motor[3][AXIS_ROLL] = params.mix_m4_roll.get();
    coefficients.motor[3][AXIS_PITCH] = params.mix_m4_pitch.get();
    coefficients.motor[3][AXIS_YAW] = params.mix_m4_yaw.get();

    return coefficients;
}

bool MotorMixer::command_is_finite(const Command &command)
{
    return std::isfinite(command.collective) &&
           std::isfinite(command.roll) &&
           std::isfinite(command.pitch) &&
           std::isfinite(command.yaw);
}

bool MotorMixer::coefficients_are_finite(const Coefficients &coefficients)
{
    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            if (!std::isfinite(coefficients.motor[motor][axis])) {
                return false;
            }
        }
    }
    return true;
}

float MotorMixer::normalise_to_unit(float values[MOTOR_COUNT])
{
    float maximum = 0.0f;
    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        maximum = MAX(maximum, fabsf(values[motor]));
    }

    if (maximum <= 1.0f) {
        return 1.0f;
    }

    const float scale = 1.0f / maximum;
    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        values[motor] *= scale;
    }
    return scale;
}

MotorMixer::Result MotorMixer::allocate(const Command &command,
                                        const Coefficients &coefficients)
{
    Result result {};
    if (!command_is_finite(command) || !coefficients_are_finite(coefficients)) {
        return result;
    }

    Command bounded {
        constrain_float(command.collective, -1.0f, 1.0f),
        constrain_float(command.roll, -1.0f, 1.0f),
        constrain_float(command.pitch, -1.0f, 1.0f),
        constrain_float(command.yaw, -1.0f, 1.0f),
    };

    // Build only the attitude part first.  A single scale is applied to all
    // three attitude axes, preserving the requested torque direction.
    float attitude[MOTOR_COUNT] {};
    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        for (uint8_t axis = AXIS_ROLL; axis < AXIS_COUNT; axis++) {
            attitude[motor] += coefficients.motor[motor][axis] *
                               command_axis(bounded, static_cast<Axis>(axis));
        }
    }

    result.attitude_scale = normalise_to_unit(attitude);
    result.attitude_limited =
        result.attitude_scale < (1.0f - MIX_EPSILON) ||
        fabsf(bounded.roll - command.roll) > MIX_EPSILON ||
        fabsf(bounded.pitch - command.pitch) > MIX_EPSILON ||
        fabsf(bounded.yaw - command.yaw) > MIX_EPSILON;
    result.achieved_roll = bounded.roll * result.attitude_scale;
    result.achieved_pitch = bounded.pitch * result.attitude_scale;
    result.achieved_yaw = bounded.yaw * result.attitude_scale;

    // Find the interval of collective values for which every motor remains
    // inside [-1,1].  Zero collective is always feasible after attitude
    // normalisation.  This also supports a user-supplied FWD column whose
    // entries are not all +1.
    float collective_min = -1.0f;
    float collective_max = 1.0f;
    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        const float coefficient = coefficients.motor[motor][AXIS_COLLECTIVE];
        if (fabsf(coefficient) <= MIX_EPSILON) {
            continue;
        }

        float lower = (-1.0f - attitude[motor]) / coefficient;
        float upper = (1.0f - attitude[motor]) / coefficient;
        if (lower > upper) {
            const float temporary = lower;
            lower = upper;
            upper = temporary;
        }
        collective_min = MAX(collective_min, lower);
        collective_max = MIN(collective_max, upper);
    }

    // Numerical roundoff can make the interval very slightly inverted at a
    // fully saturated attitude request.  Zero remains the safe fallback.
    if (collective_min > collective_max) {
        collective_min = 0.0f;
        collective_max = 0.0f;
    }

    result.achieved_collective = constrain_float(bounded.collective,
                                                  collective_min,
                                                  collective_max);
    result.collective_limited =
        fabsf(result.achieved_collective - command.collective) > MIX_EPSILON;

    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        const float output = attitude[motor] +
            coefficients.motor[motor][AXIS_COLLECTIVE] * result.achieved_collective;
        result.motor[motor] = constrain_float(output, -1.0f, 1.0f);
    }

    result.valid = true;
    return result;
}

const MotorMixer::Result &MotorMixer::allocate(const Command &command)
{
    _result = allocate(command, coefficients_from_parameters(_params));
    return _result;
}

void MotorMixer::reset()
{
    _result = Result {};
}

float MotorMixer::output(uint8_t motor_index) const
{
    if (motor_index >= MOTOR_COUNT) {
        return 0.0f;
    }
    return _result.motor[motor_index];
}
