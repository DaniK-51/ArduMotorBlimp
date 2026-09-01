#pragma once

#include <AP_Math/AP_Math.h>

#include "Parameters.h"

// Allocates one signed collective command and three normalised body-torque
// commands to the four reversible motors.  Positive motor output means force
// along body +X.  The default layout is M1 top, M2 right, M3 bottom, M4 left.
class MotorMixer {
public:
    static constexpr uint8_t MOTOR_COUNT = 4;
    static constexpr uint8_t AXIS_COUNT = 4;

    enum Axis : uint8_t {
        AXIS_COLLECTIVE = 0,
        AXIS_ROLL = 1,
        AXIS_PITCH = 2,
        AXIS_YAW = 3,
    };

    struct Command {
        float collective = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;

        constexpr Command(float collective_in = 0.0f,
                          float roll_in = 0.0f,
                          float pitch_in = 0.0f,
                          float yaw_in = 0.0f) :
            collective(collective_in),
            roll(roll_in),
            pitch(pitch_in),
            yaw(yaw_in)
        {}
    };

    struct Coefficients {
        float motor[MOTOR_COUNT][AXIS_COUNT] {};
    };

    struct Result {
        float motor[MOTOR_COUNT] {};
        float achieved_collective = 0.0f;
        float achieved_roll = 0.0f;
        float achieved_pitch = 0.0f;
        float achieved_yaw = 0.0f;
        float attitude_scale = 1.0f;
        bool collective_limited = false;
        bool attitude_limited = false;
        bool valid = false;
    };

    explicit MotorMixer(const ParametersG2 &params) : _params(params) {}

    // Allocate using the live parameter matrix and retain the result for the
    // motor-output task.
    const Result &allocate(const Command &command);

    void reset();

    const Result &result() const { return _result; }
    const float *outputs() const { return _result.motor; }
    float output(uint8_t motor_index) const;

    // Pure helpers intended for unit tests.
    static Coefficients canonical_coefficients();
    static Coefficients coefficients_from_parameters(const ParametersG2 &params);
    static Result allocate(const Command &command, const Coefficients &coefficients);
    static bool command_is_finite(const Command &command);
    static bool coefficients_are_finite(const Coefficients &coefficients);
    static float normalise_to_unit(float values[MOTOR_COUNT]);

private:
    const ParametersG2 &_params;
    Result _result;
};
