#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <cmath>
#include <limits>

#include "FlightControl.h"
#include "MotorMixer.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// The production parameter table lives in Parameters.cpp, which also owns the
// complete vehicle object graph and is intentionally not linked into this
// focused controller test.  Tests set every controller parameter explicitly;
// an empty table only satisfies ParametersG2's normal default-initialisation
// constructor without pulling the whole vehicle into this binary.
const AP_Param::GroupInfo ParametersG2::var_info[] = {
    AP_GROUPEND
};

namespace {

constexpr float EPSILON = 1.0e-5f;

Quaternion attitude_from_euler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);
    attitude.normalize();
    return attitude;
}

void expect_vector_near(const Vector3f &actual,
                        const Vector3f &expected,
                        float tolerance = EPSILON)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

void expect_motor_outputs(const MotorMixer::Result &result,
                          float m1,
                          float m2,
                          float m3,
                          float m4,
                          float tolerance = EPSILON)
{
    EXPECT_NEAR(result.motor[0], m1, tolerance);
    EXPECT_NEAR(result.motor[1], m2, tolerance);
    EXPECT_NEAR(result.motor[2], m3, tolerance);
    EXPECT_NEAR(result.motor[3], m4, tolerance);
}

MotorMixer::Command mixer_command(float collective,
                                  float roll,
                                  float pitch,
                                  float yaw)
{
    MotorMixer::Command command;
    command.collective = collective;
    command.roll = roll;
    command.pitch = pitch;
    command.yaw = yaw;
    return command;
}

void configure_controller_parameters(ParametersG2 &params)
{
    params.att_angle_roll_p.set(2.5f);
    params.att_angle_pitch_p.set(2.5f);
    params.att_angle_yaw_p.set(2.0f);
    params.att_rate_roll_max_dps.set(60.0f);
    params.att_rate_pitch_max_dps.set(60.0f);
    params.att_rate_yaw_max_dps.set(45.0f);

    params.manual_roll_max_deg.set(30.0f);
    params.manual_pitch_max_deg.set(60.0f);
    params.manual_yaw_rate_max_dps.set(60.0f);
    params.manual_collective_max.set(0.8f);

    params.position_p.set(1.0f);
    params.velocity_p.set(1.0f);
    params.velocity_i.set(0.0f);
    params.velocity_imax.set(0.5f);
    params.nav_velocity_max_mps.set(10.0f);
    params.nav_accel_max_mss.set(1.0f);
    params.nav_collective_max.set(0.8f);
    params.nav_pitch_max_deg.set(30.0f);
    params.nav_accel_min_mss.set(0.001f);
    params.nav_reverse_hysteresis.set(0.2f);
    params.nav_thrust_angle_deg.set(30.0f);
    params.waypoint_radius_m.set(0.2f);
    params.waypoint_speed_mps.set(0.1f);
}

FlightControl::NavigationState navigation_state(
    const Vector3f &position_ned_m = Vector3f {},
    const Vector3f &velocity_ned_mps = Vector3f {},
    const Quaternion &attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f))
{
    FlightControl::NavigationState state;
    state.position_ned_m = position_ned_m;
    state.velocity_ned_mps = velocity_ned_mps;
    state.attitude_body_to_ned = attitude_body_to_ned;
    state.position_valid = true;
    state.velocity_valid = true;
    state.attitude_valid = true;
    return state;
}

} // namespace

TEST(MotorMixer, CanonicalAxisSigns)
{
    const MotorMixer::Coefficients coefficients =
        MotorMixer::canonical_coefficients();

    MotorMixer::Command command;
    command.collective = 0.2f;
    MotorMixer::Result result = MotorMixer::allocate(command, coefficients);
    ASSERT_TRUE(result.valid);
    expect_motor_outputs(result, 0.2f, 0.2f, 0.2f, 0.2f);

    command = MotorMixer::Command {};
    command.roll = 0.2f;
    result = MotorMixer::allocate(command, coefficients);
    ASSERT_TRUE(result.valid);
    expect_motor_outputs(result, 0.2f, -0.2f, 0.2f, -0.2f);

    command = MotorMixer::Command {};
    command.pitch = 0.2f;
    result = MotorMixer::allocate(command, coefficients);
    ASSERT_TRUE(result.valid);
    expect_motor_outputs(result, -0.2f, 0.0f, 0.2f, 0.0f);

    command = MotorMixer::Command {};
    command.yaw = 0.2f;
    result = MotorMixer::allocate(command, coefficients);
    ASSERT_TRUE(result.valid);
    expect_motor_outputs(result, 0.0f, -0.2f, 0.0f, 0.2f);
}

TEST(MotorMixer, AttitudeHasPriorityAtSaturation)
{
    const MotorMixer::Command command = mixer_command(
        0.8f, // collective
        0.8f, // roll
        0.0f, // pitch
        0.8f  // yaw
    );

    const MotorMixer::Result result = MotorMixer::allocate(
        command, MotorMixer::canonical_coefficients());

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.attitude_limited);
    EXPECT_TRUE(result.collective_limited);
    EXPECT_NEAR(result.attitude_scale, 0.625f, EPSILON);
    EXPECT_NEAR(result.achieved_roll, 0.5f, EPSILON);
    EXPECT_NEAR(result.achieved_yaw, 0.5f, EPSILON);
    EXPECT_NEAR(result.achieved_collective, 0.5f, EPSILON);

    // The attitude vector is scaled as a whole before collective is added.
    // Independent per-motor clipping would produce different M2/M4 values.
    expect_motor_outputs(result, 1.0f, -0.5f, 1.0f, 0.5f);
}

TEST(MotorMixer, AllocationStaysBoundedAndReconstructable)
{
    const MotorMixer::Coefficients coefficients =
        MotorMixer::canonical_coefficients();
    const float requests[] {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

    for (const float collective : requests) {
        for (const float roll : requests) {
            for (const float pitch : requests) {
                for (const float yaw : requests) {
                    const MotorMixer::Result result = MotorMixer::allocate(
                        mixer_command(collective, roll, pitch, yaw),
                        coefficients);
                    ASSERT_TRUE(result.valid);

                    for (uint8_t motor = 0;
                         motor < MotorMixer::MOTOR_COUNT;
                         motor++) {
                        EXPECT_GE(result.motor[motor], -1.0f - EPSILON);
                        EXPECT_LE(result.motor[motor], 1.0f + EPSILON);

                        const float reconstructed =
                            coefficients.motor[motor][MotorMixer::AXIS_COLLECTIVE] *
                                result.achieved_collective +
                            coefficients.motor[motor][MotorMixer::AXIS_ROLL] *
                                result.achieved_roll +
                            coefficients.motor[motor][MotorMixer::AXIS_PITCH] *
                                result.achieved_pitch +
                            coefficients.motor[motor][MotorMixer::AXIS_YAW] *
                                result.achieved_yaw;
                        EXPECT_NEAR(result.motor[motor], reconstructed, EPSILON);
                    }
                }
            }
        }
    }
}

TEST(MotorMixer, RejectsNonFiniteInputsWithoutMotorOutput)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    MotorMixer::Coefficients coefficients =
        MotorMixer::canonical_coefficients();

    MotorMixer::Command command;
    command.collective = nan;
    MotorMixer::Result result = MotorMixer::allocate(command, coefficients);
    EXPECT_FALSE(result.valid);
    expect_motor_outputs(result, 0.0f, 0.0f, 0.0f, 0.0f);

    command = MotorMixer::Command {};
    command.yaw = infinity;
    result = MotorMixer::allocate(command, coefficients);
    EXPECT_FALSE(result.valid);
    expect_motor_outputs(result, 0.0f, 0.0f, 0.0f, 0.0f);

    command = MotorMixer::Command {};
    coefficients.motor[2][MotorMixer::AXIS_PITCH] = nan;
    result = MotorMixer::allocate(command, coefficients);
    EXPECT_FALSE(result.valid);
    expect_motor_outputs(result, 0.0f, 0.0f, 0.0f, 0.0f);
}

TEST(FlightControl, QuaternionErrorUsesBodyAxesAndShortestRotation)
{
    const Quaternion identity = attitude_from_euler(0.0f, 0.0f, 0.0f);
    const Quaternion roll_target =
        attitude_from_euler(radians(20.0f), 0.0f, 0.0f);
    expect_vector_near(
        FlightControl::quaternion_error_body(identity, roll_target),
        Vector3f {radians(20.0f), 0.0f, 0.0f});

    const Quaternion current =
        attitude_from_euler(0.0f, 0.0f, radians(170.0f));
    Quaternion target =
        attitude_from_euler(0.0f, 0.0f, radians(-170.0f));
    Vector3f error = FlightControl::quaternion_error_body(current, target);
    EXPECT_NEAR(error.x, 0.0f, EPSILON);
    EXPECT_NEAR(error.y, 0.0f, EPSILON);
    EXPECT_NEAR(error.z, radians(20.0f), EPSILON);

    // q and -q must lead to the same attitude error.
    target.q1 = -target.q1;
    target.q2 = -target.q2;
    target.q3 = -target.q3;
    target.q4 = -target.q4;
    expect_vector_near(
        FlightControl::quaternion_error_body(current, target), error);
}

TEST(FlightControl, ManualCentredYawHoldsInitialCompassHeading)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);

    const Quaternion initial =
        attitude_from_euler(0.0f, 0.0f, radians(40.0f));
    controller.reset(initial);

    FlightControl::AttitudeTarget target;
    FlightControl::ManualInput input;
    ASSERT_TRUE(controller.build_manual_target(input, initial, 0.02f, target));
    EXPECT_NEAR(target.attitude_body_to_ned.get_euler_yaw(),
                radians(40.0f), EPSILON);

    // Heading feedback may move, but a centred yaw stick retains the latched
    // compass heading rather than following the measured heading.
    const Quaternion disturbed =
        attitude_from_euler(0.0f, 0.0f, radians(10.0f));
    ASSERT_TRUE(controller.build_manual_target(input, disturbed, 0.02f, target));
    EXPECT_NEAR(target.attitude_body_to_ned.get_euler_yaw(),
                radians(40.0f), EPSILON);

    input.roll = 0.5f;
    input.pitch = -0.5f;
    input.yaw = 1.0f;
    input.collective = -0.5f;
    ASSERT_TRUE(controller.build_manual_target(input, disturbed, 0.1f, target));
    EXPECT_NEAR(target.attitude_body_to_ned.get_euler_roll(),
                radians(15.0f), EPSILON);
    EXPECT_NEAR(target.attitude_body_to_ned.get_euler_pitch(),
                radians(-30.0f), EPSILON);
    EXPECT_NEAR(target.attitude_body_to_ned.get_euler_yaw(),
                radians(46.0f), EPSILON);
    EXPECT_NEAR(target.collective, -0.4f, EPSILON);
}

TEST(FlightControl, ManualRejectsInvalidInputAndTimeStep)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);
    const Quaternion identity = attitude_from_euler(0.0f, 0.0f, 0.0f);
    controller.reset(identity);

    FlightControl::AttitudeTarget target;
    FlightControl::ManualInput input;
    EXPECT_FALSE(controller.build_manual_target(input, identity, 0.0f, target));

    input.pitch = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(controller.build_manual_target(input, identity, 0.02f, target));
}

TEST(FlightControl, GuidancePointsBodyXAxisAlongNEDDemand)
{
    ParametersG2 params;
    configure_controller_parameters(params);

    const Quaternion identity = attitude_from_euler(0.0f, 0.0f, 0.0f);
    FlightControl controller(params);
    controller.reset(identity);
    ASSERT_TRUE(controller.set_position_target(Vector3f {0.0f, 10.0f, 0.0f}));

    FlightControl::GuidanceOutput output =
        controller.update_guidance(navigation_state(), 0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.thrust_sign, 1);
    expect_vector_near(output.axis_target_ned, Vector3f {0.0f, 1.0f, 0.0f});
    // At a 90-degree pointing error thrust is withheld while the nose turns.
    EXPECT_NEAR(output.attitude.collective, 0.0f, EPSILON);

    const Quaternion pointing_east =
        attitude_from_euler(0.0f, 0.0f, radians(90.0f));
    FlightControl aligned_controller(params);
    aligned_controller.reset(pointing_east);
    ASSERT_TRUE(aligned_controller.set_position_target(
        Vector3f {0.0f, 10.0f, 0.0f}));
    output = aligned_controller.update_guidance(
        navigation_state(Vector3f {}, Vector3f {}, pointing_east), 0.02f);
    ASSERT_TRUE(output.valid);
    expect_vector_near(output.axis_target_ned, Vector3f {0.0f, 1.0f, 0.0f});
    EXPECT_GT(output.attitude.collective, 0.0f);
}

TEST(FlightControl, GuidanceLimitsPitchForVerticalWaypoint)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);
    const Quaternion identity = attitude_from_euler(0.0f, 0.0f, 0.0f);
    controller.reset(identity);

    // Negative NED Z is upward.  With no vertical thruster the target body X
    // axis must pitch up, and NAV_PIT_MAX limits that attitude request.
    ASSERT_TRUE(controller.set_position_target(Vector3f {0.0f, 0.0f, -10.0f}));
    const FlightControl::GuidanceOutput output =
        controller.update_guidance(navigation_state(), 0.02f);

    ASSERT_TRUE(output.valid);
    EXPECT_TRUE(output.pitch_limited);
    EXPECT_NEAR(output.attitude.attitude_body_to_ned.get_euler_pitch(),
                radians(30.0f), EPSILON);
    expect_vector_near(output.axis_target_ned,
                       Vector3f {cosf(radians(30.0f)),
                                 0.0f,
                                 -sinf(radians(30.0f))});
}

TEST(FlightControl, ReverseSelectionUsesHysteresis)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);
    const Quaternion identity = attitude_from_euler(0.0f, 0.0f, 0.0f);
    controller.reset(identity);
    const FlightControl::NavigationState state = navigation_state();

    ASSERT_TRUE(controller.set_position_target(Vector3f {-10.0f, 0.0f, 0.0f}));
    FlightControl::GuidanceOutput output =
        controller.update_guidance(state, 0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.thrust_sign, -1);
    EXPECT_LT(output.attitude.collective, 0.0f);
    // Reverse thrust lets the vehicle keep its nose pointed along +X.
    expect_vector_near(output.axis_target_ned, Vector3f {1.0f, 0.0f, 0.0f});

    // A nearly sideways request with only a small +X component is inside the
    // hysteresis band and must not cause a forward/reverse chatter event.
    ASSERT_TRUE(controller.set_position_target(Vector3f {0.1f, 10.0f, 0.0f}));
    output = controller.update_guidance(state, 0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.thrust_sign, -1);

    // A decisive request in front crosses the hysteresis threshold.
    ASSERT_TRUE(controller.set_position_target(Vector3f {10.0f, 0.0f, 0.0f}));
    output = controller.update_guidance(state, 0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.thrust_sign, 1);
    EXPECT_GT(output.attitude.collective, 0.0f);
}

TEST(FlightControl, WaypointRequiresBothPositionAndSpeedTolerance)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);
    controller.reset(attitude_from_euler(0.0f, 0.0f, 0.0f));
    ASSERT_TRUE(controller.set_position_target(Vector3f {}));

    FlightControl::GuidanceOutput output = controller.update_guidance(
        navigation_state(Vector3f {0.1f, 0.0f, 0.0f},
                         Vector3f {0.05f, 0.0f, 0.0f}),
        0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_TRUE(output.waypoint_reached);

    output = controller.update_guidance(
        navigation_state(Vector3f {0.1f, 0.0f, 0.0f},
                         Vector3f {0.11f, 0.0f, 0.0f}),
        0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_FALSE(output.waypoint_reached);

    output = controller.update_guidance(
        navigation_state(Vector3f {0.21f, 0.0f, 0.0f}, Vector3f {}),
        0.02f);
    ASSERT_TRUE(output.valid);
    EXPECT_FALSE(output.waypoint_reached);
}

TEST(FlightControl, InvalidNavigationDataIsRejectedSafely)
{
    ParametersG2 params;
    configure_controller_parameters(params);
    FlightControl controller(params);
    controller.reset(attitude_from_euler(0.0f, 0.0f, 0.0f));
    ASSERT_TRUE(controller.set_position_target(Vector3f {1.0f, 0.0f, 0.0f}));

    const Vector3f original_target = controller.position_target();
    EXPECT_FALSE(controller.set_position_target(Vector3f {
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}));
    expect_vector_near(controller.position_target(), original_target);

    FlightControl::NavigationState state = navigation_state();
    state.position_valid = false;
    const FlightControl::GuidanceOutput output =
        controller.update_guidance(state, 0.02f);
    EXPECT_FALSE(output.valid);
    EXPECT_FALSE(output.attitude.valid);
    EXPECT_NEAR(output.attitude.collective, 0.0f, EPSILON);
}

AP_GTEST_MAIN()
