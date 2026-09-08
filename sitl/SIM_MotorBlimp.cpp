/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "SIM_MotorBlimp.h"

#if AP_SIM_MOTORBLIMP_ENABLED

#include <AP_Math/AP_Math.h>

using namespace SITL;

const AP_Param::GroupInfo MotorBlimp::var_info[] = {
    // @Param: MASS
    // @DisplayName: Airship mass
    // @Description: Total airship mass used by the basic model
    // @Units: kg
    // @Range: 0.01 100
    AP_GROUPINFO("MASS", 1, MotorBlimp, mass_kg, 0.145f),

    // @Param: LENGTH
    // @DisplayName: Hull length
    // @Description: Length of the cylindrical hull
    // @Units: m
    // @Range: 0.1 100
    AP_GROUPINFO("LENGTH", 2, MotorBlimp, length, 1.0f),

    // @Param: RADIUS
    // @DisplayName: Hull radius
    // @Description: Radius of the cylindrical hull and motor mounting ring
    // @Units: m
    // @Range: 0.01 20
    AP_GROUPINFO("RADIUS", 3, MotorBlimp, radius, 0.2f),

    // @Param: THR_MAX
    // @DisplayName: Maximum motor thrust
    // @Description: Maximum static thrust produced by one motor in either direction
    // @Units: N
    // @Range: 0 1000
    AP_GROUPINFO("THR_MAX", 4, MotorBlimp, thrust_max, 0.15f),

    // @Param: Q_MAX
    // @DisplayName: Maximum reaction torque
    // @Description: Propeller reaction torque about body X at full command
    // @Units: Nm
    // @Range: 0 100
    AP_GROUPINFO("Q_MAX", 5, MotorBlimp, reaction_torque_max, 0.0003f),

    // @Param: BUOY
    // @DisplayName: Buoyancy ratio
    // @Description: Buoyant force divided by vehicle weight. One gives neutral buoyancy
    // @Range: 0 2
    AP_GROUPINFO("BUOY", 6, MotorBlimp, buoyancy_ratio, 1.0f),

    // @Param: CB_Z
    // @DisplayName: Centre of buoyancy height
    // @Description: Positive distance that the centre of buoyancy is above the centre of mass
    // @Units: m
    // @Range: 0 10
    AP_GROUPINFO("CB_Z", 7, MotorBlimp, cb_height, 0.05f),

    // @Param: CDA_X
    // @DisplayName: Axial drag area
    // @Description: Drag coefficient times reference area along body X
    // @Units: m^2
    // @Range: 0 1000
    AP_GROUPINFO("CDA_X", 8, MotorBlimp, cda_x, 0.06f),

    // @Param: CDA_Y
    // @DisplayName: Lateral drag area
    // @Description: Drag coefficient times reference area along body Y
    // @Units: m^2
    // @Range: 0 1000
    AP_GROUPINFO("CDA_Y", 9, MotorBlimp, cda_y, 0.35f),

    // @Param: CDA_Z
    // @DisplayName: Vertical drag area
    // @Description: Drag coefficient times reference area along body Z
    // @Units: m^2
    // @Range: 0 1000
    AP_GROUPINFO("CDA_Z", 10, MotorBlimp, cda_z, 0.35f),

    // @Param: MOI_X
    // @DisplayName: Roll moment of inertia
    // @Description: Roll moment of inertia. Zero derives it from a solid cylinder
    // @Units: kgm^2
    // @Range: 0 10000
    AP_GROUPINFO("MOI_X", 11, MotorBlimp, moi_x, 0.0f),

    // @Param: MOI_Y
    // @DisplayName: Pitch moment of inertia
    // @Description: Pitch moment of inertia. Zero derives it from a solid cylinder
    // @Units: kgm^2
    // @Range: 0 10000
    AP_GROUPINFO("MOI_Y", 12, MotorBlimp, moi_y, 0.0f),

    // @Param: MOI_Z
    // @DisplayName: Yaw moment of inertia
    // @Description: Yaw moment of inertia. Zero derives it from a solid cylinder
    // @Units: kgm^2
    // @Range: 0 10000
    AP_GROUPINFO("MOI_Z", 13, MotorBlimp, moi_z, 0.0f),

    // @Param: RDAMP_X
    // @DisplayName: Quadratic roll damping
    // @Description: Quadratic angular damping coefficient about body X
    // @Range: 0 10000
    AP_GROUPINFO("RDAMP_X", 14, MotorBlimp, rotational_damping_x, 0.0005f),

    // @Param: RDAMP_Y
    // @DisplayName: Quadratic pitch damping
    // @Description: Quadratic angular damping coefficient about body Y
    // @Range: 0 10000
    AP_GROUPINFO("RDAMP_Y", 15, MotorBlimp, rotational_damping_y, 0.015f),

    // @Param: RDAMP_Z
    // @DisplayName: Quadratic yaw damping
    // @Description: Quadratic angular damping coefficient about body Z
    // @Range: 0 10000
    AP_GROUPINFO("RDAMP_Z", 16, MotorBlimp, rotational_damping_z, 0.015f),

    // @Param: RPM_MAX
    // @DisplayName: Maximum motor RPM
    // @Description: Motor RPM reported by SITL at full command
    // @Units: RPM
    // @Range: 0 100000
    AP_GROUPINFO("RPM_MAX", 17, MotorBlimp, rpm_max, 12000.0f),

    AP_GROUPEND
};

MotorBlimp::MotorBlimp(const char *frame_str) :
    Aircraft(frame_str)
{
    AP::sitl()->models.motorblimp_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);

    Aircraft::mass = MAX(mass_kg.get(), 0.001f);
    external_payload_mass = 0.0f;
    frame_height = MAX(radius.get(), 0.01f);
    ground_behavior = GROUND_BEHAVIOR_NONE;
    lock_step_scheduled = true;
    use_smoothing = false;

    // The first four PWM outputs are the four longitudinal motors.
    motor_mask = 0x0F;
    for (uint8_t i = 0; i < ARRAY_SIZE(rpm); i++) {
        rpm[i] = 0.0f;
    }

    // Leave voltage/current at zero so SITL_State supplies its generic battery
    // model using the absolute bidirectional motor demand.
    battery_voltage = 0.0f;
    battery_current = 0.0f;
}

bool MotorBlimp::on_ground() const
{
    // Aircraft::update_dynamics() normally keeps position clamped to the
    // ground until a single simulation step moves the complete frame-height
    // tolerance.  At the 1200 Hz model rate that made a light, neutrally
    // buoyant airship accumulate roughly 1.2 m/s upward velocity while its
    // position remained pinned.  Permit an upward-moving hull to leave the
    // floor immediately, while retaining the normal collision clamp during a
    // descent.
    return Aircraft::on_ground() && velocity_ef.z >= 0.0f;
}

Vector3f MotorBlimp::get_moment_of_inertia(float total_mass) const
{
    const float r = MAX(radius.get(), 0.001f);
    const float l = MAX(length.get(), 0.001f);

    // Solid-cylinder estimates. Explicit non-zero parameters override them.
    const float cylinder_x = 0.5f * total_mass * sq(r);
    const float cylinder_yz = total_mass * (3.0f * sq(r) + sq(l)) / 12.0f;

    return Vector3f{
        is_positive(moi_x.get()) ? moi_x.get() : cylinder_x,
        is_positive(moi_y.get()) ? moi_y.get() : cylinder_yz,
        is_positive(moi_z.get()) ? moi_z.get() : cylinder_yz
    };
}

void MotorBlimp::calculate_forces(const struct sitl_input &input,
                                  Vector3f &rot_accel,
                                  Vector3f &body_accel)
{
    Aircraft::mass = MAX(mass_kg.get(), 0.001f);
    frame_height = MAX(radius.get(), 0.01f);

    const float total_mass = MAX(gross_mass(), 0.001f);
    const float r = MAX(radius.get(), 0.001f);
    const float max_thrust = MAX(thrust_max.get(), 0.0f);
    const float max_reaction_torque = MAX(reaction_torque_max.get(), 0.0f);

    const Vector3f motor_position[4] {
        Vector3f{0.0f,  0.0f, -r}, // M1: top
        Vector3f{0.0f, +r,     0.0f}, // M2: right
        Vector3f{0.0f,  0.0f, +r}, // M3: bottom
        Vector3f{0.0f, -r,     0.0f}, // M4: left
    };
    const float reaction_sign[4] {+1.0f, -1.0f, +1.0f, -1.0f};

    Vector3f force_body;
    Vector3f torque_body;

    for (uint8_t i = 0; i < 4; i++) {
        const float command = constrain_float(filtered_servo_angle(input, i), -1.0f, 1.0f);
        // The mixer output is a normalised signed thrust request.  Keep the
        // SITL actuator linear in that contract; applying command squared here
        // would make low autonomous demands far weaker than the controller's
        // acceleration model predicts.
        const Vector3f motor_force{max_thrust * command, 0.0f, 0.0f};

        force_body += motor_force;
        torque_body += motor_position[i] % motor_force;
        torque_body.x += reaction_sign[i] * max_reaction_torque * command;
        rpm[i] = MAX(rpm_max.get(), 0.0f) * fabsf(command);
    }

    // Constant buoyancy is intentional for the indoor/low-altitude model. The
    // Aircraft base class adds gravity later in update_dynamics().
    const float buoyant_force = MAX(buoyancy_ratio.get(), 0.0f) * Aircraft::mass * GRAVITY_MSS;
    const Vector3f buoyancy_earth{0.0f, 0.0f, -buoyant_force};
    const Vector3f buoyancy_body = dcm.transposed() * buoyancy_earth;
    force_body += buoyancy_body;

    const Vector3f centre_of_buoyancy{0.0f, 0.0f, -MAX(cb_height.get(), 0.0f)};
    torque_body += centre_of_buoyancy % buoyancy_body;

    // Anisotropic quadratic drag of the cylindrical hull.
    const float rho = get_air_density(location.alt * 0.01f);
    force_body += Vector3f{
        -0.5f * rho * MAX(cda_x.get(), 0.0f) * velocity_air_bf.x * fabsf(velocity_air_bf.x),
        -0.5f * rho * MAX(cda_y.get(), 0.0f) * velocity_air_bf.y * fabsf(velocity_air_bf.y),
        -0.5f * rho * MAX(cda_z.get(), 0.0f) * velocity_air_bf.z * fabsf(velocity_air_bf.z)
    };

    torque_body += Vector3f{
        -MAX(rotational_damping_x.get(), 0.0f) * gyro.x * fabsf(gyro.x),
        -MAX(rotational_damping_y.get(), 0.0f) * gyro.y * fabsf(gyro.y),
        -MAX(rotational_damping_z.get(), 0.0f) * gyro.z * fabsf(gyro.z)
    };

    body_accel = force_body / total_mass;

    // Euler rigid-body equation: I*w_dot = torque - w x (I*w).
    const Vector3f inertia = get_moment_of_inertia(total_mass);
    const Vector3f angular_momentum{
        inertia.x * gyro.x,
        inertia.y * gyro.y,
        inertia.z * gyro.z
    };
    const Vector3f gyroscopic_torque = gyro % angular_momentum;
    const Vector3f net_torque = torque_body - gyroscopic_torque;

    rot_accel = Vector3f{
        net_torque.x / inertia.x,
        net_torque.y / inertia.y,
        net_torque.z / inertia.z
    };
}

void MotorBlimp::update(const struct sitl_input &input)
{
    update_wind(input);

    Vector3f rot_accel;
    calculate_forces(input, rot_accel, accel_body);

    add_shove_forces(rot_accel, accel_body);
    add_twist_forces(rot_accel);
    add_external_forces(accel_body);

    update_dynamics(rot_accel);
    update_external_payload(input);
    update_position();
    time_advance();
    update_mag_field_bf();
}

#endif // AP_SIM_MOTORBLIMP_ENABLED
