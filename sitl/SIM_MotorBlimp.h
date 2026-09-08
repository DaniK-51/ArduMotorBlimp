/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MOTORBLIMP_ENABLED

#include "SIM_Aircraft.h"

#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  Six degree-of-freedom model for a neutrally buoyant cylindrical airship.

  All four reversible propellers point along body X.  Their positions around
  the circular hull provide pitch and yaw moments, while alternating propeller
  reaction torques provide roll control.
 */
class MotorBlimp : public Aircraft {
public:
    explicit MotorBlimp(const char *frame_str);

    void update(const struct sitl_input &input) override;

    static Aircraft *create(const char *frame_str)
    {
        return NEW_NOTHROW MotorBlimp(frame_str);
    }

    static const AP_Param::GroupInfo var_info[];

private:
    void calculate_forces(const struct sitl_input &input,
                          Vector3f &rot_accel,
                          Vector3f &body_accel);
    Vector3f get_moment_of_inertia(float total_mass) const;
    bool on_ground() const override;

    AP_Float mass_kg;
    AP_Float length;
    AP_Float radius;
    AP_Float thrust_max;
    AP_Float reaction_torque_max;
    AP_Float buoyancy_ratio;
    AP_Float cb_height;
    AP_Float cda_x;
    AP_Float cda_y;
    AP_Float cda_z;
    AP_Float moi_x;
    AP_Float moi_y;
    AP_Float moi_z;
    AP_Float rotational_damping_x;
    AP_Float rotational_damping_y;
    AP_Float rotational_damping_z;
    AP_Float rpm_max;
};

} // namespace SITL

#endif // AP_SIM_MOTORBLIMP_ENABLED
