#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_MotorBlimp : public AP_Arming {
public:
    AP_Arming_MotorBlimp()
    {
        // A bidirectional motor must never receive the minimum command merely
        // because the vehicle is disarmed.  The vehicle output loop supplies
        // neutral explicitly; the HAL remains soft-disarmed until arm().
        require.set_default((uint8_t)Required::YES_ZERO_PWM);
    }

    bool arm(Method method, bool do_arming_checks = true) override;
    bool disarm(Method method, bool do_disarm_checks = true) override;

    CLASS_NO_COPY(AP_Arming_MotorBlimp);

    friend class ArduMotorBlimp;

protected:
    bool pre_arm_checks(bool report) override;
    bool gps_checks(bool report) override;
};
