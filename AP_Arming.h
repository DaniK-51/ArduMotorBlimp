#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Blimp : public AP_Arming
{
public:
    friend class Blimp;

    AP_Arming_Blimp() : AP_Arming()
    {
        require.set_default((uint8_t)Required::YES_MIN_PWM);
    }

    CLASS_NO_COPY(AP_Arming_Blimp);

    bool rc_calibration_checks(bool display_failure) override;

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:

    bool pre_arm_checks(bool display_failure) override;
    bool arm_checks(AP_Arming::Method method) override;

    bool mandatory_checks(bool display_failure) override;

    bool ins_checks(bool display_failure) override;
    bool gps_checks(bool display_failure) override;
    bool barometer_checks(bool display_failure) override;
    bool board_voltage_checks(bool display_failure) override;

    bool parameter_checks(bool display_failure);
    bool motor_checks(bool display_failure);
    bool mandatory_gps_checks(bool display_failure);
    bool gcs_failsafe_check(bool display_failure);
    bool alt_checks(bool display_failure);

    void set_pre_arm_check(bool b);

private:

    bool run_pre_arm_checks(bool display_failure);
};
