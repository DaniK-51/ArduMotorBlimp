#pragma once

#include "Blimp.h"
class Parameters;
class ParametersG2;

class GCS_Blimp;

class Mode
{

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        BRAKE =         0,
        MANUAL =        1,
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // child classes should override these methods
    virtual bool init(bool ignore_checks)
    {
        return true;
    }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const
    {
        return false;
    }
    virtual bool has_user_takeoff(bool must_navigate) const
    {
        return false;
    }
    virtual bool in_guided_mode() const
    {
        return false;
    }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    virtual bool is_landing() const
    {
        return false;
    }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const
    {
        return false;
    }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc)
    {
        return false;
    };
    virtual int32_t wp_bearing() const
    {
        return 0;
    }
    virtual uint32_t wp_distance() const
    {
        return 0;
    }
    virtual float crosstrack_error() const
    {
        return 0.0f;
    }

    void update_navigation();

    // pilot input processing
    void get_pilot_input(Vector3f &pilot, float &yaw);

protected:

    bool is_disarmed_or_landed() const;

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_MotorsBlimp *&motors;
    RC_Channel *&channel_right;
    RC_Channel *&channel_front;
    RC_Channel *&channel_up;
    RC_Channel *&channel_yaw;

public:
    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Blimp &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };

protected:

    const char *name() const override
    {
        return "MANUAL";
    }
    const char *name4() const override
    {
        return "MANU";
    }
};

class ModeBrake : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return false;
    };

protected:

    const char *name() const override
    {
        return "BRAKE";
    }
    const char *name4() const override
    {
        return "BRKE";
    }
};
