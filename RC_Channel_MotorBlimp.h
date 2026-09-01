#pragma once

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

class RC_Channel_MotorBlimp : public RC_Channel {
};

class RC_Channels_MotorBlimp : public RC_Channels {
public:
    RC_Channel_MotorBlimp obj_channels[NUM_RC_CHANNELS];
    RC_Channel_MotorBlimp *channel(const uint8_t chan) override {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    bool in_rc_failsafe() const override {
        if (!has_ever_seen_rc_input()) {
            return true;
        }
        return (AP_HAL::millis() - last_input_ms()) > get_fs_timeout_ms();
    }

    bool has_valid_input() const override {
        return RC_Channels::has_valid_input() && !in_rc_failsafe();
    }

protected:
    int8_t flight_mode_channel_number() const override { return 0; }
};
