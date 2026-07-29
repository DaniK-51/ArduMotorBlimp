#pragma once

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

    bool in_rc_failsafe() const override { return false; }

protected:
    int8_t flight_mode_channel_number() const override { return 0; }
};
