#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_MotorBlimp.h"

class GCS_MotorBlimp : public GCS {
public:
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_MotorBlimp);

    void update_vehicle_sensor_status_flags() override;
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

protected:
    GCS_MAVLINK_MotorBlimp *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_MotorBlimp(uart);
    }
};
