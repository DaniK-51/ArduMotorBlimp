#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_MotorBlimp : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;

protected:
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;
    uint8_t send_available_mode(uint8_t index) const override;
    uint64_t capabilities() const override;
    bool try_send_message(enum ap_message id) override;
    void handle_message(const mavlink_message_t &msg) override;

private:
    void handle_set_position_target_local_ned(const mavlink_message_t &msg);
};
