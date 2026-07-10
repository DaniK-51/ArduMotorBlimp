#include "Blimp.h"

#include <AC_AttitudeControl/AC_PosControl.h>

#define MA 0.99
#define MO (1-MA)

void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float scaler_xr_n;
    float xr_out = fabsf(blimp.motors->x_out) + fabsf(blimp.motors->roll_out);
    if (xr_out > 1) {
        scaler_xr_n = 1 / xr_out;
    } else {
        scaler_xr_n = 1;
    }
    scaler_xr = scaler_xr*MA + scaler_xr_n*MO;

    float scaler_pyaw_n;
    float pyaw_out = fabsf(blimp.motors->pitch_out) + fabsf(blimp.motors->yaw_out);
    if (pyaw_out > 1) {
        scaler_pyaw_n = 1 / pyaw_out;
    } else {
        scaler_pyaw_n = 1;
    }
    scaler_pyaw = scaler_pyaw*MA + scaler_pyaw_n*MO;

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,xr,pyaw,xrn,pyawn",
                                "Qffff",
                                AP_HAL::micros64(),
                                scaler_xr, scaler_pyaw, scaler_xr_n, scaler_pyaw_n);
#endif

    float yaw_ef = blimp.ahrs.get_yaw();
    Vector3f err_xyz = target_pos - blimp.pos_ned;
    float err_yaw = wrap_PI(target_yaw - yaw_ef);

    Vector4b zero;
    if ((fabsf(err_xyz.x) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.x = true;
    }
    if ((fabsf(err_xyz.y) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.y = true;
    }
    if ((fabsf(err_xyz.z) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if ((fabsf(err_yaw)   < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }

    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    // Position PIDs -> target velocities
    float target_vel_x = 0;
    if (!axes_disabled.x) {
        target_vel_x = blimp.pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
    }
    float target_vel_pitch = 0;
    if (!axes_disabled.z) {
        target_vel_pitch = blimp.pid_pos_pitch.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }
    float target_vel_roll = 0;
    if (!axes_disabled.y) {
        target_vel_roll = blimp.pid_pos_roll.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
    }
    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        blimp.pid_pos_yaw.set_target_rate(target_yaw);
        blimp.pid_pos_yaw.set_actual_rate(yaw_ef);
    }

    float target_vel_x_c = constrain_float(target_vel_x, -blimp.g.max_vel_x, blimp.g.max_vel_x);
    float target_vel_pitch_c = constrain_float(target_vel_pitch, -blimp.g.max_vel_pitch, blimp.g.max_vel_pitch);
    float target_vel_roll_c = constrain_float(target_vel_roll, -blimp.g.max_vel_roll, blimp.g.max_vel_roll);
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    float target_vel_x_c_scaled = target_vel_x_c * scaler_xr;
    float vel_x_filtd_scaled = blimp.vel_ned_filtd.x * scaler_xr;

    // Velocity PIDs -> actuator outputs
    float act_x = 0;
    if (!axes_disabled.x) {
        act_x = blimp.pid_vel_x.update_all(target_vel_x_c_scaled, vel_x_filtd_scaled, dt, limit.x);
    }
    float act_pitch = 0;
    if (!axes_disabled.z) {
        act_pitch = blimp.pid_vel_pitch.update_all(target_vel_pitch_c * scaler_pyaw, blimp.vel_ned_filtd.z * scaler_pyaw, dt, limit.z);
    }
    float act_roll = 0;
    if (!axes_disabled.y) {
        act_roll = blimp.pid_vel_roll.update_all(target_vel_roll_c * scaler_xr, blimp.vel_ned_filtd.y * scaler_xr, dt, limit.y);
    }
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_pyaw, blimp.vel_yaw_filtd * scaler_pyaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_pos_x.set_integrator(0);
        blimp.pid_pos_pitch.set_integrator(0);
        blimp.pid_pos_roll.set_integrator(0);
        blimp.pid_pos_yaw.set_integrator(0);
        blimp.pid_vel_x.set_integrator(0);
        blimp.pid_vel_pitch.set_integrator(0);
        blimp.pid_vel_roll.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }

    if (zero.x) {
        blimp.motors->x_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->x_out = act_x;
    }
    if (zero.y) {
        blimp.motors->roll_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->roll_out = act_roll;
    }
    if (zero.z) {
        blimp.motors->pitch_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->pitch_out = act_pitch;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_x_c * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_roll_c * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, -target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_pitch_c * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}

void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    Vector4b zero;
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.x = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.y = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }
    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    float target_vel_x_c = constrain_float(target_vel_ef.x, -blimp.g.max_vel_x, blimp.g.max_vel_x);
    float target_vel_pitch_c = constrain_float(target_vel_ef.z, -blimp.g.max_vel_pitch, blimp.g.max_vel_pitch);
    float target_vel_roll_c = constrain_float(target_vel_ef.y, -blimp.g.max_vel_roll, blimp.g.max_vel_roll);
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    float target_vel_x_c_scaled = target_vel_x_c * scaler_xr;
    float vel_x_filtd_scaled = blimp.vel_ned_filtd.x * scaler_xr;

    // Velocity PIDs -> actuator outputs
    float act_x = 0;
    if (!axes_disabled.x) {
        act_x = blimp.pid_vel_x.update_all(target_vel_x_c_scaled, vel_x_filtd_scaled, dt, limit.x);
    }
    float act_pitch = 0;
    if (!axes_disabled.z) {
        act_pitch = blimp.pid_vel_pitch.update_all(target_vel_pitch_c * scaler_pyaw, blimp.vel_ned_filtd.z * scaler_pyaw, dt, limit.z);
    }
    float act_roll = 0;
    if (!axes_disabled.y) {
        act_roll = blimp.pid_vel_roll.update_all(target_vel_roll_c * scaler_xr, blimp.vel_ned_filtd.y * scaler_xr, dt, limit.y);
    }
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_pyaw, blimp.vel_yaw_filtd * scaler_pyaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_vel_x.set_integrator(0);
        blimp.pid_vel_pitch.set_integrator(0);
        blimp.pid_vel_roll.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
    }

    if (zero.x) {
        blimp.motors->x_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->x_out = act_x;
    }
    if (zero.y) {
        blimp.motors->roll_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->roll_out = act_roll;
    }
    if (zero.z) {
        blimp.motors->pitch_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->pitch_out = act_pitch;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, 0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_x_c * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, 0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_roll_c * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, 0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_pitch_c * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}
