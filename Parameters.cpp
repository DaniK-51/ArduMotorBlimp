#include "Blimp.h"

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Blimp parameter definitions
 *
 */

#define DEFAULT_FRAME_CLASS 0

const AP_Param::Info Blimp::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @Range: 1 255
    // @Increment: 1
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: .5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Param: PILOT_THR_BHV
    // @DisplayName: Throttle stick behavior
    // @Description: Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
    // @User: Standard
    // @Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection
    GSCALAR(throttle_behavior, "PILOT_THR_BHV", 0),

    // SerialManager was here

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay, "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:VELX,1:VELY,2:VELZ,3:VELYAW,4:POSX,5:POSY,6:POZ,7:POSYAW
    GSCALAR(gcs_pid_mask, "GCS_PID_MASK",     0),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.
    // @Values: 0:Disabled/NoAction,5:Land
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values:  0:Disabled,3:Enabled always Land
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_ENABLED_ALWAYS_RTL),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 910 1100
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:LAND,1:MANUAL,2:VELOCITY,3:LOITER,4:RTL,5:AUTO
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot.
    // @CopyValuesFrom: FLTMODE1
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (uint8_t)Mode::Number::MANUAL),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. This value is made up of the sum of each of the log types you want to be saved. On boards supporting microSD cards or other large block-storage devices it is usually best just to enable all basic log types by setting this to 65535.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,6:RC Input,7:IMU,9:Battery Monitor,10:RC Output,12:PID,13:Compass
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Blimp),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds. A value of zero disables auto disarm.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 1:Land, 3:Land even in MANUAL
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION",    FS_EKF_ACTION_DEFAULT),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH",    FS_EKF_THRESHOLD_DEFAULT),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check enable
    // @Description: This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    1),

    // @Param: MAX_VEL_X
    // @DisplayName: Max X Velocity
    // @Description: Sets the maximum X (forward/backward) velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_x, "MAX_VEL_X", 0.5),

    // @Param: MAX_VEL_PITCH
    // @DisplayName: Max Pitch Velocity
    // @Description: Sets the maximum pitch velocity, in rad/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_pitch, "MAX_VEL_PITCH", 0.4),

    // @Param: MAX_VEL_ROLL
    // @DisplayName: Max Roll Velocity
    // @Description: Sets the maximum roll velocity, in rad/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_roll, "MAX_VEL_ROLL", 0.4),

    // @Param: MAX_VEL_YAW
    // @DisplayName: Max yaw Velocity
    // @Description: Sets the maximum yaw velocity, in rad/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_yaw, "MAX_VEL_YAW", 0.5),

    // @Param: MAX_POS_X
    // @DisplayName: Max X Position change
    // @Description: Sets the maximum X (forward/backward) position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_x, "MAX_POS_X", 0.2),

    // @Param: MAX_POS_PITCH
    // @DisplayName: Max Pitch Position change
    // @Description: Sets the maximum pitch position change, in rad/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_pitch, "MAX_POS_PITCH", 0.15),

    // @Param: MAX_POS_ROLL
    // @DisplayName: Max Roll Position change
    // @Description: Sets the maximum roll position change, in rad/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_roll, "MAX_POS_ROLL", 0.15),

    // @Param: MAX_POS_YAW
    // @DisplayName: Max Yaw Position change
    // @Description: Sets the maximum Yaw position change, in rad/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_yaw, "MAX_POS_YAW", 0.3),

    // @Param: SIMPLE_MODE
    // @DisplayName: Simple mode
    // @Description: Simple mode for Position control - "forward" moves blimp in +ve X direction world-frame
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(simple_mode, "SIMPLE_MODE", 0),

    // @Param: DIS_MASK
    // @DisplayName: Disable output mask
    // @Description: Mask for disabling (setting to zero) one or more of the 4 output axes in mode Velocity or Loiter
    // @Bitmask: 0:Roll,1:X,2:Pitch,3:Yaw
    // @User: Standard
    GSCALAR(dis_mask, "DIS_MASK", 0),

    // @Param: PID_DZ
    // @DisplayName: Deadzone for the position PIDs
    // @Description: Output 0 thrust signal when blimp is within this distance (in meters) of the target position. Warning: If this param is greater than MAX_POS_XY param then the blimp won't move at all in the XY plane in Loiter mode as it does not allow more than a second's lag. Same for the other axes.
    // @Units: m
    // @Range: 0.1 1
    // @User: Standard
    GSCALAR(pid_dz, "PID_DZ", 0),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS", AP_InertialSensor),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[0],  gcs0,       "SR0_",     GCS_MAVLINK_Parameters),

#if MAVLINK_COMM_NUM_BUFFERS >= 2
    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 3
    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 4
    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 5
    // @Group: SR4_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[4],  gcs4,       "SR4_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 6
    // @Group: SR5_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[5],  gcs5,       "SR5_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 7
    // @Group: SR6_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[6],  gcs6,       "SR6_",     GCS_MAVLINK_Parameters),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if AP_SIM_ENABLED
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group: MOTOR_
    // @Path: MotorMix.cpp
    GOBJECTPTR(motors, "MOTOR_", MotorMix),

    // @Param: VELX_P
    // @DisplayName: Velocity (X) P gain
    // @Description: Velocity (X forward/backward) P gain. Converts velocity error to acceleration.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELX_I
    // @DisplayName: Velocity (X) I gain
    // @Description: Velocity (X) I gain. Corrects long-term velocity error.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELX_D
    // @DisplayName: Velocity (X) D gain
    // @Description: Velocity (X) D gain. Corrects short-term velocity changes.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELX_IMAX
    // @DisplayName: Velocity (X) integrator maximum
    // @Description: Velocity (X) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VELX_FLTE
    // @DisplayName: Velocity (X) input filter
    // @Description: Velocity (X) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELX_FLTD
    // @DisplayName: Velocity (X) D input filter
    // @Description: Velocity (X) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELX_FF
    // @DisplayName: Velocity (X) feed forward gain
    // @Description: Velocity (X) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_x, "VELX_", AC_PID_Basic),

    // @Param: VELPITCH_P
    // @DisplayName: Velocity (pitch) P gain
    // @Description: Velocity (pitch) P gain. Converts pitch velocity error to acceleration.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELPITCH_I
    // @DisplayName: Velocity (pitch) I gain
    // @Description: Velocity (pitch) I gain.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELPITCH_D
    // @DisplayName: Velocity (pitch) D gain
    // @Description: Velocity (pitch) D gain.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELPITCH_IMAX
    // @DisplayName: Velocity (pitch) integrator maximum
    // @Description: Velocity (pitch) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: rad/s/s
    // @User: Advanced

    // @Param: VELPITCH_FLTE
    // @DisplayName: Velocity (pitch) input filter
    // @Description: Velocity (pitch) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELPITCH_FLTD
    // @DisplayName: Velocity (pitch) D input filter
    // @Description: Velocity (pitch) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELPITCH_FF
    // @DisplayName: Velocity (pitch) feed forward gain
    // @Description: Velocity (pitch) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_pitch, "VELPITCH_", AC_PID_Basic),

    // @Param: VELROLL_P
    // @DisplayName: Velocity (roll) P gain
    // @Description: Velocity (roll) P gain. Converts roll velocity error to acceleration.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELROLL_I
    // @DisplayName: Velocity (roll) I gain
    // @Description: Velocity (roll) I gain.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELROLL_D
    // @DisplayName: Velocity (roll) D gain
    // @Description: Velocity (roll) D gain.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELROLL_IMAX
    // @DisplayName: Velocity (roll) integrator maximum
    // @Description: Velocity (roll) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: rad/s/s
    // @User: Advanced

    // @Param: VELROLL_FLTE
    // @DisplayName: Velocity (roll) input filter
    // @Description: Velocity (roll) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELROLL_FLTD
    // @DisplayName: Velocity (roll) D input filter
    // @Description: Velocity (roll) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELROLL_FF
    // @DisplayName: Velocity (roll) feed forward gain
    // @Description: Velocity (roll) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_roll, "VELROLL_", AC_PID_Basic),

    // @Param: VELYAW_P
    // @DisplayName: Velocity (yaw) P gain
    // @Description: Velocity (yaw) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELYAW_I
    // @DisplayName: Velocity (yaw) I gain
    // @Description: Velocity (yaw) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELYAW_D
    // @DisplayName: Velocity (yaw) D gain
    // @Description: Velocity (yaw) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELYAW_IMAX
    // @DisplayName: Velocity (yaw) integrator maximum
    // @Description: Velocity (yaw) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VELYAW_FLTE
    // @DisplayName: Velocity (yaw) input filter
    // @Description: Velocity (yaw) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELYAW_FF
    // @DisplayName: Velocity (yaw) feed forward gain
    // @Description: Velocity (yaw) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_yaw, "VELYAW_", AC_PID_Basic),

    // @Param: POSX_P
    // @DisplayName: Position (X) P gain
    // @Description: Position (X forward/backward) P gain. Converts position error to velocity.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: POSX_I
    // @DisplayName: Position (X) I gain
    // @Description: Position (X) I gain.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSX_D
    // @DisplayName: Position (X) D gain
    // @Description: Position (X) D gain.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSX_IMAX
    // @DisplayName: Position (X) integrator maximum
    // @Description: Position (X) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: POSX_FLTE
    // @DisplayName: Position (X) input filter
    // @Description: Position (X) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSX_FLTD
    // @DisplayName: Position (X) D input filter
    // @Description: Position (X) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSX_FF
    // @DisplayName: Position (X) feed forward gain
    // @Description: Position (X) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_pos_x, "POSX_", AC_PID_Basic),

    // @Param: POSPITCH_P
    // @DisplayName: Position (pitch) P gain
    // @Description: Position (pitch) P gain. Converts pitch position error to velocity.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: POSPITCH_I
    // @DisplayName: Position (pitch) I gain
    // @Description: Position (pitch) I gain.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSPITCH_D
    // @DisplayName: Position (pitch) D gain
    // @Description: Position (pitch) D gain.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSPITCH_IMAX
    // @DisplayName: Position (pitch) integrator maximum
    // @Description: Position (pitch) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: rad/s/s
    // @User: Advanced

    // @Param: POSPITCH_FLTE
    // @DisplayName: Position (pitch) input filter
    // @Description: Position (pitch) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSPITCH_FLTD
    // @DisplayName: Position (pitch) D input filter
    // @Description: Position (pitch) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSPITCH_FF
    // @DisplayName: Position (pitch) feed forward gain
    // @Description: Position (pitch) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_pos_pitch, "POSPITCH_", AC_PID_Basic),

    // @Param: POSROLL_P
    // @DisplayName: Position (roll) P gain
    // @Description: Position (roll) P gain. Converts roll position error to velocity.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: POSROLL_I
    // @DisplayName: Position (roll) I gain
    // @Description: Position (roll) I gain.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSROLL_D
    // @DisplayName: Position (roll) D gain
    // @Description: Position (roll) D gain.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSROLL_IMAX
    // @DisplayName: Position (roll) integrator maximum
    // @Description: Position (roll) integrator maximum.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: rad/s/s
    // @User: Advanced

    // @Param: POSROLL_FLTE
    // @DisplayName: Position (roll) input filter
    // @Description: Position (roll) input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSROLL_FLTD
    // @DisplayName: Position (roll) D input filter
    // @Description: Position (roll) D input filter in Hz.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSROLL_FF
    // @DisplayName: Position (roll) feed forward gain
    // @Description: Position (roll) feed forward gain.
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_pos_roll, "POSROLL_", AC_PID_Basic),

    // @Param: POSYAW_P
    // @DisplayName: Position (yaw) axis controller P gain
    // @Description: Position (yaw) axis controller P gain.
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_I
    // @DisplayName: Position (yaw) axis controller I gain
    // @Description: Position (yaw) axis controller I gain.
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_IMAX
    // @DisplayName: Position (yaw) axis controller I gain maximum
    // @Description: Position (yaw) axis controller I gain maximum.
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Standard

    // @Param: POSYAW_D
    // @DisplayName: Position (yaw) axis controller D gain
    // @Description: Position (yaw) axis controller D gain.
    // @Range: 0.001 0.1
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FF
    // @DisplayName: Position (yaw) axis controller feed forward
    // @Description: Position (yaw) axis controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FLTT
    // @DisplayName: Position (yaw) target frequency filter in Hz
    // @Description: Position (yaw) target frequency filter in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTE
    // @DisplayName: Position (yaw) error frequency filter in Hz
    // @Description: Position (yaw) error frequency filter in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTD
    // @DisplayName: Position (yaw) derivative input filter in Hz
    // @Description: Position (yaw) derivative input filter in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSYAW_PDMX
    // @DisplayName: Position (yaw) axis controller PD sum maximum
    // @Description: Position (yaw) axis controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Advanced

    // @Param: POSYAW_D_FF
    // @DisplayName: Position (yaw) Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.1
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSYAW_NTF
    // @DisplayName: Position (yaw) Target notch filter index
    // @Description: Position (yaw) Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSYAW_NEF
    // @DisplayName: Position (yaw) Error notch filter index
    // @Description: Position (yaw) Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    GOBJECT(pid_pos_yaw, "POSYAW_", AC_PID),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Param: DEV_OPTIONS
    // @DisplayName: Development options
    // @Description: Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning
    // @Bitmask: 0:Unknown
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 11, ParametersG2, sysid_enforce, 0),

    // 12 was AP_Stats

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for blimp.
    // @Values: 0:Finnedblimp
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_CLASS", 15, ParametersG2, frame_class, DEFAULT_FRAME_CLASS),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Blimp),

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PILOT_SPEED_DN", 24, ParametersG2, pilot_speed_dn, 0),

    // 30 was AP_Scripting

    // @Param: FS_VIBE_ENABLE
    // @DisplayName: Vibration Failsafe enable
    // @Description: This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("FS_VIBE_ENABLE", 35, ParametersG2, fs_vibe_enabled, 1),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe options bitmask
    // @Description: Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.
    // @Bitmask: 4:Continue if in pilot controlled modes on GCS failsafe
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, (float)Blimp::FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL),

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 42, ParametersG2, fs_gcs_timeout, 5),

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Blimp::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    static const AP_Param::G2ObjectConversion g2_conversions[] {
#if AP_STATS_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
        { &stats, stats.var_info, 12 },
#endif
#if AP_SCRIPTING_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
        { &scripting, scripting.var_info, 30 },
#endif
    };
    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // PARAMETER_CONVERSION - Added: Feb-2024
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_BLIMP);
}
