#!/usr/bin/env python3
"""Deterministic smoke test for the native ArduMotorBlimp SITL model."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - depends on the CI image
    raise SystemExit(
        "pymavlink is required; initialise ArduPilot submodules or install pymavlink"
    ) from exc


NEUTRAL_PWM = 1500
PWM_UNCHANGED = 65535
MOTOR_FIELDS = ("servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw")
MANUAL_MODE = 0
COMPASS_REQUIRED_MODES = (("HOLD", 4), ("AUTO", 10), ("GUIDED", 15))
# Average motor PWM cancels attitude differentials, so this threshold detects
# genuine longitudinal collective while tolerating telemetry quantisation.
GUIDED_COLLECTIVE_DETECTION_PWM = 40.0
GUIDED_MIN_ESTIMATED_CLIMB_M = 0.20
GUIDED_MIN_TRUTH_CLIMB_M = 0.15
GUIDED_MAX_CLIMB_DISAGREEMENT_M = 0.15
POSITION_CONTROL_SENSOR_MASK = (
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL
    | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL
)


class SmokeFailure(RuntimeError):
    pass


class SITLSession:
    def __init__(
        self,
        binary: Path,
        defaults: Path,
        workdir: Path,
        port: int,
        home: str,
        startup_timeout: float,
    ) -> None:
        self.binary = binary
        self.defaults = defaults
        self.workdir = workdir
        self.port = port
        self.home = home
        self.startup_timeout = startup_timeout
        self.process: Optional[subprocess.Popen[bytes]] = None
        self.master = None
        self.log_handle = None
        self.log_path = self.workdir / "sitl.log"
        self.target_system = 1
        self.target_component = 1
        self.override_channel_count = 18

    def start(self) -> None:
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.log_handle = self.log_path.open("wb")
        command = [
            str(self.binary),
            "--model",
            "motorblimp",
            "--wipe",
            "--defaults",
            str(self.defaults),
            "--home",
            self.home,
            "--base-port",
            str(self.port),
            "--speedup",
            "1",
        ]
        self.process = subprocess.Popen(
            command,
            cwd=self.workdir,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.monotonic() + self.startup_timeout
        endpoint = f"tcp:127.0.0.1:{self.port}"
        last_error: Optional[BaseException] = None
        while time.monotonic() < deadline:
            self._check_process()
            connection = None
            try:
                connection = mavutil.mavlink_connection(
                    endpoint,
                    source_system=255,
                    autoreconnect=False,
                    robust_parsing=True,
                )
                heartbeat = connection.wait_heartbeat(timeout=1)
                if heartbeat is not None:
                    self.master = connection
                    self.target_system = heartbeat.get_srcSystem()
                    self.target_component = heartbeat.get_srcComponent()
                    if heartbeat.type != mavutil.mavlink.MAV_TYPE_AIRSHIP:
                        raise SmokeFailure(
                            f"heartbeat MAV_TYPE={heartbeat.type}, expected MAV_TYPE_AIRSHIP"
                    )
                    self._request_messages()
                    heartbeat = self._wait_until_ready(heartbeat)
                    heartbeat = self._wait_until_active()
                    self._ensure_armed(heartbeat)
                    return
            except SmokeFailure:
                if connection is not None:
                    connection.close()
                raise
            except (ConnectionError, OSError) as exc:
                last_error = exc
                if connection is not None:
                    connection.close()
                time.sleep(0.2)

        detail = f": {last_error}" if last_error is not None else ""
        raise SmokeFailure(f"no heartbeat on {endpoint} within {self.startup_timeout}s{detail}")

    def _ensure_armed(self, heartbeat) -> None:
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if heartbeat.base_mode & armed_flag:
            return

        # Use the normal arming path: this smoke test must exercise pre-arm
        # sensor, estimator, compass, and RC checks instead of bypassing them.
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            message = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if message is not None and message.base_mode & armed_flag:
                return
        raise SmokeFailure("vehicle did not arm in SITL")

    def _wait_until_active(self):
        deadline = time.monotonic() + self.startup_timeout
        next_override = 0.0
        next_arm = 0.0
        active_samples = 0
        last_system_status = None
        last_status_text = "<none>"
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_override:
                self.send_override((NEUTRAL_PWM,) * 4)
                next_override = now + 0.08
            if now >= next_arm:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                next_arm = now + 2.0

            message = self.master.recv_match(
                type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.3
            )
            if message is None:
                continue
            if message.get_type() == "STATUSTEXT":
                last_status_text = str(message.text)
                continue
            heartbeat = message
            last_system_status = int(heartbeat.system_status)
            if (
                heartbeat.base_mode & armed_flag
                and last_system_status == mavutil.mavlink.MAV_STATE_ACTIVE
            ):
                active_samples += 1
                if active_samples >= 2:
                    return heartbeat
            else:
                active_samples = 0

        parameter_diagnostics = {}
        for name in (
            "SIM_IMU_COUNT",
            "INS_ACCOFFS_X",
            "INS_ACCSCAL_X",
            "INS_ACC2OFFS_X",
            "INS_ACC2SCAL_X",
        ):
            try:
                parameter_diagnostics[name] = self.read_parameter(name)
            except SmokeFailure:
                parameter_diagnostics[name] = "unavailable"
        raise SmokeFailure(
            "vehicle did not reach armed MAV_STATE_ACTIVE "
            f"(last system_status={last_system_status}, "
            f"last STATUSTEXT={last_status_text!r}, "
            f"parameters={parameter_diagnostics})"
        )

    def disarm(self, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        next_command = 0.0
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_command:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                next_command = now + 1.0
            self.send_override((NEUTRAL_PWM,) * 4)
            heartbeat = self.master.recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.3
            )
            if heartbeat is not None and not (heartbeat.base_mode & armed_flag):
                return
        raise SmokeFailure("vehicle did not disarm")

    def _request_messages(self) -> None:
        for message_id, frequency_hz in (
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20),
            (mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 30),
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 5),
        ):
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                int(1_000_000 / frequency_hz),
                0,
                0,
                0,
                0,
                0,
            )

        # MAVProxy requests legacy data streams when it connects.  EXTRA3
        # contains WIND even though MotorBlimp has no wind estimator, so keep
        # this request as a regression gate for the vehicle-specific handler.
        self.master.mav.request_data_stream_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
            2,
            1,
        )

    def read_parameter(self, name: str, timeout: float = 1.5) -> float:
        encoded_name = name.encode("ascii")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.master.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                encoded_name,
                -1,
            )
            message = self.master.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.3
            )
            if message is None:
                continue
            parameter_id = message.param_id
            if isinstance(parameter_id, bytes):
                parameter_id = parameter_id.decode("ascii", errors="replace")
            if str(parameter_id).rstrip("\x00") == name:
                return float(message.param_value)
        raise SmokeFailure(f"parameter {name} was not received")

    def set_parameter(self, name: str, value: float, timeout: float = 3.0) -> float:
        encoded_name = name.encode("ascii")
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_send:
                self.master.mav.param_set_send(
                    self.target_system,
                    self.target_component,
                    encoded_name,
                    value,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
                next_send = now + 0.5
            message = self.master.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.2
            )
            if message is None:
                continue
            parameter_id = message.param_id
            if isinstance(parameter_id, bytes):
                parameter_id = parameter_id.decode("ascii", errors="replace")
            if str(parameter_id).rstrip("\x00") != name:
                continue
            actual = float(message.param_value)
            if abs(actual - value) <= 1.0e-3:
                return actual
        raise SmokeFailure(f"parameter {name} did not change to {value}")

    def _wait_until_ready(self, initial_heartbeat):
        deadline = time.monotonic() + self.startup_timeout
        next_override = 0.0
        last_heartbeat = initial_heartbeat
        ready_text = False
        attitude_samples = 0
        last_status_text = "<none>"

        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_override:
                self.send_override((NEUTRAL_PWM,) * 4)
                next_override = now + 0.08

            message = self.master.recv_match(
                type=[
                    "STATUSTEXT",
                    "ATTITUDE",
                    "EKF_STATUS_REPORT",
                    "SYS_STATUS",
                    "HEARTBEAT",
                ],
                blocking=True,
                timeout=0.2,
            )
            if message is None:
                continue
            message_type = message.get_type()
            if message_type == "STATUSTEXT":
                last_status_text = str(message.text)
                text = last_status_text.lower()
                if "motorblimp ready" in text and not ready_text:
                    ready_text = True
                    # Pre-init attitude traffic does not prove that the
                    # post-init control scheduler is running.
                    attitude_samples = 0
            elif message_type == "ATTITUDE":
                if ready_text:
                    attitude_samples += 1
            elif message_type == "HEARTBEAT":
                last_heartbeat = message

            # The explicit ready text proves init_ardupilot() finished; several
            # attitude samples prove the fast estimator/scheduler loop is live.
            if ready_text and attitude_samples >= 3:
                return last_heartbeat

        raise SmokeFailure(
            "firmware did not become ready "
            f"(ready_text={ready_text}, attitude_samples={attitude_samples}, "
            f"last STATUSTEXT={last_status_text!r})"
        )

    def _check_process(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            raise SmokeFailure(f"SITL exited with status {self.process.returncode}")

    def send_override(self, rc_pwm: Sequence[int]) -> None:
        channels = [PWM_UNCHANGED] * self.override_channel_count
        channels[:4] = rc_pwm
        try:
            self.master.mav.rc_channels_override_send(
                self.target_system,
                self.target_component,
                *channels,
            )
        except TypeError:
            # Compatibility with old pymavlink dialects that expose only the
            # original eight RC override fields.
            self.override_channel_count = 8
            channels = channels[:8]
            self.master.mav.rc_channels_override_send(
                self.target_system,
                self.target_component,
                *channels,
            )

    def collect(self, duration: float, rc_pwm: Sequence[int]) -> Dict[str, object]:
        deadline = time.monotonic() + duration
        next_override = 0.0
        servo = None
        state = None
        rc_channels = None
        heartbeat = None
        sys_status = None
        status_texts: List[str] = []
        states: List[object] = []
        local_positions: List[object] = []
        servo_samples: List[Tuple[float, object]] = []
        state_samples: List[Tuple[float, object]] = []

        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_override:
                self.send_override(rc_pwm)
                next_override = now + 0.08

            message = self.master.recv_match(
                type=[
                    "SERVO_OUTPUT_RAW",
                    "SIM_STATE",
                    "RC_CHANNELS",
                    "LOCAL_POSITION_NED",
                    "HEARTBEAT",
                    "SYS_STATUS",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=0.05,
            )
            if message is None:
                continue
            if message.get_type() == "SERVO_OUTPUT_RAW":
                servo = message
                servo_samples.append((now, message))
            elif message.get_type() == "SIM_STATE":
                state = message
                states.append(message)
                state_samples.append((now, message))
            elif message.get_type() == "RC_CHANNELS":
                rc_channels = message
            elif message.get_type() == "LOCAL_POSITION_NED":
                local_positions.append(message)
            elif message.get_type() == "HEARTBEAT":
                heartbeat = message
            elif message.get_type() == "SYS_STATUS":
                sys_status = message
            elif message.get_type() == "STATUSTEXT":
                status_texts.append(str(message.text))

        return {
            "servo": servo,
            "state": state,
            "states": states,
            "rc_channels": rc_channels,
            "heartbeat": heartbeat,
            "sys_status": sys_status,
            "status_texts": status_texts,
            "local_positions": local_positions,
            "servo_samples": servo_samples,
            "state_samples": state_samples,
        }

    def send_guided_target_ned(self, target_ned: Sequence[float]) -> None:
        ignore_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            ignore_mask,
            float(target_ned[0]),
            float(target_ned[1]),
            float(target_ned[2]),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def wait_for_healthy_ekf(self, timeout: float) -> int:
        required_flags = (
            mavutil.mavlink.ESTIMATOR_ATTITUDE
            | mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ
            | mavutil.mavlink.ESTIMATOR_VELOCITY_VERT
        )
        horizontal_position_flags = (
            mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL
            | mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS
        )
        vertical_position_flags = (
            mavutil.mavlink.ESTIMATOR_POS_VERT_ABS
            | mavutil.mavlink.ESTIMATOR_POS_VERT_AGL
        )
        deadline = time.monotonic() + timeout
        next_override = 0.0
        last_flags = 0
        last_status_text = "<none>"
        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_override:
                self.send_override((NEUTRAL_PWM,) * 4)
                next_override = now + 0.08
            message = self.master.recv_match(
                type=["EKF_STATUS_REPORT", "STATUSTEXT"],
                blocking=True,
                timeout=0.2,
            )
            if message is None:
                continue
            if message.get_type() == "STATUSTEXT":
                last_status_text = str(message.text)
                continue
            last_flags = int(message.flags)
            if (
                last_flags & required_flags == required_flags
                and last_flags & horizontal_position_flags
                and last_flags & vertical_position_flags
                and not last_flags & mavutil.mavlink.ESTIMATOR_CONST_POS_MODE
            ):
                return last_flags
        raise SmokeFailure(
            f"EKF did not become healthy in {timeout}s "
            f"(flags=0x{last_flags:04x}, required=0x{required_flags:04x}, "
            f"last STATUSTEXT={last_status_text!r})"
        )

    def set_custom_mode(self, custom_mode: int, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        next_command = 0.0
        last_mode = None
        while time.monotonic() < deadline:
            self._check_process()
            now = time.monotonic()
            if now >= next_command:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                next_command = now + 1.0
            self.send_override((NEUTRAL_PWM,) * 4)
            heartbeat = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
            if heartbeat is None:
                continue
            last_mode = int(heartbeat.custom_mode)
            if last_mode == custom_mode:
                return
        raise SmokeFailure(
            f"custom mode {custom_mode} was not accepted in {timeout}s "
            f"(last mode={last_mode})"
        )

    def stop(self) -> None:
        if self.master is not None:
            try:
                self.send_override((NEUTRAL_PWM,) * 4)
            except Exception:
                pass
            self.master.close()
            self.master = None

        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.process = None

        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    def log_tail(self, lines: int = 80) -> str:
        if self.log_handle is not None:
            self.log_handle.flush()
        try:
            content = self.log_path.read_text(errors="replace").splitlines()
        except OSError:
            return "<SITL log unavailable>"
        return "\n".join(content[-lines:])


def motor_pwm(servo_message) -> Tuple[int, int, int, int]:
    if servo_message is None:
        raise SmokeFailure("SERVO_OUTPUT_RAW was not received")
    return tuple(int(getattr(servo_message, field)) for field in MOTOR_FIELDS)


def find_compass_use_parameter(session: SITLSession) -> Tuple[str, float]:
    """Return the compass-for-yaw parameter used by this ArduPilot build."""

    errors = []
    for name in ("COMPASS_USE", "COMPASS_USE1"):
        try:
            return name, session.read_parameter(name, timeout=1.0)
        except SmokeFailure as exc:
            errors.append(str(exc))
    raise SmokeFailure(
        "neither COMPASS_USE nor COMPASS_USE1 is registered "
        f"({'; '.join(errors)})"
    )


def write_manual_no_compass_defaults(source: Path, destination: Path) -> None:
    """Create a cold-start profile with no usable simulated magnetometer."""

    overrides = """

# MANUAL no-compass regression profile.  Keep COMPASS_USE enabled so normal
# AP_Arming compass health checks would reject the failed sensor unless the
# vehicle-specific MANUAL policy is active.  EKF yaw is deliberately unaided.
SIM_MAG1_FAIL 1
SIM_MAG2_FAIL 1
SIM_MAG3_FAIL 1
COMPASS_USE 1
EK3_SRC1_YAW 0
"""
    destination.write_text(
        source.read_text(encoding="utf-8") + overrides,
        encoding="utf-8",
    )


def write_manual_unaided_yaw_defaults(source: Path, destination: Path) -> None:
    """Create a profile with healthy compasses not selected by EKF yaw."""

    overrides = """

# The magnetometers remain healthy, but the active EKF yaw source is None.
# Compass-required modes must check estimator use, not only sensor health.
COMPASS_USE 1
EK3_SRC1_YAW 0
"""
    destination.write_text(
        source.read_text(encoding="utf-8") + overrides,
        encoding="utf-8",
    )


def assert_mode_rejected_for_compass(
    session: SITLSession,
    label: str,
    requested_mode: int,
    timeout: float = 4.0,
) -> None:
    """Require a compass-specific mode rejection while MANUAL stays active."""

    deadline = time.monotonic() + timeout
    next_command = 0.0
    last_mode = None
    last_status_text = "<none>"
    compass_rejection_seen = False
    manual_active_samples = 0
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    # Do not let a delayed rejection from the preceding mode satisfy this
    # mode's compass-specific reason check.
    while session.master.recv_match(type="STATUSTEXT", blocking=False) is not None:
        pass

    while time.monotonic() < deadline:
        session._check_process()
        now = time.monotonic()
        if now >= next_command:
            session.master.mav.command_long_send(
                session.target_system,
                session.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                requested_mode,
                0,
                0,
                0,
                0,
                0,
            )
            next_command = now + 1.0
        session.send_override((NEUTRAL_PWM,) * 4)

        message = session.master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.2
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            last_status_text = str(message.text)
            text = last_status_text.lower()
            compass_rejection_seen |= "compass" in text and "reject" in text
            continue

        last_mode = int(message.custom_mode)
        if last_mode == requested_mode:
            raise SmokeFailure(
                f"{label} was accepted while compass yaw use was disabled"
            )
        if (
            last_mode == MANUAL_MODE
            and message.base_mode & armed_flag
            and int(message.system_status) == mavutil.mavlink.MAV_STATE_ACTIVE
        ):
            manual_active_samples += 1
        else:
            manual_active_samples = 0

        if compass_rejection_seen and manual_active_samples >= 2:
            print(
                f"  {label} rejected without compass; MANUAL remains ACTIVE",
                flush=True,
            )
            return

    raise SmokeFailure(
        f"{label} was not rejected specifically because of the compass "
        f"(last mode={last_mode}, last STATUSTEXT={last_status_text!r}, "
        f"compass rejection={compass_rejection_seen}, "
        f"MANUAL ACTIVE samples={manual_active_samples})"
    )


def wait_for_magnetometer_health(
    session: SITLSession,
    timeout: float = 5.0,
):
    """Return SYS_STATUS proving that the physical mag frontend is healthy."""

    magnetometer_mask = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    deadline = time.monotonic() + timeout
    last_status = None
    while time.monotonic() < deadline:
        telemetry = session.collect(0.4, (NEUTRAL_PWM,) * 4)
        last_status = telemetry["sys_status"]
        if last_status is None:
            continue
        present = int(last_status.onboard_control_sensors_present)
        enabled = int(last_status.onboard_control_sensors_enabled)
        healthy = int(last_status.onboard_control_sensors_health)
        if (
            present & magnetometer_mask
            and enabled & magnetometer_mask
            and healthy & magnetometer_mask
        ):
            return last_status
    raise SmokeFailure(
        "magnetometer frontend did not report healthy in SYS_STATUS "
        f"(last={last_status})"
    )


def input_diagnostics(telemetry: Dict[str, object]) -> str:
    rc_channels = telemetry["rc_channels"]
    if rc_channels is None:
        rc_values = "unavailable"
    else:
        rc_values = str(
            tuple(int(getattr(rc_channels, f"chan{index}_raw")) for index in range(1, 5))
        )
    heartbeat = telemetry["heartbeat"]
    if heartbeat is None:
        armed = "unavailable"
    else:
        armed = str(
            bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        )
    status_texts = telemetry["status_texts"]
    status = status_texts[-1] if status_texts else "<none>"
    state = telemetry["state"]
    if state is None:
        motion = "state=unavailable"
    else:
        motion = (
            "rpy="
            f"({float(state.roll):.2f},{float(state.pitch):.2f},{float(state.yaw):.2f}), "
            "gyro="
            f"({float(state.xgyro):.2f},{float(state.ygyro):.2f},"
            f"{float(state.zgyro):.2f})"
        )
    return f"RC={rc_values}, armed={armed}, {motion}, status={status!r}"


def assert_pattern(
    label: str,
    servo_message,
    pattern: Iterable[int],
    active_threshold: int = 80,
    neutral_tolerance: int = 65,
) -> None:
    values = motor_pwm(servo_message)
    for index, (value, expected_sign) in enumerate(zip(values, pattern), start=1):
        delta = value - NEUTRAL_PWM
        if expected_sign > 0 and delta < active_threshold:
            raise SmokeFailure(f"{label}: motor {index} PWM {value} is not forward")
        if expected_sign < 0 and delta > -active_threshold:
            raise SmokeFailure(f"{label}: motor {index} PWM {value} is not reverse")
        if expected_sign == 0 and abs(delta) > neutral_tolerance:
            raise SmokeFailure(f"{label}: motor {index} PWM {value} is not neutral")
    print(f"  {label:9s} PWM={values}", flush=True)


def run_in_session(
    label: str,
    args: argparse.Namespace,
    root: Path,
    port: int,
    callback,
    defaults: Optional[Path] = None,
) -> None:
    session = SITLSession(
        args.binary,
        defaults if defaults is not None else args.defaults,
        root / label,
        port,
        args.home,
        args.startup_timeout,
    )
    try:
        session.start()
        callback(session)
    except Exception as exc:
        raise SmokeFailure(f"{label}: {exc}\n--- SITL log ---\n{session.log_tail()}") from exc
    finally:
        session.stop()


def test_healthy_compass_without_yaw_aiding(session: SITLSession) -> None:
    """Reject heading modes when the EKF does not use a healthy compass."""

    compass_parameter, compass_use = find_compass_use_parameter(session)
    yaw_source = session.read_parameter("EK3_SRC1_YAW")
    mag_failures = tuple(
        session.read_parameter(f"SIM_MAG{index}_FAIL") for index in range(1, 4)
    )
    if abs(compass_use - 1.0) > 1.0e-3 or abs(yaw_source) > 1.0e-3:
        raise SmokeFailure(
            "invalid unaided-yaw profile "
            f"({compass_parameter}={compass_use}, EK3_SRC1_YAW={yaw_source})"
        )
    if any(abs(value) > 1.0e-3 for value in mag_failures):
        raise SmokeFailure(
            f"unaided-yaw profile unexpectedly failed a magnetometer {mag_failures}"
        )

    status = wait_for_magnetometer_health(session)
    yaw_position_mask = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_YAW_POSITION
    if int(status.onboard_control_sensors_enabled) & yaw_position_mask:
        raise SmokeFailure("MANUAL incorrectly reports yaw-position control enabled")
    if int(status.onboard_control_sensors_health) & yaw_position_mask:
        raise SmokeFailure("MANUAL incorrectly reports yaw-position control healthy")
    print(
        "  physical magnetometer healthy, EKF yaw unaided, "
        "MANUAL yaw-position telemetry disabled",
        flush=True,
    )

    for label, requested_mode in COMPASS_REQUIRED_MODES:
        assert_mode_rejected_for_compass(session, label, requested_mode)


def test_mixer(session: SITLSession, command_pwm: int) -> None:
    registered_parameters = {
        name: session.read_parameter(name)
        for name in (
            "SERIAL1_PROTOCOL",
            "LOG_DISARMED",
            "STAT_RUNTIME",
            "SIM_MB_MASS",
        )
    }
    print(f"  registered parameters={registered_parameters}", flush=True)
    if int(round(registered_parameters["SERIAL1_PROTOCOL"])) != 2:
        raise SmokeFailure(
            "SERIAL1_PROTOCOL did not load the common vehicle default "
            f"(expected=2, actual={registered_parameters['SERIAL1_PROTOCOL']})"
        )

    neutral = (NEUTRAL_PWM,) * 4

    # session.start() has already exercised the ordinary pre-arm path from a
    # cold boot.  The profile keeps compass use enabled but supplies no data,
    # while EKF yaw remains gyro-only.
    compass_parameter, compass_use = find_compass_use_parameter(session)
    mag_failures = tuple(
        session.read_parameter(f"SIM_MAG{index}_FAIL") for index in range(1, 4)
    )
    yaw_source = session.read_parameter("EK3_SRC1_YAW")
    if abs(compass_use - 1.0) > 1.0e-3:
        raise SmokeFailure(
            f"{compass_parameter} is not enabled in the no-compass profile "
            f"({compass_use})"
        )
    if (
        any(abs(value - 1.0) > 1.0e-3 for value in mag_failures)
        or abs(yaw_source) > 1.0e-3
    ):
        raise SmokeFailure(
            "invalid no-compass profile "
            f"(SIM_MAG failures={mag_failures}, EK3_SRC1_YAW={yaw_source})"
        )
    telemetry = session.collect(0.8, neutral)
    heartbeat = telemetry["heartbeat"]
    if heartbeat is None:
        raise SmokeFailure("no heartbeat after compass-failed MANUAL arm")
    if int(heartbeat.custom_mode) != MANUAL_MODE:
        raise SmokeFailure(
            "compass-failed arm did not remain in MANUAL "
            f"(custom_mode={heartbeat.custom_mode})"
        )
    if int(heartbeat.system_status) != mavutil.mavlink.MAV_STATE_ACTIVE:
        raise SmokeFailure(
            "compass-failed MANUAL is not MAV_STATE_ACTIVE "
            f"(system_status={heartbeat.system_status})"
        )
    print(
        "  MANUAL cold-start armed ACTIVE with failed compass, "
        f"{compass_parameter}=1 and EK3_SRC1_YAW=0",
        flush=True,
    )

    for label, requested_mode in COMPASS_REQUIRED_MODES:
        assert_mode_rejected_for_compass(session, label, requested_mode)

    telemetry = session.collect(1.0, neutral)
    if telemetry["state"] is None:
        raise SmokeFailure("SIM_STATE was not received")
    print(f"  {'neutral':9s} {input_diagnostics(telemetry)}", flush=True)
    assert_pattern(
        "neutral", telemetry["servo"], (0, 0, 0, 0), neutral_tolerance=8
    )

    cases = (
        ("forward", (1500, 1500, command_pwm, 1500), (+1, +1, +1, +1)),
        ("backward", (1500, 1500, 3000 - command_pwm, 1500), (-1, -1, -1, -1)),
        ("roll", (command_pwm, 1500, 1500, 1500), (+1, -1, +1, -1)),
        ("pitch", (1500, command_pwm, 1500, 1500), (-1, 0, +1, 0)),
        ("yaw", (1500, 1500, 1500, command_pwm), (0, -1, 0, +1)),
    )
    for label, rc_pwm, expected in cases:
        telemetry = session.collect(0.7, rc_pwm)
        print(f"  {label:9s} {input_diagnostics(telemetry)}", flush=True)
        # Once the linear model has begun moving, the closed-loop attitude
        # controller legitimately adds modest cross-axis corrections to
        # nominally-zero mixer terms.  Active-axis signs remain strict.
        if label != "yaw":
            assert_pattern(
                label,
                telemetry["servo"],
                expected,
                active_threshold=80,
                neutral_tolerance=100,
            )
        else:
            # A rate controller legitimately removes most yaw torque as the
            # body approaches the requested angular rate.  Inspect the peak
            # transient differential rather than demanding sustained torque
            # from the final sample.
            pwm_samples = [
                motor_pwm(message) for _, message in telemetry["servo_samples"]
            ]
            if not pwm_samples:
                raise SmokeFailure("yaw: no SERVO_OUTPUT_RAW samples")
            peak_values = max(pwm_samples, key=lambda values: values[3] - values[1])
            yaw_differential = peak_values[3] - peak_values[1]
            if (
                yaw_differential <= 120
                or peak_values[1] >= NEUTRAL_PWM - 50
                or peak_values[3] <= NEUTRAL_PWM + 50
            ):
                raise SmokeFailure(
                    "yaw: peak M4-M2 signed differential is too small "
                    f"({yaw_differential}us, PWM={peak_values})"
                )
            print(f"  {'yaw peak':9s} PWM={peak_values}", flush=True)
            states = telemetry["states"]
            if not states:
                raise SmokeFailure("yaw: no SIM_STATE samples during command")
            peak_yaw_rate = max(float(state.zgyro) for state in states)
            if peak_yaw_rate < 0.02:
                raise SmokeFailure(
                    "yaw: compass-independent body yaw rate is too small "
                    f"(SIM_STATE.zgyro peak={peak_yaw_rate:.6f}rad/s)"
                )
            print(
                f"  yaw body rate without compass peak={peak_yaw_rate:.4f}rad/s",
                flush=True,
            )
        session.collect(0.35, neutral)


def test_dynamics(
    session: SITLSession,
    rc_pwm: Sequence[int],
    field: str,
    minimum: float,
) -> None:
    neutral = (NEUTRAL_PWM,) * 4
    baseline = session.collect(0.8, neutral)
    if baseline["state"] is None:
        raise SmokeFailure("SIM_STATE was not received")

    telemetry = session.collect(1.0, rc_pwm)
    states = telemetry["states"]
    if not states:
        raise SmokeFailure("no SIM_STATE samples during command")
    peak = max(float(getattr(state, field)) for state in states)
    if peak < minimum:
        raise SmokeFailure(f"SIM_STATE.{field} peak {peak:.6f} is below {minimum:.6f}")
    print(f"  dynamics {field:5s} peak={peak:.4f}", flush=True)


def test_guided_3d_climb(
    session: SITLSession,
    ekf_timeout: float,
    guided_mode_id: int,
) -> None:
    flags = session.wait_for_healthy_ekf(ekf_timeout)
    print(f"  EKF healthy flags=0x{flags:04x}", flush=True)
    session.set_custom_mode(guided_mode_id)
    print(f"  GUIDED custom_mode={guided_mode_id}", flush=True)

    neutral = (NEUTRAL_PWM,) * 4
    baseline = session.collect(0.8, neutral)
    if not baseline["local_positions"] or not baseline["states"]:
        raise SmokeFailure(
            "LOCAL_POSITION_NED/SIM_STATE was not received before 3D target"
        )
    start_position = baseline["local_positions"][-1]
    start_z = float(start_position.z)
    start_truth_alt = float(baseline["states"][-1].alt)

    # Start from a cold, stationary model and make the forward component
    # dominant.  With no body-Y/body-Z actuators the aircraft must first pitch
    # nose-up, then apply positive body-X thrust to gain real altitude.
    target_3d = (
        float(start_position.x) + 5.0,
        float(start_position.y),
        start_z - 1.0,
    )
    session.send_guided_target_ned(target_3d)
    telemetry_3d = session.collect(18.0, neutral)
    positions_3d = telemetry_3d["local_positions"]
    state_samples = telemetry_3d["state_samples"]
    servo_samples = telemetry_3d["servo_samples"]
    if not positions_3d or not state_samples or not servo_samples:
        raise SmokeFailure("3D GUIDED target produced incomplete telemetry")

    climb_up = start_z - min(float(position.z) for position in positions_3d)
    truth_climb_up = max(float(state.alt) for _, state in state_samples) - start_truth_alt
    climb_disagreement = abs(climb_up - truth_climb_up)
    peak_pitch = max(float(state.pitch) for _, state in state_samples)
    collective_samples = [
        (
            sample_time,
            sum(value - NEUTRAL_PWM for value in motor_pwm(servo)) / 4.0,
        )
        for sample_time, servo in servo_samples
    ]
    first_truth_climb_time = next(
        (
            sample_time
            for sample_time, state in state_samples
            if float(state.alt) - start_truth_alt >= 0.05
        ),
        None,
    )
    if first_truth_climb_time is None:
        raise SmokeFailure(
            "3D GUIDED target did not establish thrust-driven climb "
            f"(local_climb={climb_up:.3f}m, truth_climb={truth_climb_up:.3f}m, "
            f"peak_pitch={peak_pitch:.3f}rad, "
            f"collective_pwm=[{min(value for _, value in collective_samples):.1f},"
            f"{max(value for _, value in collective_samples):.1f}])"
        )
    preclimb_collective = max(
        (
            collective_pwm
            for sample_time, collective_pwm in collective_samples
            if sample_time <= first_truth_climb_time
        ),
        default=-1000.0,
    )
    preclimb_pitch = max(
        (
            float(state.pitch)
            for sample_time, state in state_samples
            if sample_time <= first_truth_climb_time
        ),
        default=-3.14159,
    )

    if (
        climb_up < GUIDED_MIN_ESTIMATED_CLIMB_M
        or truth_climb_up < GUIDED_MIN_TRUTH_CLIMB_M
        or climb_disagreement > GUIDED_MAX_CLIMB_DISAGREEMENT_M
        or peak_pitch < 0.12
        or preclimb_pitch < 0.04
        or preclimb_collective < GUIDED_COLLECTIVE_DETECTION_PWM
    ):
        raise SmokeFailure(
            "3D GUIDED target did not demonstrate pitch-then-thrust climb "
            f"(local_climb={climb_up:.3f}m, truth_climb={truth_climb_up:.3f}m, "
            f"climb_disagreement={climb_disagreement:.3f}m, "
            f"peak_pitch={peak_pitch:.3f}rad, "
            f"preclimb_pitch={preclimb_pitch:.3f}rad, "
            f"preclimb_collective={preclimb_collective:.1f}us)"
        )
    print(
        "  GUIDED 3D climb "
        f"local_up={climb_up:.3f}m truth_up={truth_climb_up:.3f}m "
        f"peak_pitch={peak_pitch:.3f}rad "
        f"preclimb_pitch={preclimb_pitch:.3f}rad "
        f"preclimb_collective={preclimb_collective:.1f}us",
        flush=True,
    )


def test_guided_navigation(
    session: SITLSession,
    ekf_timeout: float,
    guided_mode_id: int,
) -> None:
    flags = session.wait_for_healthy_ekf(ekf_timeout)
    print(f"  EKF healthy flags=0x{flags:04x}", flush=True)
    session.set_custom_mode(guided_mode_id)
    print(f"  GUIDED custom_mode={guided_mode_id}", flush=True)

    neutral = (NEUTRAL_PWM,) * 4
    baseline = session.collect(0.8, neutral)
    if not baseline["local_positions"]:
        raise SmokeFailure("LOCAL_POSITION_NED was not received before GUIDED target")
    start_position = baseline["local_positions"][-1]
    start_x = float(start_position.x)

    target_ned = (
        start_x + 3.0,
        float(start_position.y),
        float(start_position.z),
    )
    session.send_guided_target_ned(target_ned)
    telemetry = session.collect(10.0, neutral)
    positions = telemetry["local_positions"]
    states = telemetry["states"]
    if not positions or not states:
        raise SmokeFailure("GUIDED target produced no position/model telemetry")
    displacement_north = max(float(position.x) - start_x for position in positions)
    peak_vn = max(float(state.vn) for state in states)
    if displacement_north < 0.30 or peak_vn < 0.05:
        raise SmokeFailure(
            "GUIDED target did not move north "
            f"(displacement={displacement_north:.3f}m, peak_vn={peak_vn:.3f}m/s)"
        )
    print(
        "  GUIDED target moved "
        f"north={displacement_north:.3f}m peak_vn={peak_vn:.3f}m/s",
        flush=True,
    )

    # A GUIDED disarm must clear the persistent target. After normal re-arm,
    # outputs stay neutral until a new SET_POSITION_TARGET message arrives.
    session.disarm()
    disarmed = session.collect(0.6, neutral)
    assert_pattern("disarmed", disarmed["servo"], (0, 0, 0, 0))
    session._wait_until_active()
    rearmed = session.collect(1.0, neutral)
    assert_pattern("rearmed", rearmed["servo"], (0, 0, 0, 0))

    # Runtime loss injection: disabling the only absolute-position backend is
    # equivalent to a tag/backend outage at the vehicle boundary.  A fresh
    # target first proves that motors are active; navigation_healthy() must
    # then stop them without waiting for the EKF dead-reckoning flags to age.
    if not rearmed["local_positions"]:
        raise SmokeFailure("LOCAL_POSITION_NED unavailable before beacon-loss test")
    outage_start_position = rearmed["local_positions"][-1]
    session.send_guided_target_ned(
        (
            float(outage_start_position.x) + 8.0,
            float(outage_start_position.y),
            float(outage_start_position.z),
        )
    )
    activation_deadline = time.monotonic() + 3.0
    next_override = 0.0
    peak_active_delta = 0
    while time.monotonic() < activation_deadline:
        session._check_process()
        now = time.monotonic()
        if now >= next_override:
            session.send_override(neutral)
            next_override = now + 0.08
        servo = session.master.recv_match(
            type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.1
        )
        if servo is None:
            continue
        peak_active_delta = max(
            peak_active_delta,
            *(abs(value - NEUTRAL_PWM) for value in motor_pwm(servo)),
        )
        if peak_active_delta >= 80:
            break
    else:
        raise SmokeFailure(
            "beacon-loss test did not establish active motors "
            f"(peak_delta={peak_active_delta}us)"
        )

    session.set_parameter("BCN_TYPE", 0.0)
    # Start the externally-observable latency bound from PARAM_VALUE ACK;
    # MAVLink request/transport latency is not part of the vehicle failsafe.
    loss_started = time.monotonic()
    # Motor safety is checked against a strict deadline.  HEARTBEAT and
    # SYS_STATUS are periodic telemetry, so allow a longer observation window
    # for their externally-visible state to catch up.
    deadline = loss_started + 2.5
    last_values = None
    neutral_elapsed = None
    critical_seen = False
    position_unhealthy_seen = False
    while time.monotonic() < deadline:
        session._check_process()
        session.send_override(neutral)
        message = session.master.recv_match(
            type=["SERVO_OUTPUT_RAW", "HEARTBEAT", "SYS_STATUS", "STATUSTEXT"],
            blocking=True,
            timeout=0.1,
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"  {message.text}", flush=True)
            continue
        if message.get_type() == "HEARTBEAT":
            critical_seen |= (
                int(message.system_status) == mavutil.mavlink.MAV_STATE_CRITICAL
            )
        elif message.get_type() == "SYS_STATUS":
            enabled = int(message.onboard_control_sensors_enabled)
            health = int(message.onboard_control_sensors_health)
            position_unhealthy_seen |= (
                enabled & POSITION_CONTROL_SENSOR_MASK
            ) == POSITION_CONTROL_SENSOR_MASK and (
                health & POSITION_CONTROL_SENSOR_MASK
            ) == 0
        else:
            last_values = motor_pwm(message)
            if (
                neutral_elapsed is None
                and all(abs(value - NEUTRAL_PWM) <= 8 for value in last_values)
            ):
                neutral_elapsed = time.monotonic() - loss_started

        if neutral_elapsed is not None and critical_seen and position_unhealthy_seen:
            break

    if neutral_elapsed is None or neutral_elapsed > 0.5:
        raise SmokeFailure(
            "motors did not reach neutral within 0.5s of beacon loss "
            f"(last PWM={last_values}, neutral_elapsed={neutral_elapsed})"
        )
    if not critical_seen or not position_unhealthy_seen:
        raise SmokeFailure(
            "telemetry did not report beacon loss within 2.5s "
            f"(critical={critical_seen}, position_unhealthy={position_unhealthy_seen})"
        )
    print(
        "  beacon loss neutral: "
        f"elapsed={neutral_elapsed:.3f}s PWM={last_values}, "
        "MAV_STATE_CRITICAL, SYS_STATUS position unhealthy",
        flush=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--defaults", required=True, type=Path)
    parser.add_argument("--home", default="55.751244,37.618423,200,0")
    parser.add_argument("--base-port", type=int, default=5760)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--command-pwm", type=int, default=1900)
    parser.add_argument(
        "--check-guided",
        action="store_true",
        help="also require healthy beacon/compass EKF and transition to GUIDED",
    )
    parser.add_argument("--guided-mode-id", type=int, default=15)
    parser.add_argument("--ekf-timeout", type=float, default=45.0)
    parser.add_argument(
        "--skip-dynamics",
        action="store_true",
        help="skip the four separate 6DoF response cases",
    )
    parser.add_argument(
        "--skip-mixer",
        action="store_true",
        help="skip the manual mixer case (useful while iterating GUIDED tests)",
    )
    args = parser.parse_args()
    args.binary = args.binary.expanduser().resolve()
    args.defaults = args.defaults.expanduser().resolve()
    if not args.binary.is_file():
        parser.error(f"binary does not exist: {args.binary}")
    if not args.defaults.is_file():
        parser.error(f"defaults file does not exist: {args.defaults}")
    if not 1600 <= args.command_pwm <= 2000:
        parser.error("--command-pwm must be in [1600, 2000]")
    return args


def main() -> int:
    args = parse_args()
    print(f"binary:  {args.binary}")
    print(f"defaults: {args.defaults}")

    with tempfile.TemporaryDirectory(prefix="motorblimp-sitl-") as temp_dir:
        root = Path(temp_dir)
        if not args.skip_mixer:
            manual_no_compass_defaults = root / "manual-no-compass.parm"
            write_manual_no_compass_defaults(
                args.defaults, manual_no_compass_defaults
            )
            run_in_session(
                "mixer",
                args,
                root,
                args.base_port,
                lambda session: test_mixer(session, args.command_pwm),
                defaults=manual_no_compass_defaults,
            )

            manual_unaided_yaw_defaults = root / "manual-unaided-yaw.parm"
            write_manual_unaided_yaw_defaults(
                args.defaults, manual_unaided_yaw_defaults
            )
            run_in_session(
                "healthy-compass-unaided-yaw",
                args,
                root,
                args.base_port + 70,
                test_healthy_compass_without_yaw_aiding,
                defaults=manual_unaided_yaw_defaults,
            )

        if not args.skip_dynamics:
            dynamics_cases = (
                ("forward", (1500, 1500, args.command_pwm, 1500), "vn", 0.05),
                ("roll", (args.command_pwm, 1500, 1500, 1500), "xgyro", 0.003),
                ("pitch", (1500, args.command_pwm, 1500, 1500), "ygyro", 0.02),
                ("yaw", (1500, 1500, 1500, args.command_pwm), "zgyro", 0.02),
            )
            for offset, (label, rc_pwm, field, minimum) in enumerate(dynamics_cases, start=1):
                run_in_session(
                    f"dynamics-{label}",
                    args,
                    root,
                    args.base_port + offset * 10,
                    lambda session, rc=rc_pwm, fld=field, threshold=minimum: test_dynamics(
                        session, rc, fld, threshold
                    ),
                )

        if args.check_guided:
            run_in_session(
                "guided-climb",
                args,
                root,
                args.base_port + 50,
                lambda session: test_guided_3d_climb(
                    session,
                    args.ekf_timeout,
                    args.guided_mode_id,
                ),
            )
            run_in_session(
                "guided-horizontal",
                args,
                root,
                args.base_port + 60,
                lambda session: test_guided_navigation(
                    session,
                    args.ekf_timeout,
                    args.guided_mode_id,
                ),
            )

    print("ArduMotorBlimp SITL smoke test: PASS", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SmokeFailure as exc:
        print(f"ArduMotorBlimp SITL smoke test: FAIL\n{exc}", file=sys.stderr)
        sys.exit(1)
