#!/usr/bin/env python3
"""End-to-end AUTO waypoint and final-hold test for MotorBlimp SITL."""

from __future__ import annotations

import argparse
from collections import deque
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Deque, Optional, Tuple

from pymavlink import mavutil

from sitl_smoke import NEUTRAL_PWM, SITLSession, SmokeFailure


NEUTRAL = (NEUTRAL_PWM,) * 4
EARTH_METRES_PER_DEGREE_LATITUDE = 111_319.49079327357
AUTO_MODE = 10
EXPECTED_SERIAL1_PROTOCOL = 2
HOME_HORIZONTAL_TOLERANCE_M = 2.0
HOME_ALTITUDE_TOLERANCE_M = 3.0


def request_message(session: SITLSession, message_id: int, rate_hz: int) -> None:
    session.master.mav.command_long_send(
        session.target_system,
        session.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000 / rate_hz),
        0,
        0,
        0,
        0,
        0,
    )


def wait_for_position(session: SITLSession, timeout: float):
    request_message(session, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    request_message(session, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    deadline = time.monotonic() + timeout
    next_override = 0.0
    global_position = None
    local_position = None
    while time.monotonic() < deadline:
        session._check_process()
        now = time.monotonic()
        if now >= next_override:
            session.send_override(NEUTRAL)
            next_override = now + 0.08
        message = session.master.recv_match(
            type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=0.2,
        )
        if message is None:
            continue
        if message.get_type() == "GLOBAL_POSITION_INT":
            global_position = message
        else:
            local_position = message
        if global_position is not None and local_position is not None:
            return global_position, local_position
    raise SmokeFailure("GLOBAL_POSITION_INT/LOCAL_POSITION_NED unavailable")


def horizontal_distance_m(
    latitude_a_e7: int,
    longitude_a_e7: int,
    latitude_b_e7: int,
    longitude_b_e7: int,
) -> float:
    latitude_a = math.radians(latitude_a_e7 * 1.0e-7)
    latitude_b = math.radians(latitude_b_e7 * 1.0e-7)
    delta_latitude = latitude_b - latitude_a
    delta_longitude = math.radians((longitude_b_e7 - longitude_a_e7) * 1.0e-7)
    mean_latitude = 0.5 * (latitude_a + latitude_b)
    return EARTH_METRES_PER_DEGREE_LATITUDE * math.degrees(
        math.hypot(delta_latitude, delta_longitude * math.cos(mean_latitude))
    )


def wait_for_home(
    session: SITLSession,
    expected: Tuple[int, int, float],
    timeout: float,
):
    """Request and validate the HOME_POSITION established from the UWB origin."""

    deadline = time.monotonic() + timeout
    next_request = 0.0
    last_ack = None
    while time.monotonic() < deadline:
        session._check_process()
        now = time.monotonic()
        if now >= next_request:
            session.master.mav.command_long_send(
                session.target_system,
                session.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            next_request = now + 1.0

        message = session.master.recv_match(
            type=["HOME_POSITION", "COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=0.25,
        )
        if message is None:
            continue
        if message.get_type() == "COMMAND_ACK":
            if int(message.command) == mavutil.mavlink.MAV_CMD_GET_HOME_POSITION:
                last_ack = int(message.result)
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"  {message.text}", flush=True)
            continue

        expected_latitude, expected_longitude, expected_altitude_m = expected
        horizontal_error = horizontal_distance_m(
            expected_latitude,
            expected_longitude,
            int(message.latitude),
            int(message.longitude),
        )
        altitude_error = abs(float(message.altitude) * 1.0e-3 - expected_altitude_m)
        if horizontal_error > HOME_HORIZONTAL_TOLERANCE_M:
            raise SmokeFailure(
                "HOME_POSITION is not at the configured UWB origin "
                f"(horizontal_error={horizontal_error:.3f}m)"
            )
        if altitude_error > HOME_ALTITUDE_TOLERANCE_M:
            raise SmokeFailure(
                "HOME_POSITION altitude does not match --home "
                f"(altitude_error={altitude_error:.3f}m)"
            )
        print(
            "  HOME_POSITION valid: "
            f"horizontal_error={horizontal_error:.3f}m, "
            f"altitude_error={altitude_error:.3f}m",
            flush=True,
        )
        return message

    raise SmokeFailure(
        "HOME_POSITION unavailable after disarm "
        f"(last MAV_CMD_GET_HOME_POSITION result={last_ack})"
    )


def wait_for_mission_ack(session: SITLSession, timeout: float, operation: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        session._check_process()
        message = session.master.recv_match(
            type=["MISSION_ACK", "STATUSTEXT"], blocking=True, timeout=0.2
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"  {message.text}", flush=True)
            continue
        result = int(message.type)
        if result != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise SmokeFailure(
                f"{operation} rejected with MAV_MISSION_RESULT={result}"
            )
        return
    raise SmokeFailure(f"timeout waiting for {operation} MISSION_ACK")


def send_waypoint_item(
    session: SITLSession,
    seq: int,
    latitude_e7: int,
    longitude_e7: int,
    altitude_m: float,
) -> None:
    session.master.mav.mission_item_int_send(
        session.target_system,
        session.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        1,
        0.0,
        0.0,
        0.0,
        math.nan,
        latitude_e7,
        longitude_e7,
        altitude_m,
    )


def upload_mission(
    session: SITLSession,
    home: Tuple[int, int, float],
    waypoint: Tuple[int, int, float],
) -> None:
    session.master.mav.mission_clear_all_send(
        session.target_system, session.target_component
    )
    wait_for_mission_ack(session, 5.0, "mission clear")

    session.master.mav.mission_count_send(
        session.target_system, session.target_component, 2
    )
    sent = set()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        session._check_process()
        message = session.master.recv_match(
            type=[
                "MISSION_REQUEST_INT",
                "MISSION_REQUEST",
                "MISSION_ACK",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=0.3,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "STATUSTEXT":
            print(f"  {message.text}", flush=True)
            continue
        if message_type == "MISSION_ACK":
            result = int(message.type)
            if result != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise SmokeFailure(
                    f"mission upload rejected with MAV_MISSION_RESULT={result}"
                )
            if sent == {0, 1}:
                return
            continue

        seq = int(message.seq)
        if seq not in (0, 1):
            raise SmokeFailure(f"unexpected mission request seq={seq}")
        latitude_e7, longitude_e7, altitude_m = home if seq == 0 else waypoint
        send_waypoint_item(
            session, seq, latitude_e7, longitude_e7, altitude_m
        )
        sent.add(seq)

    raise SmokeFailure(f"mission upload timed out after items {sorted(sent)}")


def vector_error(position, target: Tuple[float, float, float]) -> float:
    return math.sqrt(
        (float(position.x) - target[0]) ** 2
        + (float(position.y) - target[1]) ** 2
        + (float(position.z) - target[2]) ** 2
    )


def vector_speed(position) -> float:
    return math.sqrt(
        float(position.vx) ** 2
        + float(position.vy) ** 2
        + float(position.vz) ** 2
    )


def fly_and_verify(
    session: SITLSession,
    start_position,
    north_m: float,
    flight_timeout: float,
    hold_window: float,
    hold_radius: float,
    hold_speed: float,
) -> None:
    request_message(session, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    request_message(session, mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 30)

    target = (
        float(start_position.x) + north_m,
        float(start_position.y),
        float(start_position.z),
    )
    deadline = time.monotonic() + flight_timeout
    started = time.monotonic()
    next_override = 0.0
    reached = False
    complete = False
    complete_time: Optional[float] = None
    north_peak = -math.inf
    true_vn_peak = -math.inf
    minimum_error = math.inf
    final_position = None
    samples: Deque[Tuple[float, float, float, float]] = deque()

    while time.monotonic() < deadline:
        session._check_process()
        now = time.monotonic()
        if now >= next_override:
            session.send_override(NEUTRAL)
            next_override = now + 0.08

        message = session.master.recv_match(
            type=[
                "LOCAL_POSITION_NED",
                "SIM_STATE",
                "HEARTBEAT",
                "MISSION_ITEM_REACHED",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=0.1,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            if int(message.custom_mode) != AUTO_MODE:
                raise SmokeFailure(
                    f"AUTO dropped to custom_mode={message.custom_mode}"
                )
            continue
        if message_type == "SIM_STATE":
            true_vn_peak = max(true_vn_peak, float(message.vn))
            continue
        if message_type == "MISSION_ITEM_REACHED":
            if int(message.seq) == 1:
                reached = True
                print(
                    f"  waypoint reached at t={now - started:.1f}s", flush=True
                )
            continue
        if message_type == "STATUSTEXT":
            status = str(message.text)
            if "mission complete" in status.lower():
                complete = True
                if complete_time is None:
                    complete_time = now
                print(f"  {status}", flush=True)
            continue

        final_position = message
        north = float(message.x) - float(start_position.x)
        error = vector_error(message, target)
        speed = vector_speed(message)
        north_peak = max(north_peak, north)
        minimum_error = min(minimum_error, error)
        samples.append((now, error, speed, north))
        while samples and now - samples[0][0] > hold_window:
            samples.popleft()

        if (
            not reached
            or not complete
            or complete_time is None
            or now - complete_time < hold_window
            or not samples
        ):
            continue
        window_duration = samples[-1][0] - samples[0][0]
        if window_duration < hold_window - 0.25:
            continue
        error_max = max(sample[1] for sample in samples)
        speed_max = max(sample[2] for sample in samples)
        if error_max <= hold_radius and speed_max <= hold_speed:
            break
    else:
        error_max = max((sample[1] for sample in samples), default=math.inf)
        speed_max = max((sample[2] for sample in samples), default=math.inf)
        raise SmokeFailure(
            "AUTO did not reach a stable final hold "
            f"(reached={reached}, complete={complete}, "
            f"north_peak={north_peak:.3f}m, min_error={minimum_error:.3f}m, "
            f"hold_error_max={error_max:.3f}m, "
            f"hold_speed_max={speed_max:.3f}m/s)"
        )

    if final_position is None or north_peak < 0.5 or true_vn_peak < 0.05:
        raise SmokeFailure(
            "AUTO did not produce northward flight "
            f"(north_peak={north_peak:.3f}m, true_vn_peak={true_vn_peak:.3f}m/s)"
        )

    error_max = max(sample[1] for sample in samples)
    speed_max = max(sample[2] for sample in samples)
    north_span = max(sample[3] for sample in samples) - min(
        sample[3] for sample in samples
    )
    print(
        "  AUTO moved and held: "
        f"north_peak={north_peak:.3f}m, true_vn_peak={true_vn_peak:.3f}m/s, "
        f"min_error={minimum_error:.3f}m, hold_error_max={error_max:.3f}m, "
        f"hold_speed_max={speed_max:.3f}m/s, hold_span={north_span:.3f}m",
        flush=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--defaults", required=True, type=Path)
    parser.add_argument("--home", default="55.751244,37.618423,200,0")
    parser.add_argument("--base-port", type=int, default=5860)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--ekf-timeout", type=float, default=90.0)
    parser.add_argument("--position-timeout", type=float, default=10.0)
    parser.add_argument("--flight-timeout", type=float, default=75.0)
    parser.add_argument("--waypoint-north", type=float, default=3.0)
    parser.add_argument("--hold-window", type=float, default=5.0)
    parser.add_argument("--hold-radius", type=float, default=0.6)
    parser.add_argument("--hold-speed", type=float, default=0.15)
    args = parser.parse_args()
    args.binary = args.binary.expanduser().resolve()
    args.defaults = args.defaults.expanduser().resolve()
    if not args.binary.is_file():
        parser.error(f"binary does not exist: {args.binary}")
    if not args.defaults.is_file():
        parser.error(f"defaults file does not exist: {args.defaults}")
    if args.waypoint_north <= 1.0:
        parser.error("--waypoint-north must be greater than 1m")
    return args


def main() -> int:
    args = parse_args()
    print(f"binary:  {args.binary}")
    print(f"defaults: {args.defaults}")

    with tempfile.TemporaryDirectory(prefix="motorblimp-auto-") as temporary:
        session = SITLSession(
            args.binary,
            args.defaults,
            Path(temporary),
            args.base_port,
            args.home,
            args.startup_timeout,
        )
        try:
            session.start()
            flags = session.wait_for_healthy_ekf(args.ekf_timeout)
            print(f"  EKF healthy flags=0x{flags:04x}", flush=True)
            serial1_protocol = session.read_parameter("SERIAL1_PROTOCOL")
            if int(round(serial1_protocol)) != EXPECTED_SERIAL1_PROTOCOL:
                raise SmokeFailure(
                    "SERIAL1_PROTOCOL did not load the common vehicle default "
                    f"(expected={EXPECTED_SERIAL1_PROTOCOL}, "
                    f"actual={serial1_protocol})"
                )
            print(
                f"  SERIAL1_PROTOCOL={serial1_protocol:g} registered",
                flush=True,
            )

            global_position, _ = wait_for_position(
                session, args.position_timeout
            )

            # HOME is established from the healthy UWB/global origin while
            # disarmed.  Exercise that path before uploading a conventional
            # GLOBAL_RELATIVE_ALT_INT mission, then re-arm normally.
            configured_home_fields = args.home.split(",")
            if len(configured_home_fields) < 3:
                raise SmokeFailure(f"invalid --home value: {args.home!r}")
            configured_home = (
                round(float(configured_home_fields[0]) * 1.0e7),
                round(float(configured_home_fields[1]) * 1.0e7),
                float(configured_home_fields[2]),
            )
            session.disarm()
            home_position = wait_for_home(
                session,
                configured_home,
                max(args.position_timeout, 8.0),
            )
            session._wait_until_active()
            global_position, local_position = wait_for_position(
                session, args.position_timeout
            )

            latitude_delta_e7 = round(
                args.waypoint_north
                / EARTH_METRES_PER_DEGREE_LATITUDE
                * 1.0e7
            )
            relative_altitude_m = float(global_position.relative_alt) / 1000.0
            home = (
                int(home_position.latitude),
                int(home_position.longitude),
                0.0,
            )
            waypoint = (
                int(global_position.lat) + latitude_delta_e7,
                int(global_position.lon),
                relative_altitude_m,
            )
            upload_mission(session, home, waypoint)
            print(
                "  GLOBAL_RELATIVE_ALT_INT mission upload accepted "
                "(home + one waypoint)",
                flush=True,
            )
            session.set_custom_mode(AUTO_MODE, timeout=10.0)
            print("  AUTO custom_mode=10", flush=True)
            fly_and_verify(
                session,
                local_position,
                args.waypoint_north,
                args.flight_timeout,
                args.hold_window,
                args.hold_radius,
                args.hold_speed,
            )
        except Exception as exc:
            raise SmokeFailure(
                f"AUTO: {exc}\n--- SITL log ---\n{session.log_tail()}"
            ) from exc
        finally:
            session.stop()

    print("ArduMotorBlimp AUTO SITL test: PASS", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SmokeFailure as exc:
        print(f"ArduMotorBlimp AUTO SITL test: FAIL\n{exc}", file=sys.stderr)
        sys.exit(1)
