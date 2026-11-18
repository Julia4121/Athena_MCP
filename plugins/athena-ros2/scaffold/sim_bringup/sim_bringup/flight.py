"""Small, synchronous PX4 flight actions for simulation scenario scripts.

The caller owns the ROS node and must not spin it in another executor while an
action is running. Feature assertions belong to the scenario, not this module.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import Any, ClassVar, Self


@dataclass(frozen=True)
class Pose:
    """Launch-relative north/east/up metres and optional absolute NED yaw."""

    north_m: float
    east_m: float
    up_m: float
    yaw_rad: float | None = None

    def __post_init__(self) -> None:
        values = (self.north_m, self.east_m, self.up_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pose coordinates must be finite.")
        if self.yaw_rad is not None and not math.isfinite(self.yaw_rad):
            raise ValueError("Pose yaw must be finite when supplied.")


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)


def _route_lengths(points: Sequence[tuple[float, float, float]]) -> list[float]:
    lengths = [0.0]
    for start, end in pairwise(points):
        length = _distance(start, end)
        if length <= 1e-6:
            raise ValueError(
                "Consecutive path points must have different positions; use turn_to for an in-place turn."
            )
        lengths.append(lengths[-1] + length)
    return lengths


def _route_pose(
    points: Sequence[tuple[float, float, float]],
    yaws: Sequence[float],
    lengths: Sequence[float],
    progress_m: float,
) -> tuple[tuple[float, float, float], float]:
    progress_m = min(lengths[-1], max(0.0, progress_m))
    for index in range(len(points) - 1):
        if progress_m <= lengths[index + 1] or index == len(points) - 2:
            fraction = (progress_m - lengths[index]) / (
                lengths[index + 1] - lengths[index]
            )
            position = tuple(
                a + fraction * (b - a) for a, b in zip(points[index], points[index + 1])
            )
            yaw = yaws[index] + fraction * _wrap(yaws[index + 1] - yaws[index])
            return position, _wrap(yaw)
    return points[-1], yaws[-1]


def _project_progress(
    position: tuple[float, float, float],
    points: Sequence[tuple[float, float, float]],
    lengths: Sequence[float],
    minimum_m: float,
    maximum_m: float,
) -> tuple[float, float]:
    """Project only onto nearby ordered segments, avoiding jumps at crossings."""

    best_progress = minimum_m
    best_error = math.inf
    for index, (start, end) in enumerate(pairwise(points)):
        segment_start, segment_end = lengths[index], lengths[index + 1]
        if segment_end < minimum_m or segment_start > maximum_m:
            continue
        vector = tuple(b - a for a, b in zip(start, end))
        squared = sum(component * component for component in vector)
        fraction = (
            sum((p - a) * component for p, a, component in zip(position, start, vector))
            / squared
        )
        lower = max(0.0, (minimum_m - segment_start) / (segment_end - segment_start))
        upper = min(1.0, (maximum_m - segment_start) / (segment_end - segment_start))
        fraction = max(lower, min(upper, fraction))
        projected = tuple(
            a + fraction * component for a, component in zip(start, vector)
        )
        error = _distance(position, projected)
        if error < best_error:
            best_error = error
            best_progress = segment_start + fraction * (segment_end - segment_start)
    return best_progress, best_error


class FlightSession:
    """Fly one PX4 SITL vehicle from a test-owned ROS node.

    Calls spin the node synchronously. The session keeps neutral manual input
    flowing through mode changes and uses a steady-time Offboard stream.
    """

    _mode_states: ClassVar[dict[str, str]] = {
        "offboard": "NAVIGATION_STATE_OFFBOARD",
        "position": "NAVIGATION_STATE_POSCTL",
        "altitude": "NAVIGATION_STATE_ALTCTL",
        "stabilized": "NAVIGATION_STATE_STAB",
    }
    _mode_params: ClassVar[dict[str, float]] = {
        "offboard": 6.0,
        "position": 3.0,
        "altitude": 2.0,
        "stabilized": 7.0,
    }
    _manual_input_limit: ClassVar[float] = 0.5
    _manual_takeoff_ramp_seconds: ClassVar[float] = 1.0
    _manual_takeoff_liftoff_m: ClassVar[float] = 0.15
    # PX4's state topic is slower than local position; bound loss without
    # requiring a new state sample for every fresh position update.
    _status_max_age_seconds: ClassVar[float] = 15.0

    def __init__(self, node: Any) -> None:
        import rclpy
        from px4_msgs.msg import (
            ManualControlSetpoint,
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
            VehicleCommandAck,
            VehicleLandDetected,
            VehicleLocalPosition,
            VehicleStatus,
        )

        from . import test_support

        self.node = node
        self._rclpy = rclpy
        self._support = test_support
        self._types = {
            "manual": ManualControlSetpoint,
            "offboard": OffboardControlMode,
            "setpoint": TrajectorySetpoint,
            "command": VehicleCommand,
            "ack": VehicleCommandAck,
            "land": VehicleLandDetected,
            "position": VehicleLocalPosition,
            "status": VehicleStatus,
        }
        self._clock = test_support.Px4Time()
        self._subscriptions: list[Any] = []
        self._status: Any | None = None
        self._position: Any | None = None
        self._land: Any | None = None
        self._status_received_ns = 0
        self._position_received_ns = 0
        self._land_received_ns = 0
        self._origin: tuple[float, float, float] | None = None
        self._held_yaw = 0.0
        self._mode: str | None = None
        self._manual_stream: Any | None = None
        self._offboard_stream: Any | None = None
        self._manual_publisher: Any | None = None
        self._offboard_publishers: list[Any] = []
        self._closed = False

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    @staticmethod
    def _deadline(timeout_seconds: float) -> float:
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive and finite.")
        return time.monotonic() + timeout_seconds

    @staticmethod
    def _remaining(deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def _state(
        self,
        target: tuple[float, float, float] | None = None,
        progress: float | None = None,
    ) -> str:
        position = self._position
        status = self._status
        return (
            f"mode={self._mode}, nav_state={getattr(status, 'nav_state', None)}, "
            f"armed={getattr(status, 'arming_state', None)}, failsafe={getattr(status, 'failsafe', None)}, "
            f"preflight={getattr(status, 'pre_flight_checks_pass', None)}, "
            f"status_age_s={(time.monotonic_ns() - self._status_received_ns) / 1e9 if self._status_received_ns else None}, "
            f"position={None if position is None else (position.x, position.y, position.z)}, "
            f"position_age_s={(time.monotonic_ns() - self._position_received_ns) / 1e9 if self._position_received_ns else None}, "
            f"heading_ready={getattr(position, 'heading_good_for_control', None)}, "
            f"landed={getattr(self._land, 'landed', None)}, "
            f"target_ned={target}, progress_m={progress}"
        )

    def _spin(self, deadline: float) -> None:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Flight action deadline expired; {self._state()}")
        try:
            self._rclpy.spin_once(
                self.node, timeout_sec=min(0.05, self._remaining(deadline))
            )
        except Exception:
            self._neutral_manual()
            raise

    def _on_status(self, message: Any) -> None:
        self._status = message
        self._status_received_ns = time.monotonic_ns()
        self._clock.observe(message)
        if self._mode in {"position", "altitude", "stabilized"}:
            expected = getattr(self._types["status"], self._mode_states[self._mode])
            if (
                message.nav_state != expected
                or message.arming_state != self._types["status"].ARMING_STATE_ARMED
                or message.failsafe
                or message.failsafe_and_user_took_over
            ):
                self._neutral_manual()

    def _on_position(self, message: Any) -> None:
        self._position = message
        self._position_received_ns = time.monotonic_ns()
        self._clock.observe(message)

    def _on_land(self, message: Any) -> None:
        self._land = message
        self._land_received_ns = time.monotonic_ns()

    def _fresh(self, received_ns: int, maximum_seconds: float = 3.0) -> bool:
        return (
            received_ns > 0
            and time.monotonic_ns() - received_ns <= maximum_seconds * 1e9
        )

    def _valid_position(self) -> bool:
        message = self._position
        return bool(
            message is not None
            and self._fresh(self._position_received_ns, 1.5)
            and message.xy_valid
            and message.z_valid
            and message.v_xy_valid
            and message.v_z_valid
            # This SITL model can report a finite, usable heading while the
            # separate heading_good_for_control flag remains false.
            and all(
                math.isfinite(float(getattr(message, field)))
                for field in ("x", "y", "z", "vx", "vy", "vz", "heading")
            )
        )

    def wait_ready(self, timeout_seconds: float = 30) -> dict[str, Any]:
        """Discover PX4 telemetry and capture the launch-relative origin."""

        if self._closed:
            raise RuntimeError("FlightSession is closed.")
        if self._origin is not None:
            return self._snapshot()
        deadline = self._deadline(timeout_seconds)
        qos = self._support.px4_subscription_qos()
        for name, base, callback in (
            ("status", "/fmu/out/vehicle_status", self._on_status),
            ("position", "/fmu/out/vehicle_local_position", self._on_position),
            ("land", "/fmu/out/vehicle_land_detected", self._on_land),
        ):
            topic = self._support.resolve_px4_topic(
                self.node, base, self._remaining(deadline)
            )
            self._subscriptions.append(
                self.node.create_subscription(self._types[name], topic, callback, qos)
            )
        while time.monotonic() < deadline:
            if (
                self._status is not None
                and self._fresh(self._status_received_ns)
                and self._status.pre_flight_checks_pass
                and self._valid_position()
                and self._land is not None
                and self._fresh(self._land_received_ns)
            ):
                p = self._position
                self._origin = (float(p.x), float(p.y), float(p.z))
                self._held_yaw = float(p.heading)
                return self._snapshot()
            self._spin(deadline)
        raise TimeoutError(
            f"PX4 flight telemetry or pre-arm checks did not become ready; {self._state()}"
        )

    def _snapshot(self) -> dict[str, Any]:
        position = self._position
        status = self._status
        return {
            "mode": self._mode,
            "nav_state": None if status is None else int(status.nav_state),
            "arming_state": None if status is None else int(status.arming_state),
            "position_ned": None
            if position is None
            else [float(position.x), float(position.y), float(position.z)],
            "velocity_ned": None
            if position is None
            else [float(position.vx), float(position.vy), float(position.vz)],
            "heading_rad": None if position is None else float(position.heading),
        }

    def _require_ready(self) -> None:
        if self._origin is None:
            raise RuntimeError("Call wait_ready before flight actions.")
        if self._closed:
            raise RuntimeError("FlightSession is closed.")

    def _require_active(self, target: tuple[float, float, float] | None = None) -> None:
        self._require_ready()
        status = self._status
        if (
            not self._fresh(self._status_received_ns, self._status_max_age_seconds)
            or not self._valid_position()
        ):
            self._neutral_manual()
            raise RuntimeError(
                f"PX4 status or local position is stale or invalid; {self._state(target)}"
            )
        expected = (
            getattr(self._types["status"], self._mode_states[self._mode])
            if self._mode
            else None
        )
        if (
            status.arming_state != self._types["status"].ARMING_STATE_ARMED
            or status.nav_state != expected
            or status.failsafe
            or status.failsafe_and_user_took_over
        ):
            self._neutral_manual()
            raise RuntimeError(
                f"PX4 left the commanded armed mode; {self._state(target)}"
            )

    def _absolute(self, pose: Pose) -> tuple[float, float, float]:
        self._require_ready()
        x, y, z = self._origin
        return (x + pose.north_m, y + pose.east_m, z - pose.up_m)

    def _position_tuple(self) -> tuple[float, float, float]:
        p = self._position
        return (float(p.x), float(p.y), float(p.z))

    def _next_position(self, previous_ns: int, deadline: float) -> Any:
        while time.monotonic() < deadline:
            if self._position_received_ns > previous_ns:
                self._require_active()
                return self._position
            if not self._fresh(self._position_received_ns, 1.5):
                self._neutral_manual()
                raise RuntimeError(
                    f"PX4 local position stopped updating; {self._state()}"
                )
            self._spin(deadline)
        self._neutral_manual()
        raise TimeoutError(
            f"Flight action timed out waiting for position; {self._state()}"
        )

    def _wait_state(self, predicate: Any, deadline: float, label: str) -> None:
        while time.monotonic() < deadline:
            if self._fresh(self._status_received_ns) and predicate(self._status):
                return
            self._spin(deadline)
        self._neutral_manual()
        raise TimeoutError(f"PX4 did not confirm {label}; {self._state()}")

    def _wait_preflight_checks(self, deadline: float) -> None:
        """Keep the neutral manual input flowing until PX4 accepts pre-arm checks."""

        while time.monotonic() < deadline:
            if not self._fresh(
                self._status_received_ns, self._status_max_age_seconds
            ):
                self._neutral_manual()
                raise RuntimeError(
                    f"PX4 status stopped updating while waiting for pre-arm checks; {self._state()}"
                )
            if self._status.failsafe or self._status.failsafe_and_user_took_over:
                self._neutral_manual()
                raise RuntimeError(
                    f"PX4 entered failsafe while waiting for pre-arm checks; {self._state()}"
                )
            if self._status.pre_flight_checks_pass:
                return
            self._spin(deadline)
        self._neutral_manual()
        raise TimeoutError(
            f"PX4 pre-arm checks did not pass after mode selection; {self._state()}"
        )

    def _command(
        self, command: int, parameters: Sequence[float], deadline: float
    ) -> None:
        if self._remaining(deadline) <= 0:
            raise TimeoutError(
                f"No time left to send PX4 command {command}; {self._state()}"
            )
        ack = self._support.send_vehicle_command(
            self.node,
            command,
            parameters,
            timeout_seconds=min(5.0, self._remaining(deadline)),
            timestamp_source=self._clock.now_us,
        )
        if ack.result not in (
            self._types["ack"].VEHICLE_CMD_RESULT_ACCEPTED,
            self._types["ack"].VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):
            raise RuntimeError(
                f"PX4 rejected command {command} with result {ack.result}; {self._state()}"
            )

    def _manual_message(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        throttle: float = 0.0,
    ) -> Any:
        message = self._types["manual"]()
        message.valid = True
        message.data_source = self._types["manual"].SOURCE_MAVLINK_0
        message.roll, message.pitch, message.yaw, message.throttle = (
            _clamp(float(value), self._manual_input_limit)
            for value in (roll, pitch, yaw, throttle)
        )
        message.sticks_moving = any(
            abs(value) > 1e-4
            for value in (message.roll, message.pitch, message.yaw, message.throttle)
        )
        message.flaps = math.nan
        for field in ("aux1", "aux2", "aux3", "aux4", "aux5", "aux6"):
            setattr(message, field, math.nan)
        return message

    def _neutral_manual(self) -> None:
        if self._manual_stream is not None:
            self._manual_stream.update(0, self._manual_message())

    def _set_manual(
        self,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        throttle: float = 0.0,
    ) -> None:
        if self._manual_stream is None:
            raise RuntimeError("Manual control stream is not active.")
        self._manual_stream.update(0, self._manual_message(roll, pitch, yaw, throttle))

    def _prime(self, stream: Any, cycles: int, deadline: float) -> None:
        target = stream.cycles_sent + cycles
        while stream.cycles_sent < target:
            self._spin(deadline)

    def _ensure_manual_stream(self, deadline: float) -> None:
        if self._manual_stream is None:
            self._manual_publisher = self.node.create_publisher(
                self._types["manual"], "/fmu/in/manual_control_input", 10
            )
            self._manual_stream = self._support.ControlStream(
                self.node,
                [(self._manual_publisher, self._manual_message())],
                timestamp_source=self._clock.now_us,
            )
            self._prime(self._manual_stream, 12, deadline)
        else:
            self._neutral_manual()

    def _offboard_setpoint(
        self, position: tuple[float, float, float], yaw: float
    ) -> Any:
        message = self._types["setpoint"]()
        message.position = list(position)
        message.velocity = [math.nan] * 3
        message.acceleration = [math.nan] * 3
        message.jerk = [math.nan] * 3
        message.yaw = _wrap(yaw)
        message.yawspeed = 0.0
        return message

    def _ensure_offboard_stream(self, deadline: float) -> None:
        if self._offboard_stream is not None:
            return
        self._ensure_manual_stream(deadline)
        mode = self._types["offboard"]()
        mode.position = True
        mode_publisher = self.node.create_publisher(
            self._types["offboard"], "/fmu/in/offboard_control_mode", 10
        )
        setpoint_publisher = self.node.create_publisher(
            self._types["setpoint"], "/fmu/in/trajectory_setpoint", 10
        )
        self._offboard_publishers = [mode_publisher, setpoint_publisher]
        self._offboard_stream = self._support.ControlStream(
            self.node,
            [
                (mode_publisher, mode),
                (
                    setpoint_publisher,
                    self._offboard_setpoint(self._position_tuple(), self._held_yaw),
                ),
            ],
            timestamp_source=self._clock.now_us,
        )
        self._prime(self._offboard_stream, 22, deadline)

    def _stop_offboard(self) -> None:
        if self._offboard_stream is not None:
            self._offboard_stream.close()
            self._offboard_stream = None
        for publisher in self._offboard_publishers:
            self.node.destroy_publisher(publisher)
        self._offboard_publishers.clear()

    def arm(self, timeout_seconds: float = 15) -> dict[str, Any]:
        """Enter Offboard with a stationary setpoint, then arm PX4."""

        self._require_ready()
        deadline = self._deadline(timeout_seconds)
        if (
            self._fresh(self._status_received_ns)
            and self._status.arming_state == self._types["status"].ARMING_STATE_ARMED
        ):
            return self._snapshot()
        self.set_mode("offboard", timeout_seconds=self._remaining(deadline))
        if not self._status.pre_flight_checks_pass:
            raise RuntimeError(f"PX4 pre-arm checks are not passing; {self._state()}")
        self._command(
            self._types["command"].VEHICLE_CMD_COMPONENT_ARM_DISARM, (1.0,), deadline
        )
        self._wait_state(
            lambda status: (
                status.arming_state == self._types["status"].ARMING_STATE_ARMED
            ),
            deadline,
            "arming",
        )
        return self._snapshot()

    def set_mode(self, mode: str, timeout_seconds: float = 15) -> dict[str, Any]:
        """Prime the required control stream and confirm the actual PX4 mode."""

        self._require_ready()
        if mode not in self._mode_states:
            raise ValueError(f"Unsupported PX4 mode {mode!r}.")
        deadline = self._deadline(timeout_seconds)
        self._ensure_manual_stream(deadline)
        if mode == "offboard":
            self._ensure_offboard_stream(deadline)
        else:
            self._neutral_manual()
            self._prime(self._manual_stream, 12, deadline)
        expected = getattr(self._types["status"], self._mode_states[mode])
        if not (
            self._fresh(self._status_received_ns) and self._status.nav_state == expected
        ):
            self._command(
                self._types["command"].VEHICLE_CMD_DO_SET_MODE,
                (1.0, self._mode_params[mode]),
                deadline,
            )
            self._wait_state(
                lambda status: status.nav_state == expected, deadline, f"{mode} mode"
            )
        self._mode = mode
        if mode != "offboard":
            self._stop_offboard()
        return self._snapshot()

    def _control(
        self, target: tuple[float, float, float], yaw: float, speed_mps: float
    ) -> None:
        sample = self._position
        current = self._position_tuple()
        delta = tuple(goal - actual for goal, actual in zip(target, current))
        distance = math.sqrt(sum(value * value for value in delta))
        if self._mode == "offboard":
            self._offboard_stream.update(1, self._offboard_setpoint(target, yaw))
            return
        heading = float(sample.heading)
        yaw_input = _clamp(1.2 * _wrap(yaw - heading), 0.35)
        if self._mode == "position":
            speed = min(speed_mps, math.sqrt(2.0 * max(distance - 0.08, 0.0)))
            north_velocity = delta[0] * speed / distance if distance > 1e-6 else 0.0
            east_velocity = delta[1] * speed / distance if distance > 1e-6 else 0.0
            up_velocity = -delta[2] * speed / distance if distance > 1e-6 else 0.0
            forward = north_velocity * math.cos(heading) + east_velocity * math.sin(
                heading
            )
            right = -north_velocity * math.sin(heading) + east_velocity * math.cos(
                heading
            )
            self._set_manual(
                roll=right / 3.0,
                pitch=forward / 3.0,
                yaw=yaw_input,
                throttle=up_velocity / 3.0,
            )
            return
        vx, vy, vz = float(sample.vx), float(sample.vy), float(sample.vz)
        horizontal = math.hypot(delta[0], delta[1])
        desired_speed = min(speed_mps, math.sqrt(2.0 * max(horizontal - 0.08, 0.0)))
        desired_north = (
            delta[0] * desired_speed / horizontal if horizontal > 1e-6 else 0.0
        )
        desired_east = (
            delta[1] * desired_speed / horizontal if horizontal > 1e-6 else 0.0
        )
        north_accel = _clamp(2.0 * (desired_north - vx), 1.0)
        east_accel = _clamp(2.0 * (desired_east - vy), 1.0)
        magnitude = math.hypot(north_accel, east_accel)
        if magnitude > 1.0:
            north_accel /= magnitude
            east_accel /= magnitude
        forward_accel = north_accel * math.cos(heading) + east_accel * math.sin(heading)
        right_accel = -north_accel * math.sin(heading) + east_accel * math.cos(heading)
        # Altitude keeps height when centered; Stabilized needs this vertical loop continuously.
        throttle = _clamp(1.4 * (current[2] - target[2]) + 0.6 * vz, 0.5)
        stick_accel = 9.81 * math.tan(math.radians(35))
        self._set_manual(
            roll=right_accel / stick_accel,
            pitch=forward_accel / stick_accel,
            yaw=yaw_input,
            throttle=throttle,
        )

    def _navigate(
        self,
        points: Sequence[tuple[float, float, float]],
        yaws: Sequence[float],
        *,
        speed_mps: float,
        tolerance_m: float,
        deadline: float,
    ) -> dict[str, Any]:
        if not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("speed_mps must be positive and finite.")
        if not math.isfinite(tolerance_m) or tolerance_m <= 0:
            raise ValueError("tolerance_m must be positive and finite.")
        lengths = _route_lengths(points)
        total = lengths[-1]
        progress = 0.0
        max_error = 0.0
        started = time.monotonic()
        last_position_ns = self._position_received_ns
        settled_since: float | None = None
        try:
            while time.monotonic() < deadline:
                self._require_active(points[-1])
                try:
                    sample = self._next_position(last_position_ns, deadline)
                except TimeoutError as exc:
                    raise TimeoutError(
                        f"Flight path did not settle before deadline; "
                        f"{self._state(points[-1], progress)}, length_m={total}, "
                        f"maximum_cross_track_error_m={max_error}"
                    ) from exc
                last_position_ns = self._position_received_ns
                current = self._position_tuple()
                lookahead = max(tolerance_m, min(0.7, speed_mps * 0.6))
                projected, cross_track = _project_progress(
                    current,
                    points,
                    lengths,
                    max(0.0, progress - tolerance_m),
                    min(total, progress + lookahead + tolerance_m),
                )
                if cross_track <= max(2.0 * tolerance_m, 0.4):
                    progress = max(progress, projected)
                max_error = max(max_error, cross_track)
                scheduled = min(total, speed_mps * (time.monotonic() - started))
                target_progress = min(total, scheduled, progress + lookahead)
                target, yaw = _route_pose(points, yaws, lengths, target_progress)
                remaining = max(0.0, total - progress)
                local_speed = min(
                    speed_mps, math.sqrt(2.0 * max(remaining - tolerance_m * 0.5, 0.0))
                )
                self._control(target, yaw, local_speed)
                endpoint_error = _distance(current, points[-1])
                velocity = math.sqrt(
                    sample.vx * sample.vx
                    + sample.vy * sample.vy
                    + sample.vz * sample.vz
                )
                yaw_error = abs(_wrap(yaws[-1] - sample.heading))
                settled = (
                    target_progress >= total - 1e-6
                    and progress >= total - tolerance_m
                    and endpoint_error <= tolerance_m
                    and velocity <= 0.12
                    and yaw_error <= math.radians(10)
                )
                if settled:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic() - settled_since >= 0.5:
                        self._held_yaw = yaws[-1]
                        if self._mode != "offboard":
                            self._neutral_manual()
                        return {
                            **self._snapshot(),
                            "progress_m": progress,
                            "path_length_m": total,
                            "maximum_cross_track_error_m": max_error,
                            "endpoint_error_m": endpoint_error,
                            "duration_seconds": time.monotonic() - started,
                        }
                else:
                    settled_since = None
            raise TimeoutError(
                f"Flight path did not settle before deadline; {self._state(points[-1], progress)}, "
                f"length_m={total}, maximum_cross_track_error_m={max_error}"
            )
        except Exception:
            self._neutral_manual()
            raise

    def goto(
        self,
        pose: Pose,
        *,
        speed_mps: float = 0.7,
        tolerance_m: float = 0.25,
        timeout_seconds: float = 30,
    ) -> dict[str, Any]:
        """Reach one launch-relative pose and remain settled for 0.5 seconds."""

        self._require_active()
        deadline = self._deadline(timeout_seconds)
        target = self._absolute(pose)
        yaw = float(self._position.heading) if pose.yaw_rad is None else pose.yaw_rad
        start = self._position_tuple()
        if _distance(start, target) <= 1e-6:
            if abs(_wrap(yaw - self._position.heading)) > math.radians(5):
                self.turn_to(yaw, timeout_seconds=self._remaining(deadline))
            return self._hold(target, yaw, 0.5, deadline)
        return self._navigate(
            [start, target],
            [float(self._position.heading), yaw],
            speed_mps=speed_mps,
            tolerance_m=tolerance_m,
            deadline=deadline,
        )

    def follow_path(
        self,
        points: Sequence[Pose],
        *,
        speed_mps: float = 0.7,
        tolerance_m: float = 0.35,
        timeout_seconds: float = 60,
    ) -> dict[str, Any]:
        """Track an ordered geometric path without dwelling at its vertices."""

        self._require_active()
        if len(points) < 2:
            raise ValueError(
                "follow_path needs at least two poses; use goto for one pose."
            )
        deadline = self._deadline(timeout_seconds)
        absolute = [self._absolute(pose) for pose in points]
        yaws: list[float] = []
        last_yaw = float(self._position.heading)
        for pose in points:
            if pose.yaw_rad is not None:
                last_yaw = pose.yaw_rad
            yaws.append(last_yaw)
        _route_lengths(absolute)
        if _distance(self._position_tuple(), absolute[0]) > tolerance_m:
            approach = Pose(
                points[0].north_m, points[0].east_m, points[0].up_m, points[0].yaw_rad
            )
            self.goto(
                approach,
                speed_mps=speed_mps,
                tolerance_m=tolerance_m,
                timeout_seconds=self._remaining(deadline),
            )
        return self._navigate(
            absolute,
            yaws,
            speed_mps=speed_mps,
            tolerance_m=tolerance_m,
            deadline=deadline,
        )

    def _manual_takeoff_until_liftoff(self, deadline: float) -> float:
        """Ramp bounded climb input until fresh position data confirms ascent."""

        try:
            self._require_active()
            start_z = float(self._position.z)
            last_position_ns = self._position_received_ns
            started = time.monotonic()
            while time.monotonic() < deadline:
                sample = self._next_position(last_position_ns, deadline)
                last_position_ns = self._position_received_ns
                climbed_m = start_z - float(sample.z)
                if climbed_m >= self._manual_takeoff_liftoff_m:
                    return climbed_m
                ramp = min(
                    1.0,
                    max(0.0, (time.monotonic() - started) / self._manual_takeoff_ramp_seconds),
                )
                self._set_manual(throttle=ramp * self._manual_input_limit)
            raise TimeoutError(
                f"PX4 position did not confirm liftoff before deadline; {self._state()}"
            )
        except Exception:
            # Keep the manual stream alive and neutral while the test inspects the failure.
            self._neutral_manual()
            raise

    def takeoff(
        self,
        up_m: float,
        *,
        speed_mps: float = 0.7,
        timeout_seconds: float = 35,
        mode: str | None = None,
    ) -> dict[str, Any]:
        """Arm if necessary, then climb relative to wait_ready in the selected mode."""

        if not math.isfinite(up_m) or up_m <= 0:
            raise ValueError("takeoff up_m must be positive and finite.")
        if mode is not None and mode not in self._mode_states:
            raise ValueError(f"Unsupported PX4 mode {mode!r}.")
        deadline = self._deadline(timeout_seconds)
        if mode is None:
            if (
                self._status is None
                or self._status.arming_state != self._types["status"].ARMING_STATE_ARMED
            ):
                self.arm(timeout_seconds=self._remaining(deadline))
        else:
            status_type = self._types["status"]
            status_fresh = (
                self._status is not None
                and self._fresh(self._status_received_ns)
            )
            already_armed = (
                status_fresh
                and self._status.arming_state == status_type.ARMING_STATE_ARMED
            )
            if not already_armed and not (
                status_fresh and self._status.pre_flight_checks_pass
            ):
                raise RuntimeError(f"PX4 pre-arm checks are not passing; {self._state()}")
            self.set_mode(mode, timeout_seconds=self._remaining(deadline))
            if not already_armed:
                if mode in {"position", "altitude", "stabilized"}:
                    self._wait_preflight_checks(deadline)
                self._command(
                    self._types["command"].VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    (1.0,),
                    deadline,
                )
                self._wait_state(
                    lambda status: status.arming_state == status_type.ARMING_STATE_ARMED,
                    deadline,
                    "arming",
                )
            if mode in {"position", "altitude", "stabilized"}:
                self._manual_takeoff_until_liftoff(deadline)
        return self.goto(
            Pose(0.0, 0.0, up_m),
            speed_mps=speed_mps,
            timeout_seconds=self._remaining(deadline),
        )

    def _hold(
        self,
        target: tuple[float, float, float],
        yaw: float,
        duration_seconds: float,
        deadline: float,
    ) -> dict[str, Any]:
        started = time.monotonic()
        last_ns = self._position_received_ns
        maximum_drift = 0.0
        try:
            while time.monotonic() - started < duration_seconds:
                self._require_active(target)
                self._next_position(last_ns, deadline)
                last_ns = self._position_received_ns
                self._control(target, yaw, 0.0)
                maximum_drift = max(
                    maximum_drift, _distance(self._position_tuple(), target)
                )
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Hold timed out; {self._state(target)}")
            self._held_yaw = yaw
            if self._mode != "offboard":
                self._neutral_manual()
            return {
                **self._snapshot(),
                "maximum_drift_m": maximum_drift,
                "duration_seconds": time.monotonic() - started,
            }
        except Exception:
            self._neutral_manual()
            raise

    def turn_to(
        self,
        yaw_rad: float,
        *,
        timeout_seconds: float = 15,
        tolerance_rad: float = math.radians(5),
    ) -> dict[str, Any]:
        self._require_active()
        if (
            not math.isfinite(yaw_rad)
            or not math.isfinite(tolerance_rad)
            or tolerance_rad <= 0
        ):
            raise ValueError(
                "yaw_rad and tolerance_rad must be finite; tolerance_rad must be positive."
            )
        deadline = self._deadline(timeout_seconds)
        target = self._position_tuple()
        settled_since: float | None = None
        last_ns = self._position_received_ns
        try:
            while time.monotonic() < deadline:
                self._next_position(last_ns, deadline)
                last_ns = self._position_received_ns
                self._control(target, yaw_rad, 0.0)
                if abs(_wrap(yaw_rad - self._position.heading)) <= tolerance_rad:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic() - settled_since >= 0.5:
                        self._held_yaw = yaw_rad
                        if self._mode != "offboard":
                            self._neutral_manual()
                        return self._snapshot()
                else:
                    settled_since = None
            raise TimeoutError(
                f"Yaw did not settle before deadline; {self._state(target)}"
            )
        except Exception:
            self._neutral_manual()
            raise

    def hover(
        self, duration_seconds: float, *, timeout_seconds: float | None = None
    ) -> dict[str, Any]:
        """Hold current pose and report drift; the scenario sets its pass limit."""

        self._require_active()
        if not math.isfinite(duration_seconds) or duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive and finite.")
        deadline = self._deadline(
            timeout_seconds if timeout_seconds is not None else duration_seconds + 5
        )
        return self._hold(
            self._position_tuple(),
            float(self._position.heading),
            duration_seconds,
            deadline,
        )

    def land(self, timeout_seconds: float = 45) -> dict[str, Any]:
        """Use PX4 Auto Land and wait for landed and disarmed telemetry."""

        self._require_ready()
        deadline = self._deadline(timeout_seconds)
        status_type = self._types["status"]
        if (
            self._fresh(self._status_received_ns)
            and self._status.arming_state == status_type.ARMING_STATE_DISARMED
        ):
            if (
                self._land is not None
                and self._fresh(self._land_received_ns)
                and self._land.landed
            ):
                return {**self._snapshot(), "landed": True, "disarmed": True}
            raise RuntimeError(
                f"PX4 is disarmed without fresh landed evidence; {self._state()}"
            )
        if self._mode in {"altitude", "stabilized"}:
            self.set_mode("position", timeout_seconds=self._remaining(deadline))
        self._neutral_manual()
        if self._manual_stream is not None:
            self._prime(self._manual_stream, 12, deadline)
        self._command(self._types["command"].VEHICLE_CMD_NAV_LAND, (), deadline)
        self._wait_state(
            lambda status: status.nav_state == status_type.NAVIGATION_STATE_AUTO_LAND,
            deadline,
            "Auto Land",
        )
        self._mode = None
        self._stop_offboard()
        while time.monotonic() < deadline:
            landed = (
                self._land is not None
                and self._fresh(self._land_received_ns)
                and self._land.landed
            )
            if (
                landed
                and self._fresh(self._status_received_ns)
                and self._status.arming_state == status_type.ARMING_STATE_DISARMED
            ):
                self._stop_manual()
                return {**self._snapshot(), "landed": True, "disarmed": True}
            self._spin(deadline)
        raise TimeoutError(f"PX4 did not confirm touchdown and disarm; {self._state()}")

    def _stop_manual(self) -> None:
        if self._manual_stream is not None:
            self._neutral_manual()
            self._manual_stream.close()
            self._manual_stream = None
        if self._manual_publisher is not None:
            self.node.destroy_publisher(self._manual_publisher)
            self._manual_publisher = None

    def close(self) -> None:
        """Stop helper-owned streams; landing remains an explicit scenario action."""

        if self._closed:
            return
        self._closed = True
        self._stop_offboard()
        self._stop_manual()
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        self._subscriptions.clear()
