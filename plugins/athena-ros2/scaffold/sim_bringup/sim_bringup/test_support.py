"""Small PX4 ROS test primitives; scenario routes and assertions belong to tests."""

from __future__ import annotations

import copy
import json
import math
import os
import re
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from types import TracebackType
from typing import Any, Self

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def px4_subscription_qos() -> QoSProfile:
    """Match PX4's best-effort, transient-local telemetry publishers."""

    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def resolve_px4_topic(node: Node, base: str, timeout_seconds: float = 10) -> str:
    """Find an exact PX4 topic or its highest available _vN variant."""

    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        names = {name for name, _ in node.get_topic_names_and_types()}
        if base in names:
            return base
        variants = [name for name in names if re.fullmatch(re.escape(base) + r"_v\d+", name)]
        if variants:
            return max(variants, key=lambda name: int(name.rsplit("_v", 1)[1]))
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
    raise TimeoutError(f"PX4 topic {base} was not discovered within {timeout_seconds:g} seconds.")


def wait_for_message(
    node: Node,
    message_type: type,
    topic: str,
    predicate: Callable[[Any], bool] = lambda _message: True,
    timeout_seconds: float = 10,
) -> Any:
    """Wait for a matching message on a test-owned node and report a deadline."""

    received: list[Any] = []
    subscription = node.create_subscription(
        message_type, topic,
        lambda message: received.append(message) if predicate(message) else None,
        px4_subscription_qos(),
    )
    try:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if received:
                return received[-1]
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(f"No matching message on {topic} within {timeout_seconds:g} seconds.")
    finally:
        node.destroy_subscription(subscription)


class Px4Time:
    """Estimate PX4 boot time from fresh PX4 telemetry, not the ROS clock."""

    def __init__(self, max_age_seconds: float = 2.0) -> None:
        if not math.isfinite(max_age_seconds) or max_age_seconds <= 0:
            raise ValueError("max_age_seconds must be positive and finite")
        self._timestamp_us: int | None = None
        self._received_ns = 0
        self._max_age_ns = int(max_age_seconds * 1_000_000_000)

    def observe(self, message: Any) -> None:
        timestamp_us = int(message.timestamp)
        if timestamp_us <= 0:
            return
        self._timestamp_us = timestamp_us
        self._received_ns = time.monotonic_ns()

    def now_us(self) -> int:
        if self._timestamp_us is None:
            raise RuntimeError("No PX4 timestamp has been observed yet.")
        age_ns = time.monotonic_ns() - self._received_ns
        if age_ns < 0 or age_ns > self._max_age_ns:
            raise RuntimeError("PX4 telemetry is too old to timestamp a command.")
        return self._timestamp_us + age_ns // 1000


def send_vehicle_command(
    node: Node,
    command: int,
    parameters: Sequence[float] = (),
    timeout_seconds: float = 5,
    *,
    timestamp_source: Callable[[], int],
) -> Any:
    """Publish one VehicleCommand and return its PX4 acknowledgement for inspection."""

    from px4_msgs.msg import VehicleCommand, VehicleCommandAck

    if len(parameters) > 7 or any(not math.isfinite(float(value)) for value in parameters):
        raise ValueError("VehicleCommand needs at most seven finite parameters.")
    ack_topic = resolve_px4_topic(node, "/fmu/out/vehicle_command_ack", timeout_seconds)
    acknowledgements: list[Any] = []
    accepting = False

    def observe(message: Any) -> None:
        if accepting and message.command == command:
            acknowledgements.append(message)

    subscription = node.create_subscription(VehicleCommandAck, ack_topic, observe, px4_subscription_qos())
    publisher = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
    try:
        deadline = time.monotonic() + timeout_seconds
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise TimeoutError("PX4 did not subscribe to /fmu/in/vehicle_command.")
        message = VehicleCommand()
        message.timestamp = timestamp_source()
        message.command = command
        for index, value in enumerate(parameters, start=1):
            setattr(message, f"param{index}", float(value))
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        accepting = True
        publisher.publish(message)
        while time.monotonic() < deadline:
            if acknowledgements:
                return acknowledgements[-1]
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(f"PX4 did not acknowledge command {command} within {timeout_seconds:g} seconds.")
    finally:
        node.destroy_publisher(publisher)
        node.destroy_subscription(subscription)


class ControlStream:
    """Keep caller-selected PX4 control messages flowing; pause for loss tests."""

    def __init__(
        self,
        node: Node,
        publications: Sequence[tuple[Any, Any]],
        rate_hz: float = 20,
        *,
        timestamp_source: Callable[[], int],
    ) -> None:
        if not publications or not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("ControlStream needs messages and a positive rate.")
        self.node = node
        self._lock = threading.Lock()
        self._publications = [(publisher, copy.deepcopy(message)) for publisher, message in publications]
        self._timestamp_source = timestamp_source
        self.cycles_sent = 0
        # Keep the control heartbeat regular even while Gazebo's /clock stalls.
        self._timer = node.create_timer(1.0 / rate_hz, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _publish(self) -> None:
        with self._lock:
            items = [(publisher, copy.deepcopy(message)) for publisher, message in self._publications]
            timestamp_us = self._timestamp_source()
            for publisher, message in items:
                if hasattr(message, "timestamp"):
                    message.timestamp = timestamp_us
                if hasattr(message, "timestamp_sample"):
                    message.timestamp_sample = timestamp_us
                publisher.publish(message)
            self.cycles_sent += 1

    def update(self, index: int, message: Any) -> None:
        with self._lock:
            publisher, _ = self._publications[index]
            self._publications[index] = (publisher, copy.deepcopy(message))

    def pause(self) -> None:
        self._timer.cancel()
        # Wait for a callback already publishing, so a deliberate stream gap is clean.
        with self._lock:
            pass

    def resume(self) -> None:
        self._timer.reset()

    def close(self) -> None:
        self.node.destroy_timer(self._timer)


class BatteryFeed:
    """Publish external MAVLink battery measurements throughout a scenario."""

    def __init__(self, node: Node, initial_percent: int, *, rate_hz: float = 5.0) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("Battery feed rate must be positive and finite.")
        self.node = node
        self._percent = self._validate_percent(initial_percent)
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._published = threading.Event()
        self._thread: threading.Thread | None = None
        self._peer: Any = None
        self._error: Exception | None = None
        self.first_observed: Any = None

    @staticmethod
    def _validate_percent(value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
            raise ValueError("Battery percentage must be an integer from 0 to 100.")
        return value

    def _publish(self) -> None:
        from pymavlink import mavutil

        try:
            while not self._stop.is_set():
                with self._lock:
                    percent = self._percent
                # Four real cell voltages followed by MAVLink's unknown marker.
                self._peer.mav.battery_status_send(
                    0, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                    mavutil.mavlink.MAV_BATTERY_TYPE_LIPO, 2500,
                    [4000] * 4 + [65535] * 6, -1, -1, -1, percent,
                )
                self._published.set()
                self._stop.wait(self._period)
        except Exception as exc:  # noqa: BLE001 - surface a background sender failure.
            self._error = exc
            self._published.set()

    def __enter__(self) -> Self:
        from pymavlink import mavutil

        self._peer = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
        try:
            heartbeat = self._peer.recv_match(type="HEARTBEAT", blocking=True, timeout=8.0)
            if heartbeat is None:
                raise TimeoutError("PX4 MAVLink heartbeat was not received on UDP 14550.")
            self._peer.mav.srcSystem = heartbeat.get_srcSystem()
            self._peer.mav.srcComponent = 190
            self._thread = threading.Thread(target=self._publish, name="battery-status-feed", daemon=True)
            self._thread.start()
            if not self._published.wait(3.0) or self._error is not None:
                raise RuntimeError(f"Battery feed did not start: {self._error}")
            self.first_observed = self.wait_for_percent(self._percent)
            return self
        except Exception:
            self.close()
            raise

    def set_percent(self, percent: int) -> None:
        with self._lock:
            self._percent = self._validate_percent(percent)

    def wait_for_percent(self, percent: int, timeout_seconds: float = 5.0) -> Any:
        """Return fresh PX4 output that confirms the requested fraction."""

        from px4_msgs.msg import BatteryStatus

        requested = self._validate_percent(percent)
        topic = resolve_px4_topic(self.node, "/fmu/out/battery_status", timeout_seconds)
        return wait_for_message(
            self.node, BatteryStatus, topic,
            lambda message: (bool(message.connected)
                             and abs(float(message.remaining) - requested / 100.0) <= 0.015),
            timeout_seconds,
        )

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._peer is not None:
            self._peer.close()
            self._peer = None

    def __exit__(self, _type: type[BaseException] | None,
                 _value: BaseException | None, _traceback: TracebackType | None) -> None:
        self.close()


def write_step_completed(
    step: str, failsafe_flags: Any, flag_name: str, vehicle_status: Any,
    *, path: str | Path | None = None,
) -> dict[str, Any]:
    """Record a named observed PX4 fault and integer navigation state."""

    if not step or not isinstance(step, str):
        raise ValueError("Step name must be a nonempty string.")
    observed = getattr(failsafe_flags, flag_name, None)
    state = getattr(vehicle_status, "nav_state", None)
    if not isinstance(observed, bool) or isinstance(state, bool) or not isinstance(state, int):
        raise TypeError("Expected an observed boolean failsafe flag and integer PX4 nav_state.")
    if path is None:
        run_dir = os.environ.get("ATHENA_SIMULATION_RUN_DIR")
        if not run_dir:
            raise ValueError("Provide path or set ATHENA_SIMULATION_RUN_DIR.")
        path = Path(run_dir) / "scenario_events.jsonl"
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    event = {"event": "step_completed", "step": step, "step_id": step,
             "wall_time_ns": time.time_ns(),
             "result": {"failsafe_flag": flag_name, "failsafe_active": observed,
                        "nav_state": state}}
    with destination.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, sort_keys=True) + "\n")
    return event
