"""Small PX4 ROS test primitives; scenario routes and assertions belong to tests."""

from __future__ import annotations

import copy
import math
import re
import threading
import time
from collections.abc import Callable, Sequence
from typing import Any

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
