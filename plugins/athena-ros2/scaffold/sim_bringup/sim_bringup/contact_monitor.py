"""Record observed Gazebo collision pairs without judging a scenario."""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts


class ContactMonitor(Node):
    def __init__(self, topics: list[str]) -> None:
        super().__init__("athena_contact_monitor")
        self.topics = topics
        self.summaries: dict[tuple[str, str, str], dict[str, object]] = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._contact_subscriptions = [
            self.create_subscription(Contacts, topic, lambda message, name=topic: self.observe(name, message), qos)
            for topic in topics
        ]

    def observe(self, topic: str, message: Contacts) -> None:
        received_ns = time.time_ns()
        for contact in message.contacts:
            first, second = sorted((contact.collision1.name, contact.collision2.name))
            key = (topic, first, second)
            summary = self.summaries.get(key)
            if summary is None:
                self.summaries[key] = {
                    "topic": topic,
                    "collision_a": first,
                    "collision_b": second,
                    "count": 1,
                    "first_wall_time_ns": received_ns,
                    "last_wall_time_ns": received_ns,
                }
            else:
                summary["count"] = int(summary["count"]) + 1
                summary["last_wall_time_ns"] = received_ns


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--topic", action="append", required=True)
    args = parser.parse_args()
    rclpy.init()
    monitor = ContactMonitor(args.topic)

    def stop(_signum: int, _frame: object) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        deadline = time.monotonic() + 15
        while rclpy.ok() and time.monotonic() < deadline:
            if all(monitor.count_publishers(topic) > 0 for topic in args.topic):
                args.ready_file.write_text(json.dumps({"topics": args.topic}) + "\n", encoding="utf-8")
                break
            rclpy.spin_once(monitor, timeout_sec=0.25)
        else:
            if rclpy.ok():
                raise RuntimeError("Contact topic publishers did not become ready.")
        while rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.5)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            for summary in monitor.summaries.values():
                stream.write(json.dumps(summary, sort_keys=True) + "\n")
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
