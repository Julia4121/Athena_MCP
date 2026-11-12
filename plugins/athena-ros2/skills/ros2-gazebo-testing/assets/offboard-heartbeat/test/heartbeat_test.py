"""Check the example publishes a heartbeat stamped in fresh PX4 time."""

from __future__ import annotations

import sys
import time

import rclpy
from px4_msgs.msg import OffboardControlMode, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sim_bringup.test_support import px4_subscription_qos, resolve_px4_topic


def main() -> int:
    rclpy.init()
    node = Node("athena_heartbeat_test")
    latest_status: list[int] = []
    heartbeats: list[int] = []
    try:
        status_topic = resolve_px4_topic(node, "/fmu/out/vehicle_status", 15)
        node.create_subscription(
            VehicleStatus, status_topic,
            lambda message: latest_status.append(int(message.timestamp)),
            px4_subscription_qos(),
        )
        node.create_subscription(
            OffboardControlMode, "/fmu/in/offboard_control_mode",
            lambda message: heartbeats.append(int(message.timestamp)),
            QoSProfile(depth=10),
        )
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if latest_status and heartbeats:
                difference_us = abs(latest_status[-1] - heartbeats[-1])
                if difference_us < 2_000_000:
                    print(f"PX4-timed heartbeat observed; timestamp difference {difference_us} us")
                    return 0
        print("No fresh PX4-timed heartbeat was observed.", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
