"""Choose a compact PX4 evidence set and check that rosbag subscribed."""

from __future__ import annotations

import re
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# Battery is deliberately opt-in; the standard flight and failsafe evidence is small.
DEFAULT_TOPIC_BASES = (
    "/clock",
    "/fmu/out/vehicle_local_position",
    "/fmu/out/vehicle_status",
    "/fmu/out/vehicle_land_detected",
    "/fmu/out/failsafe_flags",
    "/fmu/out/vehicle_command_ack",
    "/fmu/out/manual_control_setpoint",
    "/fmu/in/telemetry_status",
    "/fmu/in/vehicle_command",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/manual_control_input",
)


def contact_topics(drone_sdf: Path) -> list[str]:
    return sorted({
        topic
        for sensor in ET.parse(drone_sdf).getroot().findall(".//sensor")
        if (sensor.get("type") or "").lower() == "contact"
        if (topic := (sensor.findtext("topic") or "").strip()).startswith("/")
    })


def select_topics(available: set[str], extra: list[str]) -> tuple[list[str], list[str]]:
    """Resolve PX4's versioned topic names without recording every sensor."""

    selected: list[str] = []
    missing: list[str] = []
    for base in DEFAULT_TOPIC_BASES:
        if base in available:
            topic = base
        else:
            matches = sorted(
                (name for name in available if re.fullmatch(re.escape(base) + r"_v\d+", name)),
                key=lambda name: int(name.rsplit("_v", 1)[1]),
            )
            topic = matches[-1] if matches else None
        if topic is None:
            missing.append(base)
            topic = base  # Rosbag discovers a publisher started by the application later.
        if topic not in selected:
            selected.append(topic)
    for topic in extra:
        if topic not in selected:
            selected.append(topic)
    return selected, missing


def list_topics(env: dict[str, str]) -> set[str]:
    result = subprocess.run(
        ["ros2", "topic", "list", "--no-daemon"],
        env=env, capture_output=True, text=True, timeout=8, check=False,
    )
    if result.returncode:
        raise RuntimeError("Unable to list ROS topics for recording: " + (result.stderr or result.stdout)[-500:])
    return set(result.stdout.splitlines())


def wait_for_recording(process: subprocess.Popen[bytes], status_topic: str, env: dict[str, str], seconds: float = 20) -> bool:
    """Observe a recorder subscription before starting the application test."""

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False
        try:
            result = subprocess.run(
                ["ros2", "topic", "info", "--verbose", "--no-daemon", status_topic],
                env=env, capture_output=True, text=True, timeout=5, check=False,
            )
        except subprocess.TimeoutExpired:
            continue
        if result.returncode == 0:
            match = re.search(r"Subscription count:\s*(\d+)", result.stdout)
            if match and int(match.group(1)) > 0:
                return True
        time.sleep(0.25)
    return False
