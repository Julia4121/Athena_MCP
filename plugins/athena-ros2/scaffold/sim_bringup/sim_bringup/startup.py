"""Supervise one headless Gazebo/PX4 session for Codex-authored tests."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

SENSOR_BRIDGES = {
    "air_pressure": ("gz.msgs.FluidPressure", "sensor_msgs/msg/FluidPressure"),
    "camera": ("gz.msgs.Image", "sensor_msgs/msg/Image"),
    "depth_camera": ("gz.msgs.Image", "sensor_msgs/msg/Image"),
    "gps": ("gz.msgs.NavSat", "sensor_msgs/msg/NavSatFix"),
    "navsat": ("gz.msgs.NavSat", "sensor_msgs/msg/NavSatFix"),
    "gpu_lidar": ("gz.msgs.PointCloudPacked", "sensor_msgs/msg/PointCloud2"),
    "imu": ("gz.msgs.IMU", "sensor_msgs/msg/Imu"),
    "lidar": ("gz.msgs.LaserScan", "sensor_msgs/msg/LaserScan"),
    "ray": ("gz.msgs.LaserScan", "sensor_msgs/msg/LaserScan"),
    "magnetometer": ("gz.msgs.Magnetometer", "sensor_msgs/msg/MagneticField"),
    "contact": ("gz.msgs.Contacts", "ros_gz_interfaces/msg/Contacts"),
}


def _run(command: list[str], env: dict[str, str], timeout: float = 5) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, env=env, capture_output=True, text=True, timeout=timeout, check=False)


def _quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def _sensor_bridges(drone_sdf: Path) -> list[str]:
    root = ET.parse(drone_sdf).getroot()
    topics: dict[str, tuple[str, str]] = {
        "/clock": ("gz.msgs.Clock", "rosgraph_msgs/msg/Clock")
    }
    for sensor in root.findall(".//sensor"):
        topic = (sensor.findtext("topic") or "").strip()
        mapping = SENSOR_BRIDGES.get((sensor.get("type") or "").lower())
        if topic and mapping:
            topics[topic] = mapping
    return [f"{topic}@{ros_type}[{gz_type}" for topic, (gz_type, ros_type) in topics.items()]


class Supervisor:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.status_path = Path(args.status_file)
        self.children: dict[str, subprocess.Popen[bytes]] = {}
        self.logs: dict[str, object] = {}
        self.stop_requested = False
        self.px4_parameters = {
            "NAV_DLL_ACT": 0,
            "UXRCE_DDS_SYNCT": 0,
            **json.loads(args.px4_parameters),
        }
        self.phase = "preflight"
        self.readiness: dict[str, bool] = {
            "gazebo_world": False,
            "drone_entity": False,
            "ros_clock": False,
            "xrce_session": False,
            "px4_telemetry": False,
            "px4_startup": False,
        }
        self.readiness_details: dict[str, object] = {}
        self.env = os.environ.copy()
        resource_roots = [
            str(Path(args.resource_root)),
            str(Path(args.resource_root) / "environments"),
            str(Path(args.drone_sdf).parent.parent),
            str(Path(args.drone_sdf).parent),
            str(Path(args.world_sdf).parent),
        ]
        existing = self.env.get("GZ_SIM_RESOURCE_PATH")
        if existing:
            resource_roots.append(existing)
        self.env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resource_roots)

    def _status(self, *, error: str | None = None) -> None:
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "phase": self.phase,
            "supervisor_pid": os.getpid(),
            "readiness": self.readiness,
            "readiness_details": self.readiness_details,
            "px4_parameters_requested": self.px4_parameters,
            "processes": {name: {"pid": proc.pid, "exit_code": proc.poll()} for name, proc in self.children.items()},
            "error": error,
        }
        temporary = self.status_path.with_name(self.status_path.name + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.status_path)

    def _start(self, name: str, command: list[str], env: dict[str, str], *, px4: bool = False) -> subprocess.Popen[bytes]:
        log = (self.log_dir / f"{name}.log").open("ab", buffering=0)
        self.logs[name] = log
        process = subprocess.Popen(
            command,
            env=env,
            cwd=env.get("PX4_AUTOPILOT_DIR") if px4 else None,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE if px4 else subprocess.DEVNULL,
            start_new_session=True,
        )
        self.children[name] = process
        self._status()
        return process

    def _check_children(self) -> None:
        if self.stop_requested:
            raise RuntimeError("Bringup was stopped.")
        for name, child in self.children.items():
            if child.poll() is not None:
                raise RuntimeError(f"{name} exited with code {child.returncode}.")

    def _wait(self, label: str, probe, *, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        detail = ""
        while time.monotonic() < deadline:
            self._check_children()
            try:
                ready, detail = probe()
            except (OSError, subprocess.TimeoutExpired) as exc:
                ready, detail = False, f"{type(exc).__name__}: {exc}"
            if ready:
                self.readiness[label] = True
                self._status()
                return
            time.sleep(0.5)
        raise TimeoutError(f"{label} was not ready: {detail[-500:]}")

    def _world_probe(self) -> tuple[bool, str]:
        services = _run(["gz", "service", "-l"], self.env, timeout=4)
        required = {"/world/environment/create", "/world/environment/scene/graph"}
        if services.returncode or not required.issubset(set(services.stdout.splitlines())):
            return False, (services.stderr or services.stdout)[-500:]
        clock = _run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], self.env, timeout=4)
        return clock.returncode == 0 and bool(clock.stdout.strip()), (clock.stderr or clock.stdout)[-500:]

    def _spawn(self) -> None:
        x, y, z, w = _quaternion(self.args.spawn_yaw)
        request = (
            f"sdf_filename: {json.dumps(self.args.drone_sdf)} name: \"drone\" "
            f"pose: {{ position: {{ x: {self.args.spawn_x}, y: {self.args.spawn_y}, z: {self.args.spawn_z} }} "
            f"orientation: {{ x: {x}, y: {y}, z: {z}, w: {w} }} }}"
        )
        response = _run(
            ["gz", "service", "-s", "/world/environment/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", request],
            self.env,
            timeout=12,
        )
        (self.log_dir / "spawn.log").write_text(response.stdout + response.stderr, encoding="utf-8")
        if response.returncode or re.search(r"\bdata\s*:\s*true\b", response.stdout, re.I) is None:
            raise RuntimeError("Gazebo rejected the drone spawn request; see spawn.log.")

    def _entity_probe(self) -> tuple[bool, str]:
        response = _run(
            ["gz", "service", "-s", "/world/environment/scene/graph", "--reqtype", "gz.msgs.Empty", "--reptype", "gz.msgs.StringMsg", "--timeout", "4000", "--req", ""],
            self.env,
            timeout=5,
        )
        value = response.stdout.replace(r'\"', '"').replace(r"\n", "\n")
        found = re.search(r'\bdrone \(\d+\)', value) is not None
        return response.returncode == 0 and found, (response.stderr or response.stdout)[-500:]

    def _observed_parameters(self) -> dict[str, int | float]:
        """Read PX4's last logged value for each requested startup parameter."""

        path = self.log_dir / "px4.log"
        if not path.exists():
            return {}
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        observed: dict[str, int | float] = {}
        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
        for name in self.px4_parameters:
            for line in reversed(lines):
                if re.search(rf"\b{re.escape(name)}\b", line) is None:
                    continue
                match = re.search(rf"-> new:\s*({number})", line)
                if match is None:
                    match = re.search(rf":\s*({number})(?:\s|$)", line)
                if match:
                    value = float(match.group(1))
                    observed[name] = int(value) if value.is_integer() else value
                    break
        return observed

    def _ros_probe(self) -> tuple[bool, str]:
        response = _run(["ros2", "topic", "list", "--no-daemon"], self.env, timeout=5)
        topics = set(response.stdout.splitlines())
        if response.returncode:
            return False, (response.stderr or response.stdout)[-500:]
        if "/clock" in topics and not self.readiness["ros_clock"]:
            sample = _run(
                ["ros2", "topic", "echo", "/clock", "rosgraph_msgs/msg/Clock", "--once", "--no-daemon", "--timeout", "3"],
                self.env,
                timeout=5,
            )
            self.readiness["ros_clock"] = sample.returncode == 0 and bool(sample.stdout.strip())
        agent_log = self.log_dir / "xrce_agent.log"
        if agent_log.exists() and not self.readiness["xrce_session"]:
            self.readiness["xrce_session"] = "session established" in agent_log.read_text(encoding="utf-8", errors="replace")
        px4_log = self.log_dir / "px4.log"
        if px4_log.exists() and not self.readiness["px4_startup"]:
            px4_output = px4_log.read_text(encoding="utf-8", errors="replace")
            self.readiness["px4_startup"] = "Startup script returned successfully" in px4_output
        status = next((topic for topic in topics if topic.startswith("/fmu/out/vehicle_status")), None)
        publisher_count = 0
        if status and self.readiness["xrce_session"]:
            info = _run(["ros2", "topic", "info", "--verbose", "--no-daemon", status], self.env, timeout=5)
            match = re.search(r"Publisher count:\s*(\d+)", info.stdout)
            if info.returncode == 0 and match:
                publisher_count = int(match.group(1))
        self.readiness_details = {
            "px4_vehicle_status_topic": status,
            "px4_vehicle_status_publishers": publisher_count,
            "px4_parameters_observed": self._observed_parameters(),
        }
        self.readiness["px4_telemetry"] = publisher_count > 0
        self._status()
        ready = all(self.readiness.values())
        return ready, (
            f"clock_sample={self.readiness['ros_clock']}, xrce_session={self.readiness['xrce_session']}, "
            f"px4_startup={self.readiness['px4_startup']}, "
            f"px4_topic={status or '<missing>'}, publishers={publisher_count}"
        )

    def run(self) -> None:
        self._status()
        self.phase = "gazebo_world"
        self._start("gazebo", ["gz", "sim", "-r", "-s", self.args.world_sdf], self.env)
        self._wait("gazebo_world", self._world_probe, seconds=90)
        self.phase = "drone_entity"
        self._status()
        self._spawn()
        self._wait("drone_entity", self._entity_probe, seconds=20)
        self.phase = "px4_ros"
        self._status()
        self._start("xrce_agent", ["MicroXRCEAgent", "udp4", "-p", "8888"], self.env)
        px4_env = self.env.copy()
        px4_root = px4_env.get("PX4_AUTOPILOT_DIR", "/opt/PX4-Autopilot")
        px4_env["PX4_AUTOPILOT_DIR"] = px4_root
        px4_env.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_MODEL_NAME": "drone",
                "PX4_UXRCE_DDS_PORT": "8888",
                **{f"PX4_PARAM_{name}": str(value) for name, value in self.px4_parameters.items()},
            }
        )
        px4 = self._start("px4", [str(Path(px4_root) / "build/px4_sitl_default/bin/px4")], px4_env, px4=True)
        if px4.stdin is not None:
            commands = [f"param set {name} {value}" for name, value in self.px4_parameters.items()]
            commands.extend(f"param show {name}" for name in self.px4_parameters)
            px4.stdin.write(("\n".join(commands) + "\n").encode())
            px4.stdin.flush()
        bridges = _sensor_bridges(Path(self.args.drone_sdf))
        self._start("ros_gz_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridges], self.env)
        self._wait("px4_telemetry", self._ros_probe, seconds=110)
        self.phase = "ready"
        self._status()
        while not self.stop_requested:
            self._check_children()
            time.sleep(0.25)

    def cleanup(self) -> None:
        for child in reversed(list(self.children.values())):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for child in reversed(list(self.children.values())):
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=2)
            if child.stdin is not None:
                child.stdin.close()
        for stream in self.logs.values():
            stream.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    for field in ("world_sdf", "drone_sdf", "log_dir", "status_file", "resource_root"):
        parser.add_argument("--" + field.replace("_", "-"), required=True)
    for field in ("spawn_x", "spawn_y", "spawn_z", "spawn_yaw"):
        parser.add_argument("--" + field.replace("_", "-"), type=float, required=True)
    parser.add_argument("--px4-parameters", default="{}")
    args, _ = parser.parse_known_args()
    supervisor = Supervisor(args)

    def stop(_signum, _frame) -> None:
        supervisor.stop_requested = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        supervisor.run()
    except Exception as exc:  # Report the observed failed phase before cleanup.
        supervisor._status(error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        supervisor.cleanup()


if __name__ == "__main__":
    main()
