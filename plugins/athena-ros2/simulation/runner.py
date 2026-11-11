"""Run Codex-authored tests beside a bounded Gazebo/PX4 bringup."""

from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import shlex
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

from pydantic import BaseModel, Field, field_validator

from simulation import config


class SpawnPose(BaseModel):
    """Gazebo world pose in metres and radians."""

    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float = 0.0

    @field_validator("x_m", "y_m", "z_m", "yaw_rad")
    @classmethod
    def finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("spawn pose values must be finite")
        return value


class ApplicationLaunch(BaseModel):
    """ROS launch entry point for the implementation under test."""

    package: str = Field(min_length=1)
    file: str = Field(min_length=1)
    arguments: dict[str, str] = Field(default_factory=dict)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_status(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _pid_active(pid: int) -> bool:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except (FileNotFoundError, ProcessLookupError):
        return False
    state = raw.rsplit(")", 1)[1].strip().split()[0]
    return state != "Z"


def _cleanup_bringup_children(status_path: Path) -> dict[str, bool]:
    """Stop child process groups even when ros2 launch exits before its node."""

    status = _read_status(status_path)
    pids: dict[str, int] = {}
    supervisor_pid = status.get("supervisor_pid")
    if isinstance(supervisor_pid, int):
        pids["supervisor"] = supervisor_pid
    children = status.get("processes", {})
    if isinstance(children, dict):
        for name, value in children.items():
            if isinstance(value, dict) and isinstance(value.get("pid"), int):
                pids[str(name)] = value["pid"]
    for pid in pids.values():
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and any(_pid_active(pid) for pid in pids.values()):
        time.sleep(0.1)
    for pid in pids.values():
        if _pid_active(pid):
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return {name: _pid_active(pid) for name, pid in pids.items()}


def _source_env(setup_files: list[Path], inherited: dict[str, str]) -> dict[str, str]:
    source_commands = " && ".join(f"source {shlex.quote(str(path))} >/dev/null" for path in setup_files)
    result = subprocess.run(
        ["bash", "-c", f"{source_commands} && env -0"],
        env=inherited,
        capture_output=True,
        timeout=20,
        check=False,
    )
    if result.returncode:
        raise RuntimeError("Unable to source ROS workspace: " + result.stderr.decode(errors="replace")[-1000:])
    environment = {}
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode(errors="surrogateescape")] = value.decode(errors="surrogateescape")
    return environment


def _start(command: list[str], *, env: dict[str, str], cwd: Path, log_path: Path) -> tuple[subprocess.Popen[bytes], object]:
    log = log_path.open("ab", buffering=0)
    try:
        process = subprocess.Popen(
            command,
            env=env,
            cwd=cwd,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
    except BaseException:
        log.close()
        raise
    return process, log


def _stop(process: subprocess.Popen[bytes] | None, *, grace: float = config.SHUTDOWN_GRACE_SECONDS) -> int | None:
    if process is None:
        return None
    # The group can outlive its leader (for example, a crashed launch process).
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)
    return process.returncode


def _effective_drone_sdf(drone: Path, world: Path, run_dir: Path) -> tuple[Path, bool]:
    """Avoid loading the Gazebo Sensors system twice for configurator SDFs."""

    world_element = ET.parse(world).getroot().find("world")
    if world_element is None or not any(
        plugin.get("filename") == "gz-sim-sensors-system"
        for plugin in world_element.findall("plugin")
    ):
        return drone, False
    tree = ET.parse(drone)
    model = tree.getroot().find("model")
    assert model is not None  # Checked by _preflight.
    duplicate_plugins = [
        plugin for plugin in model.findall("plugin")
        if plugin.get("filename") == "gz-sim-sensors-system"
    ]
    if not duplicate_plugins:
        return drone, False
    for plugin in duplicate_plugins:
        model.remove(plugin)
    effective = run_dir / "effective_drone.sdf"
    ET.indent(tree, space="  ")
    tree.write(effective, encoding="utf-8", xml_declaration=True)
    with effective.open("ab") as stream:
        stream.write(b"\n")
    return effective, True


def _preflight(workspace: Path, drone: Path, world: Path, timeout_seconds: int) -> None:
    if not workspace.is_dir() or not (workspace / "install/setup.bash").is_file():
        raise ValueError("Build the ROS workspace with colcon first; install/setup.bash is required.")
    if not drone.is_file() or not world.is_file():
        raise ValueError("Both drone_sdf and world_sdf must be existing files in this container.")
    if not 1 <= timeout_seconds <= config.MAX_TEST_TIMEOUT_SECONDS:
        raise ValueError(f"timeout_seconds must be between 1 and {config.MAX_TEST_TIMEOUT_SECONDS}.")
    try:
        drone_model = ET.parse(drone).getroot().find("model")
        world_element = ET.parse(world).getroot().find("world")
    except ET.ParseError as exc:
        raise ValueError(f"Unable to parse simulation SDF: {exc}") from exc
    if drone_model is None or drone_model.get("name") != "drone":
        raise ValueError("The X500-compatible drone SDF needs a model named 'drone'.")
    if world_element is None or world_element.get("name") != "environment":
        raise ValueError("The room SDF needs a world named 'environment'.")
    motor_plugins = [
        item
        for item in drone_model.findall(".//plugin")
        if item.get("filename") == "gz-sim-multicopter-motor-model-system"
    ]
    if len(motor_plugins) != 4:
        raise ValueError("PX4 X500 requires four Gazebo motor model plugins in the drone SDF.")
    sensor_types = {(sensor.get("type") or "").lower() for sensor in drone_model.findall(".//sensor")}
    missing = {"imu", "magnetometer", "air_pressure"} - sensor_types
    if missing:
        raise ValueError("The X500 drone SDF is missing PX4 sensors: " + ", ".join(sorted(missing)))


def _result_path(run_dir: Path) -> Path:
    return run_dir / "result.json"


def run_test(
    workspace_path: str,
    drone_sdf: str,
    world_sdf: str,
    spawn_pose: SpawnPose,
    application_launch: ApplicationLaunch,
    test_argv: list[str],
    timeout_seconds: int = config.TEST_TIMEOUT_SECONDS,
    record_topics: list[str] | None = None,
) -> dict[str, object]:
    """Return observed execution facts and artifacts, without a causal verdict."""

    workspace = Path(workspace_path).expanduser().resolve()
    drone = Path(drone_sdf).expanduser().resolve()
    world = Path(world_sdf).expanduser().resolve()
    if not workspace.is_dir():
        raise ValueError("workspace_path must be an existing directory in this container.")
    run_root = workspace / "agent" / "athena" / "simulation"
    run_root.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    run_dir = run_root / run_id
    run_dir.mkdir()
    result: dict[str, object] = {
        "run_id": run_id,
        "artifact_dir": str(run_dir),
        "last_phase": "preflight",
        "readiness": {},
        "process_exit_codes": {},
        "test_exit_code": None,
        "timed_out": False,
        "timeout_phase": None,
        "diagnostics": [],
        "artifacts": {"result": str(_result_path(run_dir))},
    }
    processes: dict[str, subprocess.Popen[bytes]] = {}
    logs: list[object] = []
    lock = None
    startup_status_path = run_dir / "startup_status.json"
    try:
        if not test_argv or any(not isinstance(part, str) or not part for part in test_argv):
            raise ValueError("test_argv must be a non-empty list of command arguments.")
        if record_topics and any(not topic.startswith("/") for topic in record_topics):
            raise ValueError("record_topics must contain absolute ROS topic names.")
        _preflight(workspace, drone, world, timeout_seconds)
        # PX4, XRCE, and the Gazebo service names are shared across workspaces.
        lock = Path("/tmp/athena-ros2-simulation.lock").open("w")
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another Athena simulation run is active in this container.") from exc
        effective_drone, normalized = _effective_drone_sdf(drone, world, run_dir)
        result["preparation"] = {
            "effective_drone_sdf": str(effective_drone),
            "removed_duplicate_sensors_plugin": normalized,
        }
        if normalized:
            result["artifacts"]["effective_drone_sdf"] = str(effective_drone)
        result["inputs"] = {
            "drone_sdf": str(drone),
            "drone_sha256": _sha256(drone),
            "world_sdf": str(world),
            "world_sha256": _sha256(world),
            "spawn_pose": spawn_pose.model_dump(),
            "application_launch": application_launch.model_dump(),
            "test_argv": test_argv,
            "timeout_seconds": timeout_seconds,
            "record_topics": record_topics or [],
        }
        base_env = _source_env([Path("/opt/ros/jazzy/setup.bash"), workspace / "install/setup.bash"], os.environ.copy())
        result["last_phase"] = "scaffold_build"
        build_command = [
            "colcon", "--log-base", str(run_dir / "scaffold_colcon_logs"),
            "build", "--base-paths", str(config.SCAFFOLD_ROOT),
            "--build-base", str(run_dir / "scaffold_build"),
            "--install-base", str(run_dir / "scaffold_install"),
            "--packages-select", "sim_bringup",
        ]
        built = subprocess.run(build_command, cwd=run_dir, env=base_env, capture_output=True, text=True, timeout=180, check=False)
        (run_dir / "scaffold_build.log").write_text(built.stdout + built.stderr, encoding="utf-8")
        result["artifacts"]["scaffold_build_log"] = str(run_dir / "scaffold_build.log")
        if built.returncode:
            raise RuntimeError(f"sim_bringup colcon build exited {built.returncode}; see scaffold_build.log.")
        run_env = _source_env(
            [Path("/opt/ros/jazzy/setup.bash"), workspace / "install/setup.bash", run_dir / "scaffold_install/setup.bash"],
            base_env,
        )
        resource_roots = [str(config.MODEL_ROOT), str(config.ENVIRONMENT_MODELS), str(drone.parent.parent), str(drone.parent), str(world.parent)]
        run_env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resource_roots + [run_env.get("GZ_SIM_RESOURCE_PATH", "")])
        run_env["SDF_PATH"] = os.pathsep.join(resource_roots + [run_env.get("SDF_PATH", "")])
        run_env["GZ_PARTITION"] = "athena_" + run_id.replace("-", "_")
        if not Path("/dev/dri").exists():
            run_env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        result["last_phase"] = "bringup"
        bringup_command = [
            "ros2", "launch", "sim_bringup", "simulation.launch.py",
            f"world_sdf:={world}", f"drone_sdf:={effective_drone}",
            f"spawn_x:={spawn_pose.x_m}", f"spawn_y:={spawn_pose.y_m}",
            f"spawn_z:={spawn_pose.z_m}", f"spawn_yaw:={spawn_pose.yaw_rad}",
            f"log_dir:={run_dir / 'process_logs'}", f"status_file:={startup_status_path}",
            f"resource_root:={config.MODEL_ROOT}",
        ]
        bringup, log = _start(bringup_command, env=run_env, cwd=workspace, log_path=run_dir / "bringup.log")
        processes["bringup"] = bringup
        logs.append(log)
        result["artifacts"]["bringup_log"] = str(run_dir / "bringup.log")
        result["artifacts"]["process_logs"] = str(run_dir / "process_logs")
        result["artifacts"]["startup_status"] = str(startup_status_path)
        deadline = time.monotonic() + config.STARTUP_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            status = _read_status(startup_status_path)
            result["readiness"] = status.get("readiness", {})
            result["readiness_details"] = status.get("readiness_details", {})
            if status.get("phase") == "ready":
                break
            if status.get("error") or bringup.poll() is not None:
                if str(status.get("error", "")).startswith("TimeoutError:"):
                    result["timed_out"] = True
                    result["timeout_phase"] = "bringup"
                raise RuntimeError("sim_bringup stopped before readiness: " + str(status.get("error") or bringup.returncode))
            time.sleep(0.5)
        else:
            result["timed_out"] = True
            result["timeout_phase"] = "bringup"
            raise TimeoutError("sim_bringup did not become ready before the startup deadline.")
        result["last_phase"] = "application_launch"
        app_command = [
            "ros2", "launch", application_launch.package, application_launch.file,
            *[f"{key}:={value}" for key, value in application_launch.arguments.items()],
        ]
        app, log = _start(app_command, env=run_env, cwd=workspace, log_path=run_dir / "application.log")
        processes["application"] = app
        logs.append(log)
        result["artifacts"]["application_log"] = str(run_dir / "application.log")
        time.sleep(1)
        if app.poll() is not None:
            raise RuntimeError(f"Application launch exited before the test started ({app.returncode}).")
        if record_topics:
            try:
                bag, log = _start(["ros2", "bag", "record", "-o", str(run_dir / "bag"), "--topics", *record_topics], env=run_env, cwd=workspace, log_path=run_dir / "bag.log")
                processes["bag"] = bag
                logs.append(log)
                result["artifacts"]["bag"] = str(run_dir / "bag")
                result["artifacts"]["bag_log"] = str(run_dir / "bag.log")
            except OSError as exc:
                result["diagnostics"].append(f"Unable to start optional rosbag recorder: {exc}")
        result["last_phase"] = "test"
        test, log = _start(test_argv, env=run_env, cwd=workspace, log_path=run_dir / "test.log")
        processes["test"] = test
        logs.append(log)
        result["artifacts"]["test_log"] = str(run_dir / "test.log")
        deadline = time.monotonic() + timeout_seconds
        while True:
            test_code = test.poll()
            if test_code is not None:
                result["test_exit_code"] = test_code
                result["last_phase"] = "test_finished"
                break
            for name in ("bringup", "application"):
                code = processes[name].poll()
                if code is not None:
                    raise RuntimeError(f"{name} exited while the test was running ({code}).")
            if time.monotonic() >= deadline:
                result["timed_out"] = True
                result["timeout_phase"] = "test"
                result["diagnostics"].append(f"Test command exceeded {timeout_seconds} seconds.")
                break
            time.sleep(0.25)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired, TimeoutError) as exc:
        result["diagnostics"].append(f"{type(exc).__name__}: {exc}")
        if isinstance(exc, (TimeoutError, subprocess.TimeoutExpired)):
            result["timed_out"] = True
            result["timeout_phase"] = result["last_phase"]
    finally:
        result["process_exit_codes"] = {name: process.poll() for name, process in processes.items()}
        cleanup_codes = {name: _stop(process) for name, process in reversed(list(processes.items()))}
        result["cleanup_exit_codes"] = dict(reversed(list(cleanup_codes.items())))
        remaining = _cleanup_bringup_children(startup_status_path)
        result["cleanup_remaining_processes"] = remaining
        if any(remaining.values()):
            result["diagnostics"].append("Some bringup processes remained after cleanup; inspect cleanup_remaining_processes.")
        for log in logs:
            log.close()
        final_status = _read_status(startup_status_path)
        if final_status:
            result["readiness"] = final_status.get("readiness", result["readiness"])
            result["readiness_details"] = final_status.get("readiness_details", result.get("readiness_details", {}))
            result["bringup_status"] = final_status
        _result_path(run_dir).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if lock is not None:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
            lock.close()
    return result
