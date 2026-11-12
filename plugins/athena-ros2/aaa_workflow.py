"""Host operations around drone_platform and the maintained Atlas runner."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from urllib.request import Request, urlopen

import yaml

SHA = re.compile(r"[0-9a-f]{40}")
IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,80}")
REPOSITORY_PATH = re.compile(r"src/[A-Za-z0-9_-]+(?:/[A-Za-z0-9_-]+)*")
AAA_PI = "192.168.200.28"
HOTSPOT_PI = "10.42.0.92"
DASHBOARD_URL = "http://127.0.0.1:8081"
BRIDGE_URL = "http://127.0.0.1:8765"


@dataclass(frozen=True)
class Settings:
    platform: Path
    atlas: Path
    workspace: Path

    @classmethod
    def from_env(cls) -> "Settings":
        platform = Path(os.environ.get("ATHENA_PLATFORM_ROOT", Path.home() / "drone_platform")).expanduser().resolve()
        atlas = Path(os.environ.get("ATHENA_ATLAS_ROOT", Path.home() / "Projects/agentic.worktrees/atlas")).expanduser().resolve()
        workspace = Path(os.environ.get("ATHENA_WORKSPACE", platform / ".workspaces/development")).expanduser().resolve()
        for marker in (
            platform / "scripts/aaa_launching/drone_in_AAA_launch",
            atlas / "scripts/startup_atlas_dashboard.py",
            workspace / "src",
        ):
            if not marker.exists():
                raise ValueError(f"Required Athena path is missing: {marker}")
        return cls(platform, atlas, workspace)

    @property
    def jobs(self) -> Path:
        return self.workspace / "agent/athena/aaa/jobs"


def _write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _tail(path: Path, limit: int = 4000) -> str:
    if not path.is_file():
        return ""
    with path.open("rb") as stream:
        stream.seek(max(0, path.stat().st_size - limit))
        return stream.read().decode(errors="replace")


def _run(arguments: list[str], *, timeout: int = 30, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=timeout, env=env, check=False)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(arguments)} exited {result.returncode}: {(result.stderr or result.stdout)[-1000:]}")
    return result.stdout.strip()


def _run_logged(arguments: list[str], log_path: Path, *, timeout: int | None, env: dict[str, str] | None = None) -> int:
    with log_path.open("ab") as log:
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            return process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise TimeoutError(f"Command exceeded {timeout}s; inspect {log_path}")


def _ssh(address: str, command: str, *, timeout: int = 30) -> str:
    return _run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", f"pi@{address}", command],
        timeout=timeout,
    )


def _git(checkout: Path, *arguments: str) -> str:
    return _run(["git", "-C", str(checkout), *arguments])


def _require_clean_commit(checkout: Path, sha: str) -> None:
    if _git(checkout, "rev-parse", "HEAD") != sha:
        raise ValueError(f"{checkout} is not at expected commit {sha}")
    if _git(checkout, "status", "--porcelain=v1", "--untracked-files=all"):
        raise ValueError(f"{checkout} has unpublished local changes")


def _manifest_repository(settings: Settings, path: str, sha: str, manifest_name: str) -> str:
    if not REPOSITORY_PATH.fullmatch(path):
        raise ValueError("repository_path must be a path under src without traversal")
    manifest = yaml.safe_load((settings.platform / "manifests" / manifest_name).read_text(encoding="utf-8"))
    entry = manifest.get("repositories", {}).get(path)
    if not isinstance(entry, dict) or entry.get("type") != "git" or entry.get("version") != sha:
        raise ValueError(f"{manifest_name} does not pin {path} at {sha}")
    url = entry.get("url")
    if not isinstance(url, str) or not url:
        raise ValueError(f"{manifest_name} has no URL for {path}")
    return url


def _verified_simulation(settings: Settings, path: str, sha: str, raw_result_path: str) -> str:
    result_path = Path(raw_result_path)
    if result_path.is_absolute() and result_path.is_relative_to("/home/ws"):
        result_path = settings.workspace / result_path.relative_to("/home/ws")
    result_path = result_path.resolve()
    if not result_path.is_relative_to(settings.workspace / "agent/athena/simulation"):
        raise ValueError("simulation_result_path must be an Athena simulation result in this workspace")
    result = json.loads(result_path.read_text(encoding="utf-8"))
    revision = result.get("source_revision") or {}
    if (
        result.get("test_exit_code") != 0
        or result.get("timed_out") is True
        or result.get("diagnostics")
        or not result.get("recording", {}).get("metadata_written")
        or revision.get("repository_path") != str(Path("/home/ws") / path)
        or revision.get("sha") != sha
        or revision.get("dirty") is not False
        or result.get("source_revision_end") != revision
    ):
        raise ValueError("Simulation did not pass for the clean, published source commit")
    return str(result_path)


def _pi_git_access(address: str, platform_url: str, source_url: str) -> bool:
    for url in (platform_url, source_url):
        try:
            _ssh(address, "git ls-remote " + shlex.quote(url) + " HEAD >/dev/null", timeout=25)
        except (OSError, RuntimeError, subprocess.TimeoutExpired):
            return False
    return True


def _pi_disarmed(settings: Settings, address: str) -> None:
    script = settings.platform / "scripts/aaa_launching/aaa_px4_check.py"
    output = _run(
        ["/usr/bin/python3", str(script), "disarmed", "--master", f"tcp:{address}:5760"],
        timeout=20,
    )
    try:
        result = json.loads(output.splitlines()[-1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError("PX4 disarm probe returned invalid JSON") from exc
    if result.get("ok") is not True or result.get("armed") is not False:
        raise RuntimeError(f"PX4 disarm is not confirmed: {result}")


def deploy(settings: Settings, job_dir: Path, request: dict) -> dict:
    path = request["repository_path"]
    package = request["package_name"]
    source_sha = request["source_sha"]
    platform_sha = request["platform_sha"]
    simulation_result = _verified_simulation(settings, path, source_sha, request["simulation_result_path"])
    if not request.get("props_removed_confirmed"):
        raise ValueError("An operator must confirm propellers are removed before deployment")
    if not IDENTIFIER.fullmatch(package) or not SHA.fullmatch(source_sha) or not SHA.fullmatch(platform_sha):
        raise ValueError("Invalid package name or commit SHA")
    source_url = _manifest_repository(settings, path, source_sha, "onboard.repos")
    if _manifest_repository(settings, path, source_sha, "development.repos") != source_url:
        raise ValueError("Development and onboard manifests use different source URLs")
    _require_clean_commit(settings.platform, platform_sha)
    _require_clean_commit(settings.workspace / path, source_sha)
    platform_url = _git(settings.platform, "remote", "get-url", "origin")
    if _git(settings.platform, "ls-remote", "origin", "refs/heads/main").split()[0] != platform_sha:
        raise ValueError("Publish the platform main commit before Pi deployment")
    if _git(settings.workspace / path, "ls-remote", "origin", "refs/heads/main").split()[0] != source_sha:
        raise ValueError("Publish the feature main commit before Pi deployment")

    address = AAA_PI
    used_hotspot = False
    try:
        if not _pi_git_access(address, platform_url, source_url):
            network = settings.platform / "scripts/networking/operator_network.sh"
            _run([str(network), "pi"], timeout=90)
            address = HOTSPOT_PI
            used_hotspot = True
            if not _pi_git_access(address, platform_url, source_url):
                raise RuntimeError("The Pi cannot fetch both Git repositories over the hotspot")
        _pi_disarmed(settings, address)
        log = job_dir / "deployment.log"
        remote = "cd ~/drone-platform/repo && python3 scripts/onboard/deploy.py ros"
        code = _run_logged(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", f"pi@{address}", remote],
            log, timeout=3600,
        )
        if code:
            raise RuntimeError(f"Pi ROS deployment exited {code}; inspect {log}: {_tail(log)}")
        check = (
            "git -C ~/drone-platform/repo rev-parse HEAD && "
            "git -C ~/drone-platform/workspace/" + shlex.quote(path) + " rev-parse HEAD && "
            "bash -lc " + shlex.quote(
                "source /opt/ros/jazzy/setup.bash && source ~/drone-platform/workspace/install/setup.bash && ros2 pkg prefix " + shlex.quote(package)
            )
        )
        lines = _ssh(address, check).splitlines()
        if len(lines) < 3 or lines[0] != platform_sha or lines[1] != source_sha:
            raise RuntimeError(f"Pi commit verification failed: {lines}")
        if used_hotspot:
            _run([str(settings.platform / "scripts/networking/operator_network.sh"), "normal"], timeout=60)
        return {
            "platform_sha": platform_sha,
            "source_sha": source_sha,
            "package_prefix": lines[-1],
            "deployment_log": str(log),
            "simulation_result": simulation_result,
            "repository_path": path,
            "package_name": package,
            "used_hotspot": used_hotspot,
        }
    except Exception:
        # On failure, retain the working Pi route for operator recovery.
        raise


def _get_json(url: str, timeout: int = 3) -> dict:
    with urlopen(url, timeout=timeout) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object from {url}")
    return value


def _bringup_package(settings: Settings) -> Path:
    packages = sorted(
        path for path in (settings.workspace / "src").glob("athena_*/atlas_bringup")
        if re.fullmatch(r"athena_\d+", path.parent.name) and (path / "package.xml").is_file()
    )
    if len(packages) != 1:
        raise ValueError("Expected one atlas_bringup under an athena_NN checkout")
    return packages[0]


def _require_bringup_pin(settings: Settings, package: Path) -> str:
    root = package.parent
    key = root.relative_to(settings.workspace).as_posix()
    manifest = yaml.safe_load((settings.platform / "manifests/development.repos").read_text(encoding="utf-8"))
    entry = manifest.get("repositories", {}).get(key)
    sha = entry.get("version") if isinstance(entry, dict) else None
    if not isinstance(sha, str) or not SHA.fullmatch(sha):
        raise ValueError(f"development.repos does not pin the {root.name} bringup checkout")
    _require_clean_commit(root, sha)
    return sha


def _container_path(settings: Settings, host_path: Path) -> str:
    return str(Path("/home/ws") / host_path.resolve().relative_to(settings.workspace))


def _docker_ros(settings: Settings, arguments: list[str], *, detached: bool = False) -> list[str]:
    shell = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/ws/install/setup.bash && exec " + shlex.join(arguments)
    )
    return [
        "docker", "exec", *(["-d"] if detached else []),
        "--user", f"{os.getuid()}:{os.getgid()}",
        "--workdir", "/home/ws", "drone-aaa", "bash", "-lc", shell,
    ]


def _start_dashboard(settings: Settings, job_dir: Path) -> None:
    try:
        health = _get_json(DASHBOARD_URL + "/api/atlas/px4-checker")
    except OSError:
        health = None
    if health is not None:
        if health.get("configured") is not True or health.get("drone_platform_path") != str(settings.platform):
            raise RuntimeError("Port 8081 has an Atlas dashboard configured for another platform")
        return
    log = (job_dir / "dashboard.log").open("ab")
    try:
        subprocess.Popen(
            [sys.executable, str(settings.atlas / "scripts/startup_atlas_dashboard.py"),
             "--host", "127.0.0.1", "--port", "8081", "--drone-platform-path", str(settings.platform)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
    finally:
        log.close()
    for _ in range(30):
        try:
            if _get_json(DASHBOARD_URL + "/api/atlas/px4-checker", 1).get("configured") is True:
                return
        except OSError:
            time.sleep(0.3)
    raise RuntimeError(f"Atlas dashboard did not start; inspect {job_dir / 'dashboard.log'}")


def prepare(settings: Settings, job_dir: Path, run_id: str) -> dict:
    package = _bringup_package(settings)
    bringup_sha = _require_bringup_pin(settings, package)
    # The simulator and AAA use host networking; keep their PX4 agents separate in time.
    _run(["docker", "exec", "drone-development", "flock", "-n", "/tmp/athena-ros2-simulation.lock", "true"], timeout=10)
    build_log = job_dir / "bringup_build.log"
    build = _docker_ros(settings, [
        "colcon", "build", "--base-paths", "/home/ws/src",
        "--packages-select", "atlas_bringup",
    ])
    if _run_logged(build, build_log, timeout=600):
        raise RuntimeError(f"atlas_bringup build failed; inspect {build_log}: {_tail(build_log)}")
    _start_dashboard(settings, job_dir)
    launcher = settings.platform / "scripts/aaa_launching/drone_in_AAA_launch"
    env = {**os.environ, "WORKSPACE": str(settings.workspace)}
    log = job_dir / "aaa_prepare.log"
    code = _run_logged(
        ["/usr/bin/python3", str(launcher), "--prepare-only", "--dashboard-url", DASHBOARD_URL, "--run-id", run_id],
        log, timeout=1800, env=env,
    )
    if code:
        raise RuntimeError(f"AAA preparation failed; inspect {log}: {_tail(log)}")
    bridge_log = settings.workspace / "log/aaa/atlas_bringup.log"
    bridge_log.parent.mkdir(parents=True, exist_ok=True)
    shell = (
        "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && "
        "exec ros2 launch atlas_bringup atlas_bringup.launch.py >>"
        + shlex.quote(_container_path(settings, bridge_log)) + " 2>&1"
    )
    try:
        _get_json(BRIDGE_URL + "/v1/atlas/snapshot", 1)
    except OSError:
        _run([
            "docker", "exec", "-d", "--user", f"{os.getuid()}:{os.getgid()}",
            "drone-aaa", "bash", "-lc", shell,
        ])
    for _ in range(30):
        try:
            _get_json(BRIDGE_URL + "/v1/atlas/snapshot", 1)
            break
        except OSError:
            time.sleep(0.5)
    else:
        raise RuntimeError(f"Atlas bridge did not start; inspect {bridge_log}")
    request = Request(
        DASHBOARD_URL + "/api/atlas/connect",
        data=json.dumps({"host": "127.0.0.1", "port": 8765}).encode(),
        headers={"Content-Type": "application/json"}, method="POST",
    )
    with urlopen(request, timeout=5) as response:
        connected = json.load(response)
    if connected.get("connected") is not True:
        raise RuntimeError(f"Atlas dashboard bridge connection failed: {connected}")
    return {
        "run_id": run_id,
        "dashboard_url": DASHBOARD_URL,
        "prepare_log": str(log),
        "bridge_log": str(bridge_log),
        "manifest": str(package / "config/scenario_manifest.json"),
        "bringup_sha": bringup_sha,
        "bringup_build_log": str(build_log),
    }


def _start_pi_launch(run_id: str, package: str, launch_file: str, arguments: list[str]) -> tuple[int, str]:
    if not IDENTIFIER.fullmatch(package) or not launch_file.endswith(".launch.py") or "/" in launch_file:
        raise ValueError("pi_package and pi_launch_file must name a ROS package and launch file")
    if any(not isinstance(item, str) or not item for item in arguments):
        raise ValueError("pi_launch_arguments must be nonempty strings")
    remote_dir = f"/home/pi/drone-platform/test-runs/{run_id}"
    remote_log = f"{remote_dir}/{package}.log"
    ros = "source /opt/ros/jazzy/setup.bash && source $HOME/drone-platform/workspace/install/setup.bash && exec " + shlex.join(
        ["ros2", "launch", package, launch_file, *arguments]
    )
    command = (
        f"mkdir -p {remote_dir} || exit; "
        f"setsid bash -lc {shlex.quote(ros)} </dev/null >{remote_log} 2>&1 & echo $!"
    )
    output = _ssh(AAA_PI, command)
    pid = int(output.splitlines()[-1])
    return pid, remote_log


def _stop_pi_launch(pid: int) -> None:
    try:
        _ssh(AAA_PI, f"kill -0 -- -{pid}")
    except RuntimeError:
        return
    _ssh(AAA_PI, f"kill -INT -- -{pid}")
    for _ in range(20):
        try:
            _ssh(AAA_PI, f"kill -0 -- -{pid}", timeout=5)
        except RuntimeError:
            return
        time.sleep(0.5)
    raise RuntimeError(f"Pi test launch process group {pid} did not stop after SIGINT")


def check_onboard(settings: Settings, job_dir: Path, request: dict) -> dict:
    """Exercise the deployed launch on a propellers-off, disarmed Pi."""

    if request.get("props_removed_confirmed") is not True:
        raise ValueError("An operator must freshly confirm propellers are removed for the Pi check")
    deploy_job_id = request["deploy_job_id"]
    deploy_job = read_job(deploy_job_id)
    if deploy_job.get("kind") != "deploy" or deploy_job.get("state") != "succeeded":
        raise ValueError("deploy_job_id must identify a successful pinned Pi deployment")
    deployed = deploy_job["result"]
    package = deployed["package_name"]
    launch_file = request["pi_launch_file"]
    arguments = request.get("pi_launch_arguments", [])
    expected_topics = request.get("expected_topics", [])
    if not isinstance(expected_topics, list) or any(
        not isinstance(topic, str) or not topic.startswith("/") or " " in topic
        for topic in expected_topics
    ):
        raise ValueError("expected_topics must contain absolute ROS topic names")
    duration = request.get("duration_seconds", 10)
    if not isinstance(duration, int) or not 5 <= duration <= 60:
        raise ValueError("duration_seconds must be an integer from 5 to 60")
    pi_sha = _ssh(
        AAA_PI,
        "git -C ~/drone-platform/workspace/" + shlex.quote(deployed["repository_path"]) + " rev-parse HEAD",
    )
    if pi_sha != deployed["source_sha"]:
        raise ValueError(f"Pi source commit changed since deployment: {pi_sha}")
    _pi_disarmed(settings, AAA_PI)
    pid, remote_log = _start_pi_launch(job_dir.name, package, launch_file, arguments)
    error: str | None = None
    topics: list[str] = []
    disarmed = False
    stopped = False
    local_log = job_dir / "pi_check.log"
    try:
        time.sleep(duration)
        _ssh(AAA_PI, f"kill -0 -- -{pid}")
        ros = (
            "source /opt/ros/jazzy/setup.bash && "
            "source $HOME/drone-platform/workspace/install/setup.bash && ros2 topic list --no-daemon"
        )
        topics = _ssh(AAA_PI, "bash -lc " + shlex.quote(ros), timeout=30).splitlines()
        missing = sorted(set(expected_topics) - set(topics))
        if missing:
            raise RuntimeError(f"Expected Pi topics were absent: {missing}")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    try:
        _pi_disarmed(settings, AAA_PI)
        disarmed = True
        _stop_pi_launch(pid)
        stopped = True
    except Exception as exc:
        error = (error + "; " if error else "") + f"Pi cleanup or disarm failed: {exc}"
    if stopped:
        try:
            _run([
                "scp", "-q", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                f"pi@{AAA_PI}:{remote_log}", str(local_log),
            ], timeout=30)
        except Exception as exc:
            error = (error + "; " if error else "") + f"Pi log copy failed: {exc}"
    outcome = {
        "deploy_job_id": deploy_job_id,
        "package_name": package,
        "pi_launch_file": launch_file,
        "pi_launch_arguments": arguments,
        "source_sha": pi_sha,
        "pi_launch_pid": pid,
        "pi_launch_log": remote_log,
        "pi_launch_log_copy": str(local_log) if local_log.is_file() else None,
        "observed_topics": topics,
        "expected_topics": expected_topics,
        "disarmed_confirmed": disarmed,
        "pi_launch_stopped": stopped,
        "error": error,
    }
    _write_json(job_dir / "outcome.json", outcome)
    if error or not disarmed or not stopped:
        raise RuntimeError(f"Propellers-off Pi check failed; inspect {job_dir / 'outcome.json'}")
    return outcome


def _emergency_land(settings: Settings, job_dir: Path, run_id: str, scenario_id: str, artifact: Path) -> dict:
    command = _docker_ros(settings, [
        "ros2", "run", "atlas_bringup", "atlas-scenario-runner",
        "--emergency-land", "--scenario-id", scenario_id,
        "--run-id", run_id, "--artifact-dir", _container_path(settings, artifact),
    ])
    log = job_dir / "emergency.log"
    _run_logged(command, log, timeout=120)
    result_path = artifact / "emergency_result.json"
    return json.loads(result_path.read_text(encoding="utf-8")) if result_path.is_file() else {}


def flight(settings: Settings, job_dir: Path, request: dict) -> dict:
    deploy_job = read_job(request["deploy_job_id"])
    if deploy_job.get("kind") != "deploy" or deploy_job.get("state") != "succeeded":
        raise ValueError("deploy_job_id must identify a successful pinned Pi deployment")
    bench_job = read_job(request["bench_job_id"])
    if bench_job.get("kind") != "bench" or bench_job.get("state") != "succeeded":
        raise ValueError("bench_job_id must identify a successful propellers-off Pi check")
    if bench_job["result"]["deploy_job_id"] != request["deploy_job_id"]:
        raise ValueError("The Pi check belongs to another deployment")
    prepare_job = read_job(request["prepare_job_id"])
    if prepare_job.get("kind") != "prepare" or prepare_job.get("state") != "succeeded":
        raise ValueError("prepare_job_id must identify a successful AAA preparation")
    run_id = str(prepare_job["result"]["run_id"])
    scenario_id = request["scenario_id"]
    if not IDENTIFIER.fullmatch(scenario_id):
        raise ValueError("Invalid scenario_id")
    manifest = Path(prepare_job["result"]["manifest"])
    _require_clean_commit(manifest.parent.parent.parent, prepare_job["result"]["bringup_sha"])
    data = json.loads(manifest.read_text(encoding="utf-8"))
    scenario = next((item for item in data["scenarios"] if item["scenario_id"] == scenario_id), None)
    if scenario is None:
        raise ValueError(f"Scenario not found in {manifest}: {scenario_id}")
    deployed = deploy_job["result"]
    if scenario.get("feature_package") != deployed["package_name"]:
        raise ValueError("AAA scenario must identify the deployed feature_package")
    _run(_docker_ros(settings, [
        "python3", "-c",
        "import importlib; importlib.import_module(" + repr("atlas_bringup.scenarios." + scenario_id) + ")",
    ]), timeout=30)
    pi_sha = _ssh(
        AAA_PI,
        "git -C ~/drone-platform/workspace/" + shlex.quote(deployed["repository_path"]) + " rev-parse HEAD",
    )
    if pi_sha != deployed["source_sha"]:
        raise ValueError(f"Pi source commit changed since deployment: {pi_sha}")
    _pi_disarmed(settings, AAA_PI)
    pid: int | None = None
    remote_log: str | None = None
    package = request.get("pi_package", "")
    launch_file = request.get("pi_launch_file", "")
    if not package or not launch_file:
        raise ValueError("Specify the checked Pi package and launch file")
    if package and package != deploy_job["result"]["package_name"]:
        raise ValueError("Pi test launch package differs from the deployed package")
    if (
        package != bench_job["result"]["package_name"]
        or launch_file != bench_job["result"]["pi_launch_file"]
        or request.get("pi_launch_arguments", []) != bench_job["result"]["pi_launch_arguments"]
        or bench_job["result"]["source_sha"] != pi_sha
    ):
        raise ValueError("Flight launch differs from the verified propellers-off Pi check")
    artifact = job_dir / "flight"
    artifact.mkdir()
    result: dict = {}
    runner_error: str | None = None
    try:
        if package:
            pid, remote_log = _start_pi_launch(run_id, package, launch_file, request.get("pi_launch_arguments", []))
            _pi_disarmed(settings, AAA_PI)
            _ssh(AAA_PI, f"kill -0 -- -{pid}")
        command = _docker_ros(settings, [
            "ros2", "run", "atlas_bringup", "atlas-scenario-runner",
            "--manifest", _container_path(settings, manifest),
            "--scenario-id", scenario_id,
            "--run-id", run_id,
            "--attempt", "1",
            "--artifact-dir", _container_path(settings, artifact),
            "--confirm-flight",
        ])
        log = job_dir / "runner.log"
        # Atlas deliberately keeps supervising when landing is uncertain. Do not
        # terminate that process on a host timeout while the vehicle may be armed.
        code = _run_logged(command, log, timeout=None)
        result_path = artifact / "result.json"
        if not result_path.is_file():
            raise RuntimeError(f"Runner returned {code} without a result; inspect {log}")
        result = json.loads(result_path.read_text(encoding="utf-8"))
        if code and result.get("status") == "passed":
            raise RuntimeError(f"Runner exited {code} despite reporting passed; inspect {log}")
    except Exception as exc:
        runner_error = f"{type(exc).__name__}: {exc}"
        try:
            result = _emergency_land(settings, job_dir, run_id, scenario_id, artifact)
        except Exception as emergency_exc:
            runner_error += f"; emergency landing unavailable: {type(emergency_exc).__name__}: {emergency_exc}"
            result = {"status": "emergency_unconfirmed", "landing_confirmed": False}
    disarmed = False
    try:
        _pi_disarmed(settings, AAA_PI)
        disarmed = True
    except Exception:
        pass
    launch_stopped = False
    local_pi_log: str | None = None
    pi_log_copy_error: str | None = None
    if pid is not None and disarmed:
        try:
            _stop_pi_launch(pid)
            launch_stopped = True
        except Exception as exc:
            runner_error = (runner_error + "; " if runner_error else "") + f"Pi test launch cleanup failed: {exc}"
        if launch_stopped and remote_log is not None:
            local_path = job_dir / "pi_launch.log"
            try:
                _run([
                    "scp", "-q", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                    f"pi@{AAA_PI}:{remote_log}", str(local_path),
                ], timeout=30)
                local_pi_log = str(local_path)
            except Exception as exc:
                pi_log_copy_error = str(exc)
    outcome = {
        "run_id": run_id,
        "scenario_id": scenario_id,
        "runner_result": result,
        "runner_error": runner_error,
        "pi_launch_pid": pid,
        "pi_launch_log": remote_log,
        "pi_launch_log_copy": local_pi_log,
        "pi_log_copy_error": pi_log_copy_error,
        "pi_launch_stopped": launch_stopped,
        "disarmed_confirmed": disarmed,
        "artifact_dir": str(artifact),
        "dashboard_url": DASHBOARD_URL,
        "deployed_source_sha": deploy_job["result"]["source_sha"],
        "observed_pi_sha": pi_sha,
        "simulation_result": deploy_job["result"]["simulation_result"],
    }
    _write_json(job_dir / "outcome.json", outcome)
    if not (
        result.get("status") == "passed"
        and result.get("flight_started") is True
        and result.get("landing_confirmed") is True
        and disarmed
        and (pid is None or launch_stopped)
        and runner_error is None
    ):
        raise RuntimeError(f"AAA scenario did not pass with confirmed landing and disarm; inspect {job_dir / 'outcome.json'}")
    return outcome


def start_job(kind: str, payload: dict) -> dict[str, str]:
    settings = Settings.from_env()
    settings.jobs.mkdir(parents=True, exist_ok=True)
    job_id = uuid.uuid4().hex
    job_dir = settings.jobs / job_id
    job_dir.mkdir()
    request = {"kind": kind, "payload": payload}
    request_path = job_dir / "request.json"
    _write_json(request_path, request)
    _write_json(job_dir / "status.json", {"job_id": job_id, "kind": kind, "state": "queued"})
    with (job_dir / "worker.log").open("ab") as log:
        subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--run-job", str(request_path)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
    return {"job_id": job_id, "status_path": str(job_dir / "status.json")}


def read_job(job_id: str) -> dict[str, object]:
    if not re.fullmatch(r"[0-9a-f]{32}", job_id):
        raise ValueError("job_id must be a 32-character hex ID")
    settings = Settings.from_env()
    status_path = settings.jobs / job_id / "status.json"
    if not status_path.is_file():
        raise ValueError(f"Unknown AAA job: {job_id}")
    status = json.loads(status_path.read_text(encoding="utf-8"))
    logs = sorted(status_path.parent.glob("*.log"), key=lambda path: path.stat().st_mtime)
    status["logs"] = [str(path) for path in logs]
    if logs:
        status["latest_log_tail"] = _tail(logs[-1], 2000)
    outcome = status_path.parent / "outcome.json"
    if outcome.is_file() and "outcome" not in status:
        status["outcome"] = json.loads(outcome.read_text(encoding="utf-8"))
    if status.get("kind") in {"prepare", "flight"}:
        try:
            status["dashboard"] = _get_json(DASHBOARD_URL + "/api/atlas/snapshot", 2)
        except Exception:
            pass
    return status


def run_job(request_path: Path) -> None:
    settings = Settings.from_env()
    job_dir = request_path.parent
    job_id = job_dir.name
    request = json.loads(request_path.read_text(encoding="utf-8"))
    kind = request["kind"]
    payload = request["payload"]
    status_path = job_dir / "status.json"
    _write_json(status_path, {"job_id": job_id, "kind": kind, "state": "running"})
    try:
        if kind == "deploy":
            result = deploy(settings, job_dir, payload)
        elif kind == "prepare":
            result = prepare(settings, job_dir, job_id)
        elif kind == "bench":
            result = check_onboard(settings, job_dir, payload)
        elif kind == "flight":
            result = flight(settings, job_dir, payload)
        else:
            raise ValueError(f"Unknown AAA job kind: {kind}")
        _write_json(status_path, {"job_id": job_id, "kind": kind, "state": "succeeded", "result": result})
    except Exception as exc:
        status = {
            "job_id": job_id, "kind": kind, "state": "failed",
            "error": f"{type(exc).__name__}: {exc}",
        }
        outcome = job_dir / "outcome.json"
        if outcome.is_file():
            status["outcome"] = json.loads(outcome.read_text(encoding="utf-8"))
        _write_json(status_path, status)
        raise


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-job", type=Path, required=True)
    args = parser.parse_args()
    run_job(args.run_job)


if __name__ == "__main__":
    main()
