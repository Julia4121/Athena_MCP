"""The runner surfaces report evidence and generation errors."""

from __future__ import annotations

import fcntl
import json
import subprocess
from pathlib import Path

import pytest
from simulation import runner


def test_runner_sources_available_px4_message_underlay(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("SIM_EVAL_PX4_MSGS_SETUP", "/opt/sim-eval-px4-msgs/install/setup.bash")
    assert runner._ros_setup_files(tmp_path) == [
        Path("/opt/ros/jazzy/setup.bash"),
        Path("/opt/sim-eval-px4-msgs/install/setup.bash"),
        tmp_path / "install/setup.bash",
    ]


def test_auto_report_returns_compact_summary(monkeypatch, tmp_path: Path) -> None:
    (tmp_path / "scenario_events.jsonl").write_text("", encoding="utf-8")

    def generate(command, **_kwargs):
        assert command[-2:] == ["--events", str(tmp_path / "scenario_events.jsonl")]
        (tmp_path / "flight_report.json").write_text(json.dumps({
            "position_samples": 40, "status_samples": 5, "failsafe_observed": True,
            "final_nav_state": 18, "final_arming_state": 1, "final_landed": True,
            "contacts": [
                {"collision_a": "drone::base_link::base_link_collision_3",
                 "collision_b": "room::room_link::floor_collision",
                 "time_context": {"position_at_start": {"speed_m_s": 0.72}}},
                {"collision_a": "drone::base_link::base_link_collision_4",
                 "collision_b": "room::room_link::floor_collision",
                 "time_context": {"position_at_start": {"speed_m_s": 0.83}}},
                {"collision_a": "drone::base_link::base_link_collision_0",
                 "collision_b": "room::room_link::floor_collision",
                 "time_context": {"position_at_start": {"speed_m_s": 2.5}}},
                {"collision_a": "drone::base_link::base_link_collision_3",
                 "collision_b": "room::room_link::floor_collision",
                 "time_context": {"position_at_start": None}},
            ], "battery": {
                "/fmu/out/battery_status": {"readings": [{"values": {"remaining": 0.35}}]},
            },
        }), encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(runner.subprocess, "run", generate)
    result = {"artifacts": {}, "diagnostics": []}
    runner._generate_flight_report(tmp_path, {}, result)
    assert result["artifacts"]["flight_report"] == str(tmp_path / "flight_report.json")
    assert result["flight_report"]["summary"]["battery_first_remaining"] == {
        "/fmu/out/battery_status": 0.35,
    }
    assert result["flight_report"]["summary"]["max_gear_floor_contact_speed_m_s"] == 0.83
    assert result["diagnostics"] == []


def test_auto_report_failure_is_diagnostic(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runner.subprocess, "run", lambda command, **_kwargs:
                        subprocess.CompletedProcess(command, 1, "", "bag unreadable"))
    result = {"artifacts": {}, "diagnostics": []}
    runner._generate_flight_report(tmp_path, {}, result)
    assert "bag unreadable" in result["flight_report"]["error"]
    assert "flight_report" not in result["artifacts"]
    assert "Flight report generation failed" in result["diagnostics"][0]


def test_runner_releases_simulation_lock_when_finalization_fails(monkeypatch, tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    drone = tmp_path / "drone.sdf"
    world = tmp_path / "world.sdf"
    drone.write_text("drone", encoding="utf-8")
    world.write_text("world", encoding="utf-8")
    lock_path = tmp_path / "simulation.lock"
    monkeypatch.setattr(runner, "SIMULATION_LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "_preflight", lambda *_args: None)
    monkeypatch.setattr(runner, "_source_revision", lambda *_args: None)
    monkeypatch.setattr(runner, "_effective_drone_sdf", lambda *_args: (drone, False))

    def fail_startup(*_args):
        raise RuntimeError("startup failed")

    def fail_cleanup(*_args):
        raise OSError("cleanup failed")

    monkeypatch.setattr(runner, "_source_env", fail_startup)
    monkeypatch.setattr(runner, "_cleanup_bringup_children", fail_cleanup)
    with pytest.raises(OSError, match="cleanup failed") as error:
        runner.run_test(
            str(workspace), str(drone), str(world),
            runner.SpawnPose(x_m=0, y_m=0, z_m=0),
            runner.ApplicationLaunch(package="example", file="example.launch.py"),
            ["true"],
        )
    assert error.value is not None
    with lock_path.open("w") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
