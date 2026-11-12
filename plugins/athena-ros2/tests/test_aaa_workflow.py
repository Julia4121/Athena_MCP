"""Failure and evidence checks for the host AAA wrapper, without hardware."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import aaa_workflow as aaa


class AAAWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "workspace"
        self.workspace.mkdir()
        self.settings = aaa.Settings(self.root / "platform", self.root / "atlas", self.workspace)
        (self.workspace / "agent/athena/simulation/run-1").mkdir(parents=True)

    def test_simulation_evidence_must_match_clean_deployed_commit(self) -> None:
        path = self.workspace / "agent/athena/simulation/run-1/result.json"
        sha = "a" * 40
        result = {
            "test_exit_code": 0,
            "timed_out": False,
            "diagnostics": [],
            "recording": {"metadata_written": True},
            "source_revision": {
                "repository_path": "/home/ws/src/new_sensor",
                "sha": sha,
                "dirty": False,
            },
        }
        result["source_revision_end"] = result["source_revision"].copy()
        path.write_text(json.dumps(result))
        self.assertEqual(
            aaa._verified_simulation(self.settings, "src/new_sensor", sha, "/home/ws/agent/athena/simulation/run-1/result.json"),
            str(path),
        )
        result["source_revision_end"]["dirty"] = True
        path.write_text(json.dumps(result))
        with self.assertRaisesRegex(ValueError, "clean, published"):
            aaa._verified_simulation(self.settings, "src/new_sensor", sha, str(path))
        result["source_revision_end"]["dirty"] = False
        result["source_revision"]["dirty"] = True
        path.write_text(json.dumps(result))
        with self.assertRaisesRegex(ValueError, "clean, published"):
            aaa._verified_simulation(self.settings, "src/new_sensor", sha, str(path))

    def test_deployment_uses_hotspot_when_pi_git_is_unreachable_in_aaa(self) -> None:
        job_dir = self.workspace / "deploy-job"
        job_dir.mkdir()
        source_sha = "a" * 40
        platform_sha = "b" * 40
        request = {
            "repository_path": "src/new_sensor",
            "package_name": "new_sensor",
            "source_sha": source_sha,
            "platform_sha": platform_sha,
            "simulation_result_path": "/home/ws/agent/athena/simulation/run-1/result.json",
            "props_removed_confirmed": True,
        }
        commands: list[list[str]] = []

        def run(arguments: list[str], **_kwargs) -> str:
            commands.append(arguments)
            if "ls-remote" in arguments:
                return "one published ref"
            return ""

        def ssh(_address: str, _command: str, **_kwargs) -> str:
            return f"{platform_sha}\n{source_sha}\n/opt/ros/new_sensor"

        with patch.object(aaa, "_verified_simulation", return_value="/tmp/sim-result.json"), patch.object(
            aaa, "_manifest_repository", return_value="git@github.com:Julia4121/new_sensor.git"
        ), patch.object(aaa, "_require_clean_commit"), patch.object(
            aaa, "_git", side_effect=["git@github.com:Julia4121/drone_platform.git", platform_sha + "\trefs/heads/main", source_sha + "\trefs/heads/main"]
        ), patch.object(aaa, "_pi_git_access", side_effect=[False, True]) as access, patch.object(
            aaa, "_pi_disarmed"
        ) as disarmed, patch.object(aaa, "_run", side_effect=run), patch.object(
            aaa, "_run_logged", return_value=0
        ), patch.object(aaa, "_ssh", side_effect=ssh):
            result = aaa.deploy(self.settings, job_dir, request)
        self.assertTrue(result["used_hotspot"])
        self.assertEqual(access.call_args_list[1].args[0], aaa.HOTSPOT_PI)
        disarmed.assert_called_once_with(self.settings, aaa.HOTSPOT_PI)
        self.assertEqual([command[-1] for command in commands], ["pi", "normal"])

    def _flight_inputs(self, status: str = "passed", disarm_after: bool = True) -> tuple[Path, dict]:
        bringup = self.workspace / "src/athena_01/atlas_bringup"
        (bringup / "config").mkdir(parents=True)
        manifest = bringup / "config/scenario_manifest.json"
        manifest.write_text(json.dumps({"scenarios": [{
            "scenario_id": "test-hover", "feature_package": "new_sensor",
            "scenario_deadline_seconds": 90,
        }]}))
        job_dir = self.workspace / "flight-job"
        job_dir.mkdir()
        request = {
            "deploy_job_id": "d" * 32,
            "bench_job_id": "b" * 32,
            "prepare_job_id": "p" * 32,
            "scenario_id": "test-hover",
            "pi_package": "new_sensor",
            "pi_launch_file": "test.launch.py",
            "pi_launch_arguments": [],
        }
        self.runner_result = {
            "status": status,
            "flight_started": status == "passed",
            "landing_confirmed": status == "passed",
        }
        self.disarm_after = disarm_after
        self.manifest = manifest
        return job_dir, request

    def _mock_jobs(self, job_id: str) -> dict:
        if job_id == "d" * 32:
            return {"kind": "deploy", "state": "succeeded", "result": {
                "package_name": "new_sensor", "source_sha": "a" * 40,
                "simulation_result": "/some/result.json",
                "repository_path": "src/new_sensor",
            }}
        if job_id == "b" * 32:
            return {"kind": "bench", "state": "succeeded", "result": {
                "deploy_job_id": "d" * 32, "package_name": "new_sensor",
                "pi_launch_file": "test.launch.py", "pi_launch_arguments": [],
                "source_sha": "a" * 40,
            }}
        return {"kind": "prepare", "state": "succeeded", "result": {
            "run_id": "run-1", "manifest": str(self.manifest),
            "bringup_sha": "b" * 40,
        }}

    def _mock_runner(self, _command: list[str], _log: Path, *, timeout: int | None) -> int:
        self.assertIsNone(timeout)
        result_path = _log.parent / "flight/result.json"
        result_path.write_text(json.dumps(self.runner_result))
        return 0

    def test_bench_check_stops_launch_after_disarm(self) -> None:
        job_dir = self.workspace / "bench-job"
        job_dir.mkdir()
        request = {
            "deploy_job_id": "d" * 32,
            "pi_launch_file": "test.launch.py",
            "pi_launch_arguments": [],
            "expected_topics": ["/new_sensor/status"],
            "duration_seconds": 5,
            "props_removed_confirmed": True,
        }
        replies = iter(["a" * 40, "", "/new_sensor/status\n/rosout"])
        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_ssh", side_effect=lambda *_args, **_kwargs: next(replies)
        ), patch.object(aaa, "_pi_disarmed"), patch.object(
            aaa, "_start_pi_launch", return_value=(123, "/tmp/test.log")
        ), patch.object(aaa, "_stop_pi_launch") as stop, patch.object(
            aaa.time, "sleep"
        ), patch.object(aaa, "_run", return_value=""):
            result = aaa.check_onboard(self.settings, job_dir, request)
        self.assertEqual(result["observed_topics"], ["/new_sensor/status", "/rosout"])
        self.assertTrue(result["disarmed_confirmed"])
        stop.assert_called_once_with(123)

    def test_bench_check_keeps_launch_if_disarm_is_uncertain(self) -> None:
        job_dir = self.workspace / "bench-job"
        job_dir.mkdir()
        request = {
            "deploy_job_id": "d" * 32, "pi_launch_file": "test.launch.py",
            "pi_launch_arguments": [], "expected_topics": [],
            "duration_seconds": 5, "props_removed_confirmed": True,
        }
        replies = iter(["a" * 40, "", "/rosout"])
        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_ssh", side_effect=lambda *_args, **_kwargs: next(replies)
        ), patch.object(
            aaa, "_pi_disarmed", side_effect=[None, RuntimeError("status unavailable")]
        ), patch.object(aaa, "_start_pi_launch", return_value=(123, "/tmp/test.log")), patch.object(
            aaa, "_stop_pi_launch"
        ) as stop, patch.object(aaa.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "Pi check failed"):
                aaa.check_onboard(self.settings, job_dir, request)
        stop.assert_not_called()
        self.assertFalse(json.loads((job_dir / "outcome.json").read_text())["disarmed_confirmed"])

    def test_flight_stops_pi_launch_after_confirmed_disarm(self) -> None:
        job_dir, request = self._flight_inputs()
        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_run", return_value=""
        ), patch.object(aaa, "_require_clean_commit"), patch.object(aaa, "_ssh", return_value="a" * 40), patch.object(
            aaa, "_pi_disarmed"
        ), patch.object(aaa, "_start_pi_launch", return_value=(123, "/tmp/test.log")), patch.object(
            aaa, "_stop_pi_launch"
        ) as stop, patch.object(aaa, "_docker_ros", return_value=["runner"]), patch.object(
            aaa, "_run_logged", side_effect=self._mock_runner
        ):
            result = aaa.flight(self.settings, job_dir, request)
        self.assertTrue(result["disarmed_confirmed"])
        self.assertTrue(result["pi_launch_stopped"])
        stop.assert_called_once_with(123)

    def test_flight_rejects_scenario_for_another_feature(self) -> None:
        job_dir, request = self._flight_inputs()
        manifest_data = json.loads(self.manifest.read_text())
        manifest_data["scenarios"][0]["feature_package"] = "unrelated_package"
        self.manifest.write_text(json.dumps(manifest_data))
        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_require_clean_commit"
        ):
            with self.assertRaisesRegex(ValueError, "feature_package"):
                aaa.flight(self.settings, job_dir, request)

    def test_failed_flight_keeps_pi_launch_when_disarm_is_uncertain(self) -> None:
        job_dir, request = self._flight_inputs(status="failed", disarm_after=False)
        calls = 0

        def disarm(_settings: aaa.Settings, _address: str) -> None:
            nonlocal calls
            calls += 1
            if calls == 3:
                raise RuntimeError("PX4 status unavailable")

        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_run", return_value=""
        ), patch.object(aaa, "_require_clean_commit"), patch.object(aaa, "_ssh", return_value="a" * 40), patch.object(
            aaa, "_pi_disarmed", side_effect=disarm
        ), patch.object(aaa, "_start_pi_launch", return_value=(123, "/tmp/test.log")), patch.object(
            aaa, "_stop_pi_launch"
        ) as stop, patch.object(aaa, "_docker_ros", return_value=["runner"]), patch.object(
            aaa, "_run_logged", side_effect=self._mock_runner
        ):
            with self.assertRaisesRegex(RuntimeError, "did not pass"):
                aaa.flight(self.settings, job_dir, request)
        stop.assert_not_called()
        outcome = json.loads((job_dir / "outcome.json").read_text())
        self.assertFalse(outcome["disarmed_confirmed"])

    def test_operator_cancel_cannot_be_reported_as_success(self) -> None:
        job_dir, request = self._flight_inputs(status="cancelled")
        with patch.object(aaa, "read_job", side_effect=self._mock_jobs), patch.object(
            aaa, "_run", return_value=""
        ), patch.object(aaa, "_require_clean_commit"), patch.object(aaa, "_ssh", return_value="a" * 40), patch.object(
            aaa, "_pi_disarmed"
        ), patch.object(aaa, "_start_pi_launch", return_value=(123, "/tmp/test.log")), patch.object(
            aaa, "_stop_pi_launch"
        ) as stop, patch.object(aaa, "_docker_ros", return_value=["runner"]), patch.object(
            aaa, "_run_logged", side_effect=self._mock_runner
        ):
            with self.assertRaisesRegex(RuntimeError, "did not pass"):
                aaa.flight(self.settings, job_dir, request)
        stop.assert_called_once_with(123)
        self.assertEqual(json.loads((job_dir / "outcome.json").read_text())["runner_result"]["status"], "cancelled")

    def test_failed_job_status_keeps_recovery_outcome(self) -> None:
        job_dir = self.workspace / "agent/athena/aaa/jobs" / ("a" * 32)
        job_dir.mkdir(parents=True)
        request_path = job_dir / "request.json"
        request_path.write_text(json.dumps({"kind": "bench", "payload": {}}))

        def failed_check(_settings: aaa.Settings, path: Path, _payload: dict) -> None:
            aaa._write_json(path / "outcome.json", {"disarmed_confirmed": False})
            raise RuntimeError("probe unavailable")

        with patch.object(aaa.Settings, "from_env", return_value=self.settings), patch.object(
            aaa, "check_onboard", side_effect=failed_check
        ):
            with self.assertRaisesRegex(RuntimeError, "probe unavailable"):
                aaa.run_job(request_path)
        status = json.loads((job_dir / "status.json").read_text())
        self.assertEqual(status["state"], "failed")
        self.assertEqual(status["outcome"], {"disarmed_confirmed": False})


if __name__ == "__main__":
    unittest.main()
