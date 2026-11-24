"""The runner surfaces report evidence and generation errors."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from simulation import runner


def test_auto_report_returns_compact_summary(monkeypatch, tmp_path: Path) -> None:
    (tmp_path / "scenario_events.jsonl").write_text("", encoding="utf-8")

    def generate(command, **_kwargs):
        assert command[-2:] == ["--events", str(tmp_path / "scenario_events.jsonl")]
        (tmp_path / "flight_report.json").write_text(json.dumps({
            "position_samples": 40, "status_samples": 5, "failsafe_observed": True,
            "final_nav_state": 18, "final_arming_state": 1, "final_landed": True,
            "contacts": [{}], "battery": {
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
    assert result["diagnostics"] == []


def test_auto_report_failure_is_diagnostic(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runner.subprocess, "run", lambda command, **_kwargs:
                        subprocess.CompletedProcess(command, 1, "", "bag unreadable"))
    result = {"artifacts": {}, "diagnostics": []}
    runner._generate_flight_report(tmp_path, {}, result)
    assert "bag unreadable" in result["flight_report"]["error"]
    assert "flight_report" not in result["artifacts"]
    assert "Flight report generation failed" in result["diagnostics"][0]
