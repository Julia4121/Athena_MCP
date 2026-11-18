"""Recorded holds must be measured in their own event windows."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scaffold" / "sim_bringup"))

from sim_bringup.flight_report import (
    PositionSample,
    _topic_gap_summaries,
    build_report,
    longest_stable_hold,
)


def _sample(seconds: float, x: float, speed: float = 0.0) -> PositionSample:
    return PositionSample(int(seconds * 1e9), x, 0.0, 2.0, speed)


def test_hold_breaks_on_motion_and_missing_telemetry() -> None:
    samples = [_sample(0, 0), _sample(1, 0), _sample(1.2, 1), _sample(2, 0), _sample(2.2, 0)]
    measured = longest_stable_hold(
        samples, (0, 0, 2),
        horizontal_tolerance_m=0.3, vertical_tolerance_m=0.2, maximum_speed_m_s=0.2,
        start_ns=0, end_ns=3_000_000_000,
    )
    assert measured["longest_stable_seconds"] == 0.2
    assert measured["hold_start_wall_time_ns"] == 2_000_000_000


def test_repeated_position_uses_distinct_step_windows(tmp_path: Path) -> None:
    # The second visit must not borrow the longer hover from the first visit.
    samples = [_sample(t, 0) for t in (0, 0.2, 0.4, 0.6, 0.8, 1.0, 2.0, 2.2, 2.4)]
    checkpoints = [
        {
            "name": name, "step": name, "target_forward_right_agl_m": [0, 0, 2],
            "minimum_seconds": 0.5, "horizontal_tolerance_m": 0.3,
            "vertical_tolerance_m": 0.2, "maximum_speed_m_s": 0.2,
        }
        for name in ("first", "second")
    ]
    events = [
        {"event": "step_completed", "step": "first", "wall_time_ns": 1_000_000_000},
        {"event": "step_completed", "step": "second", "wall_time_ns": 2_400_000_000},
    ]
    report = build_report(tmp_path, samples, [], [], checkpoints, events)
    assert report["checkpoints"][0]["longest_stable_seconds"] == 1.0
    assert report["checkpoints"][1]["longest_stable_seconds"] == 0.4
    assert "meets_minimum" not in report["checkpoints"][0]


def test_report_summarizes_transitions_gaps_battery_manual_and_contacts(tmp_path: Path) -> None:
    samples = [_sample(t, x=0) for t in (22, 23, 24, 24.2, 25)]
    statuses = [
        {"wall_time_ns": 22_000_000_000, "nav_state": 1, "nav_state_name": "altctl",
         "arming_state": 1, "arming_state_name": "disarmed", "failsafe": False},
        {"wall_time_ns": 23_000_000_000, "nav_state": 2, "nav_state_name": "posctl",
         "arming_state": 2, "arming_state_name": "armed", "failsafe": False},
        {"wall_time_ns": 24_000_000_000, "nav_state": 2, "nav_state_name": "posctl",
         "arming_state": 2, "arming_state_name": "armed", "failsafe": True},
        {"wall_time_ns": 25_000_000_000, "nav_state": 10, "nav_state_name": "auto_land",
         "arming_state": 1, "arming_state_name": "disarmed", "failsafe": False},
    ]
    land_states = [{"wall_time_ns": 25_000_000_000, "landed": True}]
    contacts = [{
        "topic": "/drone/contact", "collision_a": "gear", "collision_b": "floor",
        "count": 3, "first_wall_time_ns": 23_600_000_000,
        "last_wall_time_ns": 23_800_000_000,
    }]
    (tmp_path / "contacts.jsonl").write_text(
        "".join(json.dumps(contact) + "\n" for contact in contacts), encoding="utf-8"
    )
    observations = {
        "topic_sample_gaps": {
            "/fmu/out/vehicle_status": {
                "sample_count": 3, "maximum_gap_seconds": 1.0,
                "first_wall_time_ns": 21_000_000_000,
                "largest_gaps": [{"duration_seconds": 1.0}],
            }
        },
        "failsafe_flag_samples": [
            {"wall_time_ns": 22_000_000_000, "topic": "/fmu/out/failsafe_flags",
             "flags": {"offboard_signal_lost": False}},
            {"wall_time_ns": 24_000_000_000, "topic": "/fmu/out/failsafe_flags",
             "flags": {"offboard_signal_lost": True}},
        ],
        "battery_samples": {
            "/fmu/out/battery_status": [
                {"wall_time_ns": 23_000_000_000, "values": {"voltage_v": 16.2}},
                {"wall_time_ns": 25_000_000_000, "values": {"voltage_v": 15.9}},
            ]
        },
        "manual_control_samples": {
            "/fmu/out/manual_control_setpoint": [
                {"wall_time_ns": 23_900_000_000, "values": {"throttle": 0.3}},
                {"wall_time_ns": 24_000_000_000, "values": {"throttle": 0.0}},
                {"wall_time_ns": 24_200_000_000, "values": {"throttle": 0.0}},
            ]
        },
    }

    report = build_report(tmp_path, samples, statuses, land_states, [], [], observations)

    assert report["state_transitions"] == statuses
    assert [item["changed_flags"] for item in report["failsafe_flag_transitions"]] == [
        {"offboard_signal_lost": False}, {"offboard_signal_lost": True}
    ]
    assert report["topic_sample_gaps"]["/fmu/out/vehicle_status"]["maximum_gap_seconds"] == 1.0
    assert report["battery"]["/fmu/out/battery_status"]["sample_count"] == 2
    manual = report["manual_control_after_failsafe"]
    assert manual["failsafe_activation_wall_time_ns"] == 24_000_000_000
    assert manual["topics"]["/fmu/out/manual_control_setpoint"]["samples"] == [
        {"wall_time_ns": 24_000_000_000, "values": {"throttle": 0.0}},
        {"wall_time_ns": 24_200_000_000, "values": {"throttle": 0.0}},
    ]
    context = report["contacts"][0]["time_context"]
    assert context["start_seconds_from_bag_start"] == 2.6
    assert context["status_at_start"]["failsafe"] is False
    assert context["position_at_end"]["agl_m"] == 2.0


def test_report_handles_optional_battery_and_manual_data(tmp_path: Path) -> None:
    report = build_report(tmp_path, [_sample(0, 0)], [], [], [], [])
    assert report["battery"] == {}
    assert report["manual_control_after_failsafe"] == {
        "failsafe_activation_wall_time_ns": None,
        "topics": {},
    }


def test_topic_gap_summary_reports_largest_receive_time_gaps() -> None:
    summary = _topic_gap_summaries({
        "/fmu/out/vehicle_status": [0, 100_000_000, 500_000_000, 600_000_000]
    })
    assert summary["/fmu/out/vehicle_status"] == {
        "sample_count": 4,
        "first_wall_time_ns": 0,
        "last_wall_time_ns": 600_000_000,
        "gap_count": 3,
        "maximum_gap_seconds": 0.4,
        "largest_gaps": [
            {"start_wall_time_ns": 100_000_000, "end_wall_time_ns": 500_000_000, "duration_seconds": 0.4},
            {"start_wall_time_ns": 0, "end_wall_time_ns": 100_000_000, "duration_seconds": 0.1},
            {"start_wall_time_ns": 500_000_000, "end_wall_time_ns": 600_000_000, "duration_seconds": 0.1},
        ],
    }
