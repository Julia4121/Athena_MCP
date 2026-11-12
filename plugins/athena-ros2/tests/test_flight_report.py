"""Recorded holds must be measured in their own event windows."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scaffold" / "sim_bringup"))

from sim_bringup.flight_report import PositionSample, build_report, longest_stable_hold  # noqa: E402


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
    assert report["checkpoints"][0]["meets_minimum"] is True
    assert report["checkpoints"][1]["meets_minimum"] is False
    assert report["checkpoints"][1]["longest_stable_seconds"] == 0.4
