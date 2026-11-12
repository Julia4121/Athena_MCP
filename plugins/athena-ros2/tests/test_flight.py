"""Flight geometry and control decisions without a running simulator."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "scaffold" / "sim_bringup")
)

from sim_bringup import flight


def test_pose_uses_launch_relative_neu_and_rejects_nonfinite_values() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._origin = (5.0, -2.0, 0.3)
    session._closed = False
    assert session._absolute(flight.Pose(1.0, 2.0, 1.5)) == (6.0, 0.0, -1.2)
    with pytest.raises(ValueError, match="finite"):
        flight.Pose(math.nan, 0, 1)


def test_path_projection_cannot_jump_to_later_leg_at_crossing() -> None:
    points = [(-1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0), (1.0, -1.0, 0.0)]
    lengths = flight._route_lengths(points)
    early, _ = flight._project_progress((0, 0, 0), points, lengths, 0, 1.8)
    late, _ = flight._project_progress((0, 0, 0), points, lengths, 5.5, 6.8)
    assert early == pytest.approx(math.sqrt(2))
    assert late > 6.0


def test_path_yaw_interpolates_across_short_angle() -> None:
    _, yaw = flight._route_pose(
        [(0, 0, 0), (1, 0, 0)],
        [math.radians(170), math.radians(-170)],
        [0, 1],
        0.5,
    )
    assert abs(yaw) == pytest.approx(math.pi)


def test_position_and_stabilized_controls_use_different_axes() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._position = SimpleNamespace(
        x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, heading=0.0
    )
    session._set_manual = Mock()
    session._mode = "position"
    session._control((1.0, 0.0, -1.0), 0.0, 0.7)
    position_axes = session._set_manual.call_args.kwargs
    assert position_axes["pitch"] > 0
    assert position_axes["throttle"] > 0
    session._mode = "stabilized"
    session._control((0.0, 0.0, -1.0), 0.0, 0.0)
    stabilized_axes = session._set_manual.call_args.kwargs
    assert stabilized_axes["throttle"] > 0
    assert stabilized_axes["pitch"] == 0


def test_mode_takeover_neutralizes_input() -> None:
    class Status:
        ARMING_STATE_ARMED = 2
        NAVIGATION_STATE_POSCTL = 2

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._origin = (0.0, 0.0, 0.0)
    session._closed = False
    session._mode = "position"
    session._types = {"status": Status}
    session._status = SimpleNamespace(
        nav_state=18, arming_state=2, failsafe=False, failsafe_and_user_took_over=False
    )
    session._position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    session._status_received_ns = 1
    session._position_received_ns = 1
    session._land = None
    session._fresh = Mock(return_value=True)
    session._valid_position = Mock(return_value=True)
    session._neutral_manual = Mock()
    with pytest.raises(RuntimeError, match="left the commanded armed mode"):
        session._require_active()
    session._neutral_manual.assert_called_once()


def test_failed_mode_confirmation_keeps_previous_mode() -> None:
    class Status:
        NAVIGATION_STATE_ALTCTL = 1

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._origin = (0.0, 0.0, 0.0)
    session._closed = False
    session._mode = "offboard"
    session._types = {
        "status": Status,
        "command": SimpleNamespace(VEHICLE_CMD_DO_SET_MODE=176),
    }
    session._status = SimpleNamespace(nav_state=14)
    session._status_received_ns = 1
    session._fresh = Mock(return_value=True)
    session._manual_stream = SimpleNamespace(cycles_sent=0)
    session._ensure_manual_stream = Mock()
    session._neutral_manual = Mock()
    session._prime = Mock()
    session._command = Mock()
    session._wait_state = Mock(side_effect=TimeoutError("mode not confirmed"))
    session._stop_offboard = Mock()
    with pytest.raises(TimeoutError, match="mode not confirmed"):
        session.set_mode("altitude")
    assert session._mode == "offboard"
    session._stop_offboard.assert_not_called()


def test_path_requires_endpoint_to_remain_settled(monkeypatch) -> None:
    now = 0.0
    monkeypatch.setattr(flight, "time", SimpleNamespace(monotonic=lambda: now))
    session = flight.FlightSession.__new__(flight.FlightSession)
    sample = SimpleNamespace(vx=0.0, vy=0.0, vz=0.0, heading=0.0)
    session._position_received_ns = 0
    session._mode = "offboard"
    session._require_active = Mock()
    session._position_tuple = Mock(return_value=(1.0, 0.0, 0.0))
    session._control = Mock()
    session._snapshot = Mock(return_value={})
    session._neutral_manual = Mock()

    def next_position(_previous_ns, _deadline):
        nonlocal now
        now += 0.1
        session._position_received_ns += 1
        return sample

    session._next_position = next_position
    result = session._navigate(
        [(0, 0, 0), (1, 0, 0)],
        [0, 0],
        speed_mps=1.0,
        tolerance_m=0.25,
        deadline=3.0,
    )
    assert result["endpoint_error_m"] == 0
    assert result["duration_seconds"] >= 1.5


def test_path_rejects_duplicate_consecutive_points() -> None:
    with pytest.raises(ValueError, match="different positions"):
        flight._route_lengths([(0, 0, 0), (0, 0, 0)])


def test_path_timeout_reports_target_and_progress() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._position_received_ns = 1
    session._require_active = Mock()
    session._next_position = Mock(side_effect=TimeoutError("position wait ended"))
    session._state = Mock(
        side_effect=lambda target, progress: (
            f"target_ned={target}, progress_m={progress}"
        )
    )
    session._neutral_manual = Mock()
    with pytest.raises(
        TimeoutError, match=r"target_ned=\(1, 0, 0\), progress_m=0.0.*length_m=1.0"
    ):
        session._navigate(
            [(0, 0, 0), (1, 0, 0)],
            [0, 0],
            speed_mps=1,
            tolerance_m=0.25,
            deadline=flight.time.monotonic() + 1,
        )
