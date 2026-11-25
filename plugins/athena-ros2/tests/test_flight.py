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


@pytest.mark.parametrize(("status_age_seconds", "stale"), [(10.0, False), (16.0, True)])
def test_active_check_uses_bounded_status_age_with_fresh_position(
    status_age_seconds: float, stale: bool
) -> None:
    class Status:
        ARMING_STATE_ARMED = 2
        NAVIGATION_STATE_POSCTL = 2

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._origin = (0.0, 0.0, 0.0)
    session._closed = False
    session._mode = "position"
    session._types = {"status": Status}
    session._status = SimpleNamespace(
        arming_state=Status.ARMING_STATE_ARMED,
        nav_state=Status.NAVIGATION_STATE_POSCTL,
        failsafe=False,
        failsafe_and_user_took_over=False,
    )
    session._position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    session._land = None
    session._status_received_ns = flight.time.monotonic_ns() - int(status_age_seconds * 1e9)
    session._position_received_ns = flight.time.monotonic_ns()
    session._valid_position = Mock(return_value=True)
    session._neutral_manual = Mock()

    if stale:
        with pytest.raises(RuntimeError, match="status or local position is stale"):
            session._require_active()
        session._neutral_manual.assert_called_once()
    else:
        session._require_active()
        session._neutral_manual.assert_not_called()


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


def test_takeoff_without_mode_keeps_legacy_arming_and_navigation() -> None:
    class Status:
        ARMING_STATE_ARMED = 2

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(arming_state=Status.ARMING_STATE_ARMED)
    session._types = {"status": Status}
    session.arm = Mock()
    session.goto = Mock(return_value={"mode": "position"})

    result = session.takeoff(1.5)

    assert result == {"mode": "position"}
    session.arm.assert_not_called()
    session.goto.assert_called_once()
    assert session.goto.call_args.args == (flight.Pose(0.0, 0.0, 1.5),)
    assert "mode" not in session.goto.call_args.kwargs


def test_takeoff_without_mode_arms_with_legacy_offboard_helper() -> None:
    class Status:
        ARMING_STATE_ARMED = 2

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(arming_state=1)
    session._types = {"status": Status}
    session.arm = Mock()
    session.goto = Mock(return_value={})

    session.takeoff(1.0)

    session.arm.assert_called_once()


def test_takeoff_explicit_mode_is_selected_before_arming_and_liftoff() -> None:
    class Status:
        ARMING_STATE_ARMED = 2
        ARMING_STATE_DISARMED = 1

    calls: list[str] = []
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(
        arming_state=Status.ARMING_STATE_DISARMED,
        pre_flight_checks_pass=True,
    )
    session._status_received_ns = 1
    session._types = {
        "status": Status,
        "command": SimpleNamespace(VEHICLE_CMD_COMPONENT_ARM_DISARM=400),
    }
    session._fresh = Mock(return_value=True)

    def set_mode(mode: str, *, timeout_seconds: float) -> None:
        calls.append(f"mode:{mode}")

    def wait_state(predicate, _deadline, _label) -> None:
        calls.append("armed")
        session._status.arming_state = Status.ARMING_STATE_ARMED
        assert predicate(session._status)

    session.set_mode = Mock(side_effect=set_mode)
    session._wait_preflight_checks = Mock(
        side_effect=lambda _deadline: calls.append("preflight")
    )
    session._command = Mock(side_effect=lambda *_args: calls.append("arm_command"))
    session._wait_state = Mock(side_effect=wait_state)
    session._manual_takeoff_until_liftoff = Mock(side_effect=lambda _deadline: calls.append("liftoff"))
    session.goto = Mock(side_effect=lambda *_args, **_kwargs: calls.append("goto") or {})

    session.takeoff(1.2, mode="position")

    assert calls == [
        "mode:position",
        "preflight",
        "arm_command",
        "armed",
        "liftoff",
        "goto",
    ]
    session.set_mode.assert_called_once()
    assert session.set_mode.call_args.args == ("position",)


def test_manual_takeoff_waits_for_fresh_preflight_after_mode_selection() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(
        pre_flight_checks_pass=False,
        failsafe=False,
        failsafe_and_user_took_over=False,
    )
    session._status_received_ns = 1
    session._fresh = Mock(return_value=True)
    session._neutral_manual = Mock()
    session._state = Mock(return_value="mode=position")
    session._spin = Mock(
        side_effect=lambda _deadline: setattr(
            session._status, "pre_flight_checks_pass", True
        )
    )

    session._wait_preflight_checks(flight.time.monotonic() + 1)

    session._spin.assert_called_once()
    session._neutral_manual.assert_not_called()


def test_manual_takeoff_preflight_wait_reports_stale_status() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(
        pre_flight_checks_pass=False,
        failsafe=False,
        failsafe_and_user_took_over=False,
    )
    session._status_received_ns = 1
    session._fresh = Mock(return_value=False)
    session._neutral_manual = Mock()
    session._state = Mock(return_value="mode=position")

    with pytest.raises(RuntimeError, match="status stopped updating"):
        session._wait_preflight_checks(flight.time.monotonic() + 1)

    session._fresh.assert_called_once_with(
        session._status_received_ns, session._status_max_age_seconds
    )
    session._neutral_manual.assert_called_once()


def test_manual_takeoff_preflight_wait_stops_on_failsafe() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._status = SimpleNamespace(
        pre_flight_checks_pass=False,
        failsafe=True,
        failsafe_and_user_took_over=False,
    )
    session._status_received_ns = 1
    session._fresh = Mock(return_value=True)
    session._neutral_manual = Mock()
    session._state = Mock(return_value="mode=position, failsafe=True")
    session._spin = Mock()

    with pytest.raises(RuntimeError, match="entered failsafe"):
        session._wait_preflight_checks(flight.time.monotonic() + 1)

    session._spin.assert_not_called()
    session._neutral_manual.assert_called_once()


def test_manual_takeoff_ramps_bounded_input_until_liftoff_is_measured() -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._position = SimpleNamespace(z=0.0)
    session._position_received_ns = 10
    session._require_active = Mock()
    session._set_manual = Mock()
    session._neutral_manual = Mock()
    altitude_updates = iter((0.0, -0.03, -0.1, -0.16))

    def next_position(_previous_ns: int, _deadline: float) -> SimpleNamespace:
        session._position_received_ns += 1
        session._position.z = next(altitude_updates)
        return session._position

    session._next_position = next_position

    climbed_m = session._manual_takeoff_until_liftoff(flight.time.monotonic() + 2)

    assert climbed_m == pytest.approx(0.16)
    throttle = [call.kwargs["throttle"] for call in session._set_manual.call_args_list]
    assert throttle == sorted(throttle)
    assert all(0 <= value <= session._manual_input_limit for value in throttle)
    session._neutral_manual.assert_not_called()


@pytest.mark.parametrize(
    "error",
    [
        TimeoutError("PX4 position wait timed out; mode=position"),
        RuntimeError("PX4 local position stopped updating; mode=position"),
        RuntimeError("PX4 left the commanded armed mode; armed=1, failsafe=False"),
        RuntimeError("PX4 left the commanded armed mode; armed=2, failsafe=True"),
    ],
    ids=("timeout", "stale_position", "disarmed", "failsafe"),
)
def test_manual_takeoff_stops_and_neutralizes_on_px4_failure(error: Exception) -> None:
    session = flight.FlightSession.__new__(flight.FlightSession)
    session._position = SimpleNamespace(z=0.0)
    session._position_received_ns = 10
    session._require_active = Mock()
    session._next_position = Mock(side_effect=error)
    session._neutral_manual = Mock()
    session._set_manual = Mock()

    with pytest.raises(type(error), match=str(error)):
        session._manual_takeoff_until_liftoff(flight.time.monotonic() + 1)

    session._neutral_manual.assert_called_once()
    session._set_manual.assert_not_called()


def test_px4_takeover_keeps_manual_stream_alive_and_neutral() -> None:
    class Status:
        ARMING_STATE_ARMED = 2
        NAVIGATION_STATE_POSCTL = 2

    class Stream:
        def __init__(self) -> None:
            self.updates = []
            self.closed = False

        def update(self, index, message) -> None:
            self.updates.append((index, message))

        def close(self) -> None:
            self.closed = True

    session = flight.FlightSession.__new__(flight.FlightSession)
    session._mode = "position"
    session._types = {"status": Status}
    session._clock = SimpleNamespace(observe=Mock())
    session._manual_stream = Stream()
    session._manual_message = Mock(return_value="neutral")
    session._status = None

    session._on_status(SimpleNamespace(
        nav_state=9,
        arming_state=Status.ARMING_STATE_ARMED,
        failsafe=False,
        failsafe_and_user_took_over=False,
    ))

    assert session._manual_stream.updates == [(0, "neutral")]
    assert session._manual_stream.closed is False


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


def test_position_path_corrects_endpoint_after_route_progress_finishes(monkeypatch) -> None:
    now = 0.0
    monkeypatch.setattr(flight, "time", SimpleNamespace(monotonic=lambda: now))
    session = flight.FlightSession.__new__(flight.FlightSession)
    current = [1.0, 0.4, 0.0]
    sample = SimpleNamespace(vx=0.0, vy=0.0, vz=0.0, heading=0.0)
    session._position_received_ns = 0
    session._mode = "position"
    session._require_active = Mock()
    session._position_tuple = lambda: tuple(current)
    session._snapshot = Mock(return_value={})
    session._neutral_manual = Mock()

    def next_position(_previous_ns, _deadline):
        nonlocal now
        now += 0.1
        session._position_received_ns += 1
        return sample

    def control(target, _yaw, speed):
        if target != (1.0, 0.0, 0.0) or speed <= 0:
            return
        current[1] -= min(current[1], speed * 0.1)

    session._next_position = next_position
    session._control = control
    result = session._navigate(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        [0.0, 0.0],
        speed_mps=1.0,
        tolerance_m=0.25,
        deadline=5.0,
    )
    assert result["endpoint_error_m"] <= 0.25


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
