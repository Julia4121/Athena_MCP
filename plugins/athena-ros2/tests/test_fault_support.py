"""Fault helpers work with PX4-shaped messages without a live ROS process."""

from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def support(monkeypatch):
    qos = SimpleNamespace(BEST_EFFORT=1, TRANSIENT_LOCAL=1)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rclpy.clock", SimpleNamespace(Clock=object, ClockType=object))
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        DurabilityPolicy=qos, ReliabilityPolicy=qos, QoSProfile=object,
    ))
    source = Path(__file__).resolve().parents[1] / "scaffold/sim_bringup/sim_bringup/test_support.py"
    spec = importlib.util.spec_from_file_location("fault_test_support", source)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def test_battery_feed_starts_at_requested_level_and_updates(support, monkeypatch) -> None:
    sent = []
    mav = SimpleNamespace(
        srcSystem=0, srcComponent=0,
        battery_status_send=lambda *args: sent.append(args),
    )
    peer = SimpleNamespace(
        mav=mav,
        recv_match=lambda **_kwargs: SimpleNamespace(get_srcSystem=lambda: 1),
        close=lambda: None,
    )
    constants = SimpleNamespace(MAV_BATTERY_FUNCTION_ALL=0, MAV_BATTERY_TYPE_LIPO=3)
    monkeypatch.setitem(sys.modules, "pymavlink", SimpleNamespace(
        mavutil=SimpleNamespace(mavlink=constants, mavlink_connection=lambda *_args: peer),
    ))
    monkeypatch.setitem(sys.modules, "px4_msgs", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "px4_msgs.msg", SimpleNamespace(BatteryStatus=object))
    monkeypatch.setattr(support, "resolve_px4_topic", lambda *_args: "/fmu/out/battery_status")
    current = {"remaining": 0.35}

    def observed(_node, _type, _topic, predicate, _timeout):
        message = SimpleNamespace(connected=True, remaining=current["remaining"])
        assert predicate(message)
        return message

    monkeypatch.setattr(support, "wait_for_message", observed)
    with support.BatteryFeed(object(), 35, rate_hz=100) as feed:
        assert feed.first_observed.remaining == 0.35
        assert mav.srcSystem == 1 and mav.srcComponent == 190
        assert sent[0][-1] == 35
        assert sent[0][4] == [4000] * 4 + [65535] * 6
        feed.set_percent(25)
        current["remaining"] = 0.25
        assert feed.wait_for_percent(25).remaining == 0.25
        deadline = time.monotonic() + 1
        while not any(item[-1] == 25 for item in sent) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert any(item[-1] == 25 for item in sent)
    with pytest.raises(ValueError, match="integer"):
        feed.set_percent(25.5)


def test_step_event_records_observed_fault_and_integer_state(support, monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ATHENA_SIMULATION_RUN_DIR", str(tmp_path))
    monkeypatch.setattr(support.time, "time_ns", lambda: 123456789)
    event = support.write_step_completed(
        "gcs_loss", SimpleNamespace(gcs_connection_lost=True), "gcs_connection_lost",
        SimpleNamespace(nav_state=18),
    )
    assert event["wall_time_ns"] == 123456789
    assert event["result"] == {
        "failsafe_flag": "gcs_connection_lost", "failsafe_active": True, "nav_state": 18,
    }
    assert json.loads((tmp_path / "scenario_events.jsonl").read_text()) == event
    with pytest.raises(TypeError, match="observed boolean"):
        support.write_step_completed("bad", SimpleNamespace(), "gcs_connection_lost",
                                     SimpleNamespace(nav_state="LAND"))
