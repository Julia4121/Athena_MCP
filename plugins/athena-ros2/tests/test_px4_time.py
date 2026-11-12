"""PX4 command timestamps follow observed PX4 boot time."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy.node")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scaffold" / "sim_bringup"))

from sim_bringup import test_support  # noqa: E402


def test_px4_clock_uses_fresh_telemetry_and_rejects_stale_time(monkeypatch) -> None:
    now = 10_000_000_000
    monkeypatch.setattr(test_support.time, "monotonic_ns", lambda: now)
    clock = test_support.Px4Time(max_age_seconds=1)
    with pytest.raises(RuntimeError, match="No PX4 timestamp"):
        clock.now_us()
    clock.observe(SimpleNamespace(timestamp=50_000_000))
    now += 500_000_000
    assert clock.now_us() == 50_500_000
    now += 600_000_000
    with pytest.raises(RuntimeError, match="too old"):
        clock.now_us()
