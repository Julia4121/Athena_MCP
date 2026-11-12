"""Contact intervals preserve separate takeoff and landing evidence."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scaffold" / "sim_bringup"))

from sim_bringup import contact_monitor  # noqa: E402


def test_contact_pair_starts_new_interval_after_quiet_gap(monkeypatch) -> None:
    monitor = contact_monitor.ContactMonitor.__new__(contact_monitor.ContactMonitor)
    monitor.summaries = []
    monitor.latest = {}
    times = iter((1_000_000_000, 1_200_000_000, 3_000_000_000))
    monkeypatch.setattr(contact_monitor.time, "time_ns", lambda: next(times))
    contact = SimpleNamespace(
        collision1=SimpleNamespace(name="room::floor_collision"),
        collision2=SimpleNamespace(name="drone::base_link_collision_3"),
    )
    message = SimpleNamespace(contacts=[contact])

    for _ in range(3):
        monitor.observe("/drone/contact", message)

    assert len(monitor.summaries) == 2
    assert monitor.summaries[0]["count"] == 2
    assert monitor.summaries[0]["first_wall_time_ns"] == 1_000_000_000
    assert monitor.summaries[0]["last_wall_time_ns"] == 1_200_000_000
    assert monitor.summaries[1]["count"] == 1
    assert monitor.summaries[1]["first_wall_time_ns"] == 3_000_000_000
