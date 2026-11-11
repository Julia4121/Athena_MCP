"""Check the compact recording contract and PX4 parameter input."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from simulation.evidence import contact_topics, select_topics
from simulation.runner import _validate_px4_parameters


class EvidenceTests(unittest.TestCase):
    def test_default_topics_resolve_versioned_px4_and_exclude_battery(self) -> None:
        available = {
            "/clock",
            "/fmu/out/vehicle_status_v1",
            "/fmu/out/vehicle_local_position",
            "/fmu/out/battery_status",
        }
        selected, missing = select_topics(available, [])
        self.assertIn("/fmu/out/vehicle_status_v1", selected)
        self.assertNotIn("/fmu/out/vehicle_status", selected)
        self.assertNotIn("/fmu/out/battery_status", selected)
        self.assertIn("/fmu/in/vehicle_command", selected)
        self.assertIn("/fmu/in/telemetry_status", missing)
        requested, _ = select_topics(available, ["/fmu/out/battery_status"])
        self.assertIn("/fmu/out/battery_status", requested)

    def test_contact_topics_come_from_drone_sdf(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drone.sdf"
            path.write_text(
                '<sdf><model><link><sensor type="contact"><topic>/drone/contact</topic></sensor>'
                '<sensor type="imu"><topic>/drone/imu</topic></sensor></link></model></sdf>',
                encoding="utf-8",
            )
            self.assertEqual(contact_topics(path), ["/drone/contact"])

    def test_px4_settings_accept_finite_numbers_and_protect_clock_source(self) -> None:
        self.assertEqual(_validate_px4_parameters({"NAV_DLL_ACT": 1, "COM_DL_LOSS_T": 5.0}),
                         {"NAV_DLL_ACT": 1, "COM_DL_LOSS_T": 5.0})
        for invalid in ({"UXRCE_DDS_SYNCT": 1}, {"NAV_DLL_ACT": True},
                        {"NAV_DLL_ACT": float("nan")}, {"BAD-NAME": 1}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                _validate_px4_parameters(invalid)


if __name__ == "__main__":
    unittest.main()
