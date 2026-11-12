"""Launch installation errors should be found before simulator startup."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simulation.runner import ApplicationLaunch, _validate_application_launch


class LaunchPreflightTests(unittest.TestCase):
    def test_installed_launch_file_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/demo"
            launch = prefix / "share/demo/launch/demo.launch.py"
            launch.parent.mkdir(parents=True)
            launch.write_text("# example\n", encoding="utf-8")
            with patch(
                "simulation.runner.subprocess.run",
                return_value=subprocess.CompletedProcess([], 0, stdout=str(prefix), stderr=""),
            ) as command:
                _validate_application_launch(
                    ApplicationLaunch(package="demo", file="demo.launch.py"), workspace, {}
                )
            self.assertEqual(command.call_args.args[0], ["ros2", "pkg", "prefix", "demo"])

    def test_missing_or_prefixed_launch_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/demo"
            with patch(
                "simulation.runner.subprocess.run",
                return_value=subprocess.CompletedProcess([], 0, stdout=str(prefix), stderr=""),
            ):
                with self.assertRaisesRegex(ValueError, "not installed"):
                    _validate_application_launch(
                        ApplicationLaunch(package="demo", file="demo.launch.py"), workspace, {}
                    )
            with self.assertRaisesRegex(ValueError, "filename"):
                _validate_application_launch(
                    ApplicationLaunch(package="demo", file="launch/demo.launch.py"), workspace, {}
                )

    def test_package_from_another_underlay_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            underlay = root / "underlay"
            launch = underlay / "share/demo/launch/demo.launch.py"
            launch.parent.mkdir(parents=True)
            launch.write_text("# example\n", encoding="utf-8")
            with patch(
                "simulation.runner.subprocess.run",
                return_value=subprocess.CompletedProcess([], 0, stdout=str(underlay), stderr=""),
            ):
                with self.assertRaisesRegex(ValueError, "this workspace"):
                    _validate_application_launch(
                        ApplicationLaunch(package="demo", file="demo.launch.py"), root / "workspace", {}
                    )


if __name__ == "__main__":
    unittest.main()
