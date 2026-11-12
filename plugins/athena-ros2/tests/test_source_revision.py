"""Simulation results identify the exact source revision they exercised."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simulation.runner import _require_application_in_source, _source_revision, ros_runtime_environment


class SourceRevisionTests(unittest.TestCase):
    def test_ros_commands_do_not_inherit_plugin_venv(self) -> None:
        with patch.dict("os.environ", {
            "VIRTUAL_ENV": "/home/ws/.athena-mcp-venv",
            "UV_PROJECT_ENVIRONMENT": "/home/ws/.athena-mcp-venv",
            "PATH": "/home/ws/.athena-mcp-venv/bin:/usr/bin",
        }, clear=True):
            environment = ros_runtime_environment()
        self.assertEqual(environment["PATH"], "/usr/bin")
        self.assertNotIn("VIRTUAL_ENV", environment)

    def test_records_git_commit_and_worktree_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            source = workspace / "src/feature"
            source.mkdir(parents=True)
            commands = (
                ["git", "init", "-q", "-b", "main", str(source)],
                ["git", "-C", str(source), "config", "user.name", "Test"],
                ["git", "-C", str(source), "config", "user.email", "test@example.com"],
            )
            for command in commands:
                subprocess.run(command, check=True)
            package = source / "package.xml"
            package.write_text("<package><name>feature</name></package>\n")
            subprocess.run(["git", "-C", str(source), "add", "package.xml"], check=True)
            subprocess.run(["git", "-C", str(source), "commit", "-qm", "initial"], check=True)
            clean = _source_revision(workspace, str(source))
            self.assertEqual(clean["repository_path"], str(source))
            self.assertEqual(len(clean["sha"]), 40)
            self.assertFalse(clean["dirty"])
            _require_application_in_source(str(source), "feature")
            with self.assertRaisesRegex(ValueError, "not in"):
                _require_application_in_source(str(source), "unrelated_package")
            package.write_text("<package><name>feature_updated</name></package>\n")
            self.assertTrue(_source_revision(workspace, str(source))["dirty"])


if __name__ == "__main__":
    unittest.main()
