# Athena ROS 2 for Codex

This repository packages one ROS 2 development skill and a minimal local MCP server as a Codex plugin. The [skill](plugins/athena-ros2/skills/ros2-development/SKILL.md) contains the [Henki ROS 2 Best Practices](https://github.com/henki-robotics/henki_ros2_best_practices) directly, with attribution and the upstream license. The server connects over stdio and exposes no tools yet.

## Install

You need access to this private GitHub repository, [Codex](https://developers.openai.com/codex), Python 3.11 or newer, and [`uv`](https://docs.astral.sh/uv/) on your `PATH`. An installed ROS distribution is not required until ROS command tools are added.

```bash
codex plugin marketplace add ssh://git@github.com/Julia4121/Athena_MCP.git
codex plugin add athena-ros2@athena-mcp
```

Restart Codex after installation. The `ros2-development` skill is available automatically for ROS 2 work and can also be selected explicitly. The `athena-ros2` MCP server should appear in Codex's MCP server list with no tools.

To update an existing installation, run `codex plugin marketplace upgrade athena-mcp` followed by `codex plugin add athena-ros2@athena-mcp`, then restart Codex.

## Local development

From this repository, install the dependencies and check the lock file with:

```bash
uv sync --project plugins/athena-ros2 --locked
```

The plugin is defined under `plugins/athena-ros2`; `.agents/plugins/marketplace.json` makes it installable from this repository. Future command tools belong in `server.py`, while the ROS development guidance belongs in the skill.
