---
name: ros2-development
description: Develop or review ROS 2 nodes, packages, interfaces, launch files, configuration, and tests. Use for ROS 2 work, not ROS 1-only projects.
---

# ROS 2 development

Follow the task's required ROS interfaces and the existing workspace conventions. Preserve specified topic, service, action, message, parameter, node, and executable names; keep the current language and package layout unless the task requires a change.

Inspect relevant packages and source the appropriate ROS environment when needed. At modified communication boundaries, check timestamps and clock source, coordinate frames, units, and QoS compatibility. Keep `package.xml`, build files, and install rules aligned with dependencies actually used.

Make the smallest coherent change. Run a targeted build and focused checks for changed behavior; add tests or documentation where they clarify or verify that behavior. Use `rosdep` or package installation only when a required dependency is missing or unresolved, not as a routine pre-build step.

For new package or API design decisions, consult the [Henki ROS 2 best practices](references/henki-best-practices.md). Treat its preferences as guidance beneath explicit task requirements and existing project conventions.
