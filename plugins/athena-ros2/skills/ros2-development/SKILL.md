---
name: ros2-development
description: Develop or review ROS 2 nodes, packages, interfaces, launch files, configuration, and tests. Use for ROS 2 work, not ROS 1-only projects.
---

# ROS 2 development

Read the [Henki ROS 2 Best Practices](references/henki_ros2_best_practices.md) before substantial ROS 2 changes or reviews. Apply the relevant guidance while respecting the user's requirements and established project conventions.

- Identify the project's ROS distribution, language, build type, and existing patterns before choosing APIs or package structure. Check documentation for that distribution when behavior is version specific.
- Keep node communication separate from core application logic. Choose topics, services, actions, and message types for their intended semantics.
- When messages carry time or spatial data, check timestamps, clock source, coordinate frames, units, and QoS compatibility at each connection.
- Keep package dependencies, launch and parameter files, tests, and package documentation in step with code changes. Prefer deterministic tests for core logic and integration tests for ROS communication.
- Run the relevant build and tests with the project's available commands. Use failures as diagnostics, fix issues within the task's scope, and report any remaining failures.

The bundled Henki guide is copied from [henki-robotics/henki_ros2_best_practices](https://github.com/henki-robotics/henki_ros2_best_practices) at commit `2db5b52da57670a91a99a8b2cebd309141f4fc6d` under the [Apache 2.0 license](references/HENKI_LICENSE).
