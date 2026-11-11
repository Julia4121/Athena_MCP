"""Local MCP server for Athena ROS 2 tools."""

from mcp.server import MCPServer


mcp = MCPServer("athena-ros2")


if __name__ == "__main__":
    mcp.run()
