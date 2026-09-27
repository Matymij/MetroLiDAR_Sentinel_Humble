#!/usr/bin/env bash
# ros2 wrapper: убивает демон перед каждым вызовом, чтобы не ловить !rclpy.ok()
pkill -9 -f ros2-daemon 2>/dev/null || true
pkill -9 -f "from ros2cli.daemon" 2>/dev/null || true
rm -rf /root/.ros/ros2cli* 2>/dev/null || true

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
exec ros2 "$@"