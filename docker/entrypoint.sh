#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "=========================================="
echo " ROS 2 Humble Tutorial Workspace Ready! "
echo "=========================================="

exec "$@"