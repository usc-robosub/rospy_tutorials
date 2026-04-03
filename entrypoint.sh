#!/bin/bash
set -e

if [ -n "${IMG_ID}" ]
then echo "IMG_ID environment variable set: ${IMG_ID}"
fi

if [ -n "${PKG_SEL}" ]
then echo "PKG_SEL environment variable set: ${PKG_SEL}"
fi

source /opt/ros/humble/setup.bash

DEFAULT_PKGS=$(ros2 pkg list | sort)

source /root/tutorial_ws/install/setup.bash

ALL_PKGS=$(ros2 pkg list | sort)

echo "=== Packages built in tutorial_ws ==="
comm -13 <(echo "$DEFAULT_PKGS") <(echo "$ALL_PKGS")
echo ""

echo "=========================================="
echo " ROS 2 Humble Tutorial Workspace Ready! "
echo "=========================================="

exec /bin/bash
