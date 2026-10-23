#!/usr/bin/env bash
set -euo pipefail
BASE="${ROBOT_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${ISAAC_ROS_WS:-$BASE/third_party/IsaacSim_ros_5.1/humble_ws}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
"${ROS_PYTHON:-/usr/bin/python3}" "$BASE/planning/add_vision_collision.py"
"${ROS_PYTHON:-/usr/bin/python3}" "$BASE/planning/add_floor_collision.py"
"${ROS_PYTHON:-/usr/bin/python3}" "$BASE/planning/plan_vertical_pregrasp.py"
