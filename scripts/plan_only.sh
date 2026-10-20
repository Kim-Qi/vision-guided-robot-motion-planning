#!/usr/bin/env bash
set -euo pipefail
BASE="${ROBOT_PROJECT_ROOT:-/root/autodl-tmp/robot_vision_motion}"
set +u
source /opt/ros/humble/setup.bash
source "$BASE/third_party/IsaacSim_ros_5.1/humble_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
/usr/bin/python3 "$BASE/planning/add_vision_collision.py"
/usr/bin/python3 "$BASE/planning/add_floor_collision.py"
/usr/bin/python3 "$BASE/planning/plan_vertical_pregrasp.py"
