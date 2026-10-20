#!/usr/bin/env bash
set -euo pipefail
BASE="${ROBOT_PROJECT_ROOT:-/root/autodl-tmp/robot_vision_motion}"
FP_PY="${FOUNDATIONPOSE_PYTHON:-/root/autodl-tmp/conda_envs/foundationpose/bin/python}"
cd "$BASE/third_party/FoundationPose"
PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}" "$FP_PY" "$BASE/vision/run_foundationpose.py"
"$FP_PY" "$BASE/vision/verify_world_pose.py"
"$FP_PY" "$BASE/vision/make_pregrasp_target.py"
