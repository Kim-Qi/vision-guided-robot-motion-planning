#!/usr/bin/env bash
set -euo pipefail
BASE="${ROBOT_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
FP_PY="${FOUNDATIONPOSE_PYTHON:-${FOUNDATIONPOSE_ENV:-$BASE/.envs/foundationpose}/bin/python}"
cd "${FOUNDATIONPOSE_ROOT:-$BASE/third_party/FoundationPose}"
"$FP_PY" "$BASE/vision_data/prepare_object.py"
PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}" "$FP_PY" "$BASE/vision/run_foundationpose.py"
"$FP_PY" "$BASE/vision/verify_world_pose.py"
"$FP_PY" "$BASE/vision/make_pregrasp_target.py"
