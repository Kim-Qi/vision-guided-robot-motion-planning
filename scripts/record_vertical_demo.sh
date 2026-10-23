#!/usr/bin/env bash
set -o pipefail

BASE="${ROBOT_PROJECT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
VIDEO="$BASE/videos"
SIMLOG="$BASE/vision.log"
PLAN="$BASE/planning/execute_vertical_pregrasp.py"
RUNLOG="$BASE/vertical_execution.log"

START="$VIDEO/START_RECORDING"
STOP="$VIDEO/STOP_RECORDING"
OUTPUT="$VIDEO/franka_moveit_showcase.mp4"
BACKUP="$VIDEO/first_pregrasp_showcase.mp4"
FINAL="$VIDEO/foundationpose_vertical_pregrasp.mp4"

source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${ISAAC_ROS_WS:-$BASE/third_party/IsaacSim_ros_5.1/humble_ws}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 保留第一版视频
if [ -f "$OUTPUT" ] && [ ! -f "$BACKUP" ]; then
    cp "$OUTPUT" "$BACKUP"
fi

if [ -e "$START" ] || [ -e "$STOP" ]; then
    echo "ERROR: 发现未清理的录像触发文件"
    exit 1
fi

stop_recording() {
    touch "$STOP"
}
trap stop_recording EXIT

PREV_DONE=$(grep -c "VIDEO: FFMPEG EXIT = 0" "$SIMLOG" 2>/dev/null || true)
echo "VIDEO: Requesting recording..."
touch "$START"

started=0
for i in $(seq 1 30); do
    if [ ! -e "$START" ]; then
        started=1
        break
    fi
    sleep 1
done

if [ "$started" -ne 1 ]; then
    echo "ERROR: Isaac Sim 未响应录像请求"
    exit 1
fi

echo "VIDEO: Recording trigger accepted"
sleep 3

echo "ROBOT: Executing vertical pregrasp..."
"${ROS_PYTHON:-/usr/bin/python3}" "$PLAN" --execute 2>&1 | tee "$RUNLOG"
result=${PIPESTATUS[0]}

echo "ROBOT: Exit code = $result"

# 保留结束画面
sleep 3

echo "VIDEO: Stopping recording..."
touch "$STOP"

stopped=0
for i in $(seq 1 30); do
    if [ ! -e "$STOP" ]; then
        stopped=1
        break
    fi
    sleep 1
done

if [ "$stopped" -ne 1 ]; then
    echo "ERROR: 录像停止请求未完成"
    exit 1
fi

# 等待本次录像的 FFmpeg 正常完成（不能依赖旧 MP4 的 ffprobe 成功）
for i in $(seq 1 60); do
    NOW_DONE=$(grep -c "VIDEO: FFMPEG EXIT = 0" "$SIMLOG" 2>/dev/null || true)
    if [ "$NOW_DONE" -gt "$PREV_DONE" ]; then break; fi
    sleep 1
done
if [ "$NOW_DONE" -le "$PREV_DONE" ]; then echo "ERROR: FFmpeg completion not confirmed"; exit 1; fi

# 等待 FFmpeg 真正完成 MP4 封装
echo "VIDEO: Waiting for MP4 finalization..."

ready=0

for i in $(seq 1 60); do
    if [ -s "$OUTPUT" ] &&        ffprobe -v error          -show_entries format=duration          -of csv=p=0          "$OUTPUT" >/dev/null 2>&1; then
        ready=1
        break
    fi
    sleep 1
done

if [ "$ready" -ne 1 ]; then
    echo "ERROR: MP4 封装未完成，禁止复制"
    exit 1
fi

cp "$OUTPUT" "$FINAL"

# 校验复制后的视频
ffmpeg -v error -i "$FINAL" -f null - ||
    { echo "ERROR: MP4 解码失败"; exit 1; }

trap - EXIT

echo "VIDEO SAVED: $FINAL"
echo "EXECUTION EXIT CODE: $result"
exit "$result"
