> Historical 120 mm pre-grasp notes. For the current physical multi-object pipeline, see [INTERVIEW_CODE_GUIDE.md](INTERVIEW_CODE_GUIDE.md) and [the README](../README.md).

# Troubleshooting / 故障排查

| Symptom | Cause / check | Fix |
|---|---|---|
| IK/MoveIt success, gripper visually far from cube | Isaac Sim base world `(0,0,0)` but ROS2 static TF `(0,-0.64,0)` | Patch static transform to identity, restart MoveIt2, rerun vision-to-base transform and collision scene |
| Pose accuracy JSON says base Y `+0.23999` | Prior stale calculation from wrong TF | Use corrected `vision/verify_world_pose.py`; object base Y should be near `-0.40001` |
| Tool seems sideways despite position match | `panda_hand` frame is not gripper TCP | Use vertical tool quaternion and measured tool offset; calibrate physical TCP later |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` during ROS Humble setup | Disable nounset before sourcing ROS scripts |
| MP4 `moov atom not found` | Recording script copied file before FFmpeg finalized | Wait for new `VIDEO: FFMPEG EXIT = 0` event and use ffprobe/ffmpeg verification |
| FoundationPose dtype mismatch | Pytorch3D/CUDA extension mixed dtypes in a checked-out upstream source | Inspect upstream `Utils.py` and match tensor dtype/device; do not blindly patch unknown versions |
| `Invalid frame ID world` immediately after launch | TF transient startup | Wait for publisher; if persistent check ROS_DOMAIN_ID, /tf_static and launch process |
| Optical frame mismatch | USD camera (-Z forward, +Y up) vs OpenCV optical (+Z forward, +Y down) | Apply `diag(1,-1,-1)` rotation and compose 4x4 transforms carefully |
