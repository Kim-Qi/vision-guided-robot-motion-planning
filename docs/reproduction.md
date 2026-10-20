# Reproducing the verified AutoDL pre-grasp demonstration

Status: **instructions reconstructed from user execution and handoff source**, not tested on a fresh machine. Use ROS2 Humble + Isaac Sim 5.1, compatible GPU driver and NVIDIA FoundationPose official dependencies. This requires substantial resources and network/model access.

## 0. Paths / prerequisites

```bash
export ROBOT_PROJECT_ROOT=/root/autodl-tmp/robot_vision_motion
export ROS_DOMAIN_ID=0
# Place this repository's source directories under $ROBOT_PROJECT_ROOT
# Clone/build IsaacSim ROS workspace at:
#   $ROBOT_PROJECT_ROOT/third_party/IsaacSim_ros_5.1/humble_ws
# Install FoundationPose source and official weights at:
#   $ROBOT_PROJECT_ROOT/third_party/FoundationPose
# Install Isaac Sim 5.1 conda env at /root/autodl-tmp/conda_envs/isaacsim
# Install FoundationPose conda env at /root/autodl-tmp/conda_envs/foundationpose
```

See [environment.md](environment.md). Asset paths must be available to Isaac Sim. The exported ZIP did **not** contain complete RGB-D input or weight files; those are generated/acquired in the steps below.

## 1. Validate / correct the robot base static TF

The upstream headless launch originally published `world -> panda_link0 = (0, -0.64, 0)` while `run_franka_vision.py` loads Franka at `(0,0,0)` with no translation. That caused a **640 mm target frame error**. Either use the included corrected launch snapshot (`configs/isaac_moveit_headless.corrected.launch.py`) as a reference or patch your upstream checkout:

```bash
python3 scripts/patch_moveit_tf.py --launch-file   "$ROBOT_PROJECT_ROOT/third_party/IsaacSim_ros_5.1/humble_ws/src/moveit/isaac_moveit/launch/isaac_moveit_headless.launch.py"
```

Back up your vendor checkout. Verify robot base **rotation** and TF as well as translation for any changed assets. After changing launch file restart MoveIt2 to replace the old static TF.

## 2. Terminal A — Isaac Sim (keep running)

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/conda_envs/isaacsim
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
export OMNI_KIT_ALLOW_ROOT=1
export VK_ICD_FILENAMES=/tmp/isaac-vulkan-test/nvidia_egl_icd.json
export XDG_CACHE_HOME=/root/autodl-tmp/.cache
export XDG_RUNTIME_DIR=/tmp/xdg-runtime
export ROS_DOMAIN_ID=0
mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
python -u "$ROBOT_PROJECT_ROOT/simulation/run_franka_vision.py" > "$ROBOT_PROJECT_ROOT/vision.log" 2>&1
```

**Note:** the `VK_ICD_FILENAMES` path is from the user's specific AutoDL machine; it must exist and match your GPU. The script writes `vision_data/rgb.png`, `depth.npy`, `mask.png`, `object.obj`, `intrinsics.npy` and metadata. Confirm `VISION: DATA EXPORTED` in `vision.log`.

## 3. Terminal B — MoveIt2 (launch after A)

```bash
source /opt/ros/humble/setup.bash
source "$ROBOT_PROJECT_ROOT/third_party/IsaacSim_ros_5.1/humble_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 launch "$ROBOT_PROJECT_ROOT/third_party/IsaacSim_ros_5.1/humble_ws/src/moveit/isaac_moveit/launch/isaac_moveit_headless.launch.py"  ros2_control_hardware_type:=isaac use_sim_time:=true
```

After launch verify:

```bash
timeout 6 ros2 run tf2_ros tf2_echo world panda_link0
# Translation must be [0, 0, 0] and rotation identity for this scene.
```

## 4. Terminal C — official FoundationPose inference

```bash
export ROBOT_PROJECT_ROOT=/root/autodl-tmp/robot_vision_motion
export FOUNDATIONPOSE_PYTHON=/root/autodl-tmp/conda_envs/foundationpose/bin/python
bash "$ROBOT_PROJECT_ROOT/scripts/prepare_target.sh"
```

This script runs `run_foundationpose.py`, then `verify_world_pose.py` **with the corrected identity base frame**, then `make_pregrasp_target.py`. Expected object position near `(0.54925, -0.40001, 0.06380)` and pre-grasp target near `(0.54925,-0.40001,0.24380)` for the provided fixed cube. For **different object poses or camera placements**, update ground truth, camera extrinsics and the object's simulation placement instead of reusing fixed results. Inspect the projection overlay.

## 5. Add collision geometry and plan only

```bash
export ROBOT_PROJECT_ROOT=/root/autodl-tmp/robot_vision_motion
bash "$ROBOT_PROJECT_ROOT/scripts/plan_only.sh"
```

The planning scene contains the red cube and studio floor. `plan_vertical_pregrasp.py` must print `Execute = False`, `IK error code = 1`, `MoveIt error code = 1`. Planning returns a motion to `panda_hand` with quaternion `(1,0,0,0)` and a vertical downward local +Z tool axis. MoveIt2 uses IK joint constraints and OMPL; the path is not guaranteed to be straight-line Cartesian motion.

## 6. Explicit execution and video (optional; simulation only)

```bash
# Only after verifying scene and planning in the currently running MoveIt2 session:
source /opt/ros/humble/setup.bash
source "$ROBOT_PROJECT_ROOT/third_party/IsaacSim_ros_5.1/humble_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
bash "$ROBOT_PROJECT_ROOT/scripts/record_vertical_demo.sh"
/usr/bin/python3 "$ROBOT_PROJECT_ROOT/planning/check_vertical_tcp.py"
```

If not recording, execute explicitly with `/usr/bin/python3 "$ROBOT_PROJECT_ROOT/planning/execute_vertical_pregrasp.py" --execute` after plan validation. `check_vertical_tcp.py` now reads the goal from the actual `pregrasp_target.json` rather than a stale hard-coded coordinate.

## 7. Expected metrics and correctness checks

- Reported sample: 1.41 mm object center position error in simulation, 77 trajectory points, execution return code 1, approximate TCP 6.86 mm / 0.77°.
- TCP offset model: 58.4 mm along `panda_hand` local +Z; confirm against your gripper geometry before claiming accuracy.
- Record video only when FFmpeg completes and `ffprobe` accepts the output; avoid copying before MP4 `moov` atom is finalized.
- The fixed camera calibration and fixed object center are part of this **single-scene test**; don't treat them as general object detection/tracking.
