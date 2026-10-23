# Environment and external dependencies

The accepted runtime used Linux/RTX 4090 D 24 GB, Isaac Sim 5.1.0.0, ROS2 Humble and MoveIt2. Exact package observations, source hashes and license references are in [dependencies.json](dependencies.json) and [third-party notices](../THIRD_PARTY_NOTICES.md). Original FoundationPose and the active 5.1 ROS-workspace source archive have no recorded Git revision. A different dormant workspace's commit is deliberately not presented as the active version.

## Provision separately

1. Install [Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/index.html) in a Python 3.11 environment following NVIDIA's supported requirements. Driver/container/display setup depends on the host; no host-specific Vulkan or proxy configuration is distributed.
2. Install ROS2 Humble and compatible MoveIt2/OMPL/ros2_control packages using [ROS2 Humble instructions](https://docs.ros.org/en/humble/Installation.html) and [Isaac ROS workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces). Build the matching Humble workspace so `install/setup.bash` exists and `moveit_resources_panda_*`, `topic_based_ros2_control`, controllers and `isaac_moveit` resources resolve. The package model/controller configs are reused, not vendored.
3. Acquire [FoundationPose](https://github.com/NVlabs/FoundationPose) and official Refiner/Scorer weights through its instructions; build its CUDA dependencies in a separate environment. Recorded versions include torch 2.5.1+cu118, PyTorch3D 0.7.9 and nvdiffrast 0.4.0. The original source fingerprints and [dtype patch](../patches/foundationpose-projection-dtype.patch) are supplied for comparison; inspect compatibility before applying it with `git apply --check`. No model weights or FoundationPose source tree is included here.
4. Install FFmpeg/ffprobe and provide their executables on PATH. The recorded encoder uses libx264, 1280×720, yuv420p, 10 fps.
5. Edit a local `.env` copied from `.env.example`; do not commit it. Point `CONDA_EXE`, `ISAACSIM_ENV`, `FOUNDATIONPOSE_ENV` at existing installations. Optional `FOUNDATIONPOSE_ROOT` and `ISAAC_ROS_WS` support vendor repositories outside this checkout. `ISAAC_ROS_BRIDGE_LIB` is auto-discovered under the Isaac prefix unless overridden. ROS setup/system Python are configurable via `ROS_SETUP` and `ROS_PYTHON`.

Do not combine Isaac, FoundationPose CUDA and ROS system-Python environments into one requirements installation. `requirements-cpu.txt` covers only small CPU tests, not the simulator/model stack. This update installs nothing into the accepted runtime environment.

## Validation boundary

Accepted physical demonstrations were recorded in the original runtime. The publication tree keeps 13 core files byte-identical, with a path-only orchestration adapter and portable fixtures. CPU tests and static dependency/launch checks are recorded in [PUBLICATION_VALIDATION.md](PUBLICATION_VALIDATION.md); no fresh-machine install or new GPU end-to-end run of this publication tree is claimed.
