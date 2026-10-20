# Environment / 环境说明

Recorded environment: Ubuntu 22.04 class Linux kernel 5.15, RTX 4090 D (24 GB), NVIDIA driver 580.76.05, ROS2 Humble, Isaac Sim Python wheel **5.1.0.0**, FFmpeg **4.4.2**, FoundationPose torch **2.5.1+cu118**, torchvision **0.20.1+cu118**, PyTorch3D **0.7.9**, NVDiffRast **0.4.0**. Original environment package observations in [environment.txt](environment.txt).

## Third-party repositories

- [NVIDIA IsaacSim ROS Workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces): the user workspace was stored at `third_party/IsaacSim_ros_5.1/humble_ws`. The exported environment **does not contain the repo commit**, because git metadata was absent. Match the source to Isaac Sim 5.1 and verify MoveIt launch paths yourself.
- [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose): clone separately to `third_party/FoundationPose`; original repo commit was not recorded. Download **official refiner and scorer weights** by following its README; they are not included here. Run original project setup for its CUDA extensions, NVDiffRast and PyTorch3D.
- The original user had to fix one upstream `Utils.py` dtype mismatch (`pts=pts.to(dtype=K.dtype,device=K.device)`); this is a **local workaround** rather than a guaranteed upstream fix; validate against the version you install.
- Isaac ROS2 integration requires built workspace, compatible MoveIt2 packages and sourceable `install/setup.bash`. Follow the official NVIDIA integration instructions for the matching release.

## Original working path (default in source)

`/root/autodl-tmp/robot_vision_motion` (source root). The exported scripts also accept `ROBOT_PROJECT_ROOT`, but many upstream workspace paths in helper scripts remain tied to the original AutoDL layout. For a near-exact reconstruction, check out the repository at that original path or adapt environment variables.

**Do not install all dependencies with one `pip install -r requirements.txt`**: Isaac Sim and FoundationPose used independent Conda environments, plus ROS2 system Python. Mixing their Python dependencies will break ROS2 or CUDA extensions.
