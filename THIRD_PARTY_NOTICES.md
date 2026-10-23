# Third-party notices

This project integrates upstream software; it does not claim authorship of FoundationPose, its pretrained models, robot assets, PhysX, OMPL, ROS2 or MoveIt2. Dependencies/weights are acquired separately. Package versions and source SHA256 observations are in [docs/dependencies.json](docs/dependencies.json). Unknown original Git revisions are explicitly marked unknown.

| Dependency | Observed version/source | License reference |
|---|---|---|
| Isaac Sim | 5.1.0.0 wheel | Installed metadata declares NVIDIA proprietary software; [official product/documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html) and applicable NVIDIA terms govern assets/runtime |
| IsaacSim ROS workspaces | 5.1 source archive, exact revision unavailable | [Apache-2.0 upstream](https://github.com/isaac-sim/IsaacSim-ros_workspaces/blob/main/LICENSE); full text in `licenses/Apache-2.0.txt` |
| FoundationPose | Source hash recorded; original Git revision unavailable | [NVIDIA custom source license](https://github.com/NVlabs/FoundationPose/blob/main/LICENSE), including its research/evaluation use limitation; full original local license in `licenses/FoundationPose-LICENSE.txt` |
| nvdiffrast | 0.4.0 | [Nvidia Source Code License (1-Way Commercial)](https://github.com/NVlabs/nvdiffrast/blob/v0.4.0/LICENSE.txt); review its stated use limitation |
| PyTorch / torchvision | 2.5.1+cu118 / 0.20.1+cu118 | [PyTorch license](https://github.com/pytorch/pytorch/blob/v2.5.1/LICENSE) and [torchvision license](https://github.com/pytorch/vision/blob/v0.20.1/LICENSE) |
| PyTorch3D | 0.7.9 | [BSD license](https://github.com/facebookresearch/pytorch3d/blob/v0.7.9/LICENSE) |
| ROS2 rclpy / ros2_controllers | 3.3.22 / 2.54.1 | Apache-2.0 package declarations; upstream components retain notices |
| MoveIt2 / OMPL | move_group 2.5.10 / OMPL 1.7.0 | BSD package declarations; see respective upstream licenses |
| OpenCV / trimesh / Pillow / NumPy | See version observations | Apache-2.0 / MIT / MIT-CMU / BSD-family plus bundled component notices |

`configs/isaac_moveit.launch.py` and `configs/isaac_moveit_headless.launch.py` derive from NVIDIA's Apache-2.0 launch files. Original copyright/SPDX notices are retained; project modifications use measured scene TF and relocate the project root. The compatibility launch delegates to that implementation.

`patches/foundationpose-projection-dtype.patch` records the actual two-line local Utils.py workaround: match projected points to K's tensor dtype/device. It is not a new pose-estimation model or an upstream-approved fix. Apply only to a compatible upstream source after inspecting the patch. The retained FoundationPose license applies to its code/patch context. No third-party model weights or implementation tree is vendored.

The repository previously had no blanket first-party license. This update does not invent one or relicense upstream material. Review component terms before redistribution or other uses.
