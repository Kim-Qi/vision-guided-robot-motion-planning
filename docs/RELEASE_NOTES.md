# v1.0.0-pick-place candidate

Adds known-color selection among three 40 mm cubes, official FoundationPose 6D estimation, measured scene transforms and 103.4 mm grasp TCP, MoveIt/ROS2 gripper control, physical pick/place stages and independent task acceptance. Retains the 120 mm pre-grasp baseline.

Accepted runtime examples (vision/place XY errors): red 0.49/1.21 mm; blue 0.11/1.22 mm; green 0.85/3.26 mm. These are three successful examples, not a statistical success rate. See retained slip/TCP-tracking failures and workspace limitations.

Full-resolution H.264 videos and raw numeric evidence are separate release assets; verify SHA256SUMS. Integration is simulation-only, known colors/cube geometry, one chosen object per reset. Third-party implementations/weights and environments are not redistributed. Publication-path adaptation is CPU checked; the frozen original runtime supplies the physical demonstrations.
