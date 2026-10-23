> Historical 120 mm pre-grasp notes. For the current physical multi-object pipeline, see [INTERVIEW_CODE_GUIDE.md](INTERVIEW_CODE_GUIDE.md) and [the README](../README.md).

# Resume project phrasing (supported by source experiment)

**基于 Isaac Sim、FoundationPose 与 ROS2/MoveIt2 的视觉引导机械臂运动规划系统**

搭建 Isaac Sim 5.1 Franka Panda RGB-D 仿真环境，调用官方 FoundationPose 进行基于 RGB-D、目标 mask 和 CAD 模型的 6D 位姿估计；实现 OpenCV 光学坐标到仿真世界再到机器人基座的变换及 MoveIt2 IK/碰撞场景/轨迹规划；经 ROS2 控制机械臂完成单场景预抓取运动。实测目标中心位置误差 1.41 mm，临时 TCP 位置误差 6.86 mm，末端向下方向误差 0.77°。通过比较 USD 基座真实位姿与 ROS2 TF，定位并修复 0.64 m 静态坐标偏差。

**Limitations:** 不描述为在线实时跟踪、真实抓取或跨场景成功率已验证。
