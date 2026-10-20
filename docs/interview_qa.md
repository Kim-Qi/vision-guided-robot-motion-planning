# Interview notes / 面试知识点

**Q1: 项目要解决什么问题？** A: 单帧 RGB-D 数据估计已知 CAD 物体的 6D 位姿，将其转换到机器人基座坐标系，并在有障碍物的 MoveIt2 场景中规划到预抓取位置，由 ROS2 驱动 Isaac Sim Panda 执行。

**Q2: FoundationPose 需要什么输入？** A: RGB、深度、内参、物体 mask 与 CAD 网格；官方 scorer/refiner 权重。这里使用官方 register 推理，5 次 refine iteration。

**Q3: 为什么用 6D pose 而不是 2D bounding box？** A: 机器人需要三维位置与姿态；深度、相机标定和模型几何可提供 3D 约束。当前立方体关于旋转对称，因此规划仅使用稳健的中心位置并固定预抓取方向。

**Q4: 坐标变换如何写？** A: `T_world_object = T_world_optical @ T_optical_object`；`T_base_object = inv(T_world_base) @ T_world_object`。每个 T 是 4x4 齐次矩阵，明确左、右 frame。

**Q5: 为什么出现 640 mm 偏差？** A: ROS2 world-to-panda_link0 静态 TF 为 Y=-0.64 m，但 Isaac Sim USD Panda 根节点实际在世界原点。视觉定位和 MoveIt2 各自能成功，系统跨模块却不对齐。

**Q6: 你如何定位这个问题？** A: 首先验证 Isaac 物体地面真值与 FoundationPose 坐标，再比较 TF 末端执行误差和录像，最后读取 USD `/Franka/panda_link0` 运行时世界位姿并与 `tf2_echo world panda_link0` 比较，修复 launch + 重生成目标。

**Q7: MoveIt2 两阶段做什么？** A: `/compute_ik` 求满足 `panda_hand` 目标的关节解；`/move_action` 根据关节目标和 Planning Scene 规划并显式执行。可先 `plan_only` 检验而不发执行指令。

**Q8: 为什么加 CollisionObject？** A: 将目标立方体和地面写入规划场景，让规划器考虑几何障碍；仿真可视物体本身不会自动进入 MoveIt2 场景。

**Q9: 77 个轨迹点是什么？** A: 单次规划输出路径离散点数，不是轨迹精度的统计指标；不同起点和采样可使数量变化。

**Q10: TCP 是什么，精度如何计算？** A: 从 `panda_hand` TF 出发沿局部 +Z 平移 0.0584 m 作为临时工具参考点，计算到视觉预抓取目标的三维距离和向下轴夹角。报告 6.86 mm/0.77°，但不是实际手指夹持中心的标定精度。

**Q11: 为什么不是闭环抓取？** A: 一次 RGB-D 快照驱动一次规划执行，运动时没有连续 FoundationPose 更新及重新规划，也没有抓取碰撞闭合、夹持力和物体搬运验证。

**Q12: 如何进一步升级？** A: 多场景重定位实验、自动化评估统计、在线跟踪/重规划、夹爪控制与真实 TCP 标定；先保证现有单帧链路可复现。
