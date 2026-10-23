# 面试代码索引：Multi-Object Vision-Guided Pick and Place

本索引根据当前发布目录源码生成，函数名与行号经 AST 核对。行号对应此提交的文件；部署入口与已验收运行版的差异仅涉及环境路径。13个核心文件保持字节一致，见 [核心哈希](ACCEPTED_CORE_SHA256.json)。原始成功与失败见 [实验记录](../experiments/README.md)。

## 1. 项目架构与数据流

- [planning/run_pick_place.py:45](../planning/run_pick_place.py#L45) — `main`
- [runtime_paths.py:26](../runtime_paths.py#L26) — `RuntimePaths.from_environment`
- [planning/pick_place_controller.py:300](../planning/pick_place_controller.py#L300) — `PickPlace.run`

**输入输出：** 输入：目标 ID、基座系放置 XY、外部环境配置。输出：独立实验目录、状态机结果、物理验收、日志和 MP4。编排器依次等待落稳采集、启动 MoveIt、推理和控制，全部有界超时。

**算法/公式：** 单帧离线文件链：RGB-D/标定 → 三个视觉位姿 → 一个目标任务 → 规划/控制 → 真实刚体遥测验收。状态文件原子替换，批次 ID 必须一致。

**面试问题：** 为什么不是实时视觉伺服？

**参考回答：** 视觉只在落稳后采集、估计一次；执行中读到的 SDK 遥测用于反馈与物理验收，不是每帧视觉重新估计并修正轨迹。

## 2. RGB-D 和相机内参

- [simulation/multi_object_runtime.py:82](../simulation/multi_object_runtime.py#L82) — `MultiObjectRuntime.capture`
- [simulation/multi_object_runtime.py:138](../simulation/multi_object_runtime.py#L138) — `MultiObjectRuntime.update`
- [task_io.py:33](../task_io.py#L33) — `validate_capture`

**输入输出：** 输入：已落稳的实际刚体状态和两台相机对象。输出：rgb.png、米制 depth.npy、intrinsics.npy、采集时视觉光学外参/完整基座矩阵和元数据。录像相机不充当感知相机。

**算法/公式：** u=fx·X/Z+cx，v=fy·Y/Z+cy；深度类型为 distance_to_image_plane。K 从视觉相机 SDK 读取；光学坐标 X 右、Y 下、Z 前，四元数 wxyz。

**面试问题：** 如何避免录像相机外参与视觉数据混用？

**参考回答：** 采集时同时写视觉元数据和 camera_extrinsics.json；validate_capture 比较批次、frame 名和两份 SE(3)，启动 MoveIt 前不一致即失败。此前确实发现旧辅助文件记录录像相机，已修复。

## 3. 多目标颜色分割

- [vision/select_targets.py:19](../vision/select_targets.py#L19) — `color_mask`
- [vision/select_targets.py:36](../vision/select_targets.py#L36) — `main`

**输入输出：** 输入：uint8 RGB 和 red/green/blue。输出：单个 bool Mask；每个目标分别生成 Mask 与按元数据尺寸构建的 CAD。

**算法/公式：** 主通道 >55 且 >其他通道的1.4倍；取最大连通域，面积至少100像素；第二域超过最大域50%则拒绝同色歧义。

**面试问题：** 为什么能指定红色，却不能说支持开放词汇检测？

**参考回答：** ID 与颜色是显式已知配置，算法是颜色阈值和连通域；没有语言编码器、检测模型或自然语言泛化能力。两个相近面积的同色目标会拒绝而不是可靠区分。

## 4. FoundationPose 6D 位姿估计

- [vision/select_targets.py:36](../vision/select_targets.py#L36) — `main`

**输入输出：** 输入：RGB、Depth、K、单目标 Mask、同尺寸 CAD。输出：T_camera_object 及转换后的世界/基座 4×4 位姿、投影图和每对象耗时。

**算法/公式：** 共享官方 ScorePredictor/PoseRefinePredictor；调用 FoundationPose.register(iteration=5)。候选生成、refine、score 是第三方模型实现，未重新训练。三个物体都估计以建立碰撞场景，只有指定 ID 被抓取。

**面试问题：** 仿真真值是否参与了视觉估计？

**参考回答：** 没有作为 register 输入或视觉定位结果。真值用来计算误差与任务级检查；运动目标取检测结果。释放后恢复规划碰撞盒会读实际刚体位姿，这是仿真场景更新，需与视觉定位区分。

## 5. SE(3) 和坐标变换

- [scene_frames.py:15](../scene_frames.py#L15) — `rigid_transform`
- [scene_frames.py:27](../scene_frames.py#L27) — `camera_transform`
- [scene_frames.py:45](../scene_frames.py#L45) — `load_scene_frames`
- [vision/verify_world_pose.py:16](../vision/verify_world_pose.py#L16) — `compute_object_poses`

**输入输出：** 输入：相机光学系物体位姿、实际 T_world_camera、实际 T_world_base。输出：T_world_object、T_base_object。检查齐次末行、正交性、det(R)=1 和四元数规范。

**算法/公式：** T_A_B 把 B 列向量映到 A；T_w_o=T_w_c·T_c_o；T_b_o=T_w_b⁻¹·T_w_o。USD 行向量矩阵导出时转置。缺失实测基座矩阵直接报错，不回退历史偏移。

**面试问题：** 为什么不能只减去基座平移？

**参考回答：** 完整位姿可能含旋转，平移相减只在旋转为单位阵时成立。CPU 测试用非单位旋转验证完整逆变换；当前物理场景仍只验证了水平支撑面、基座Z与世界Z对齐的配置。

## 6. 抓取 TCP 与姿态补偿

- [planning/pick_place_controller.py:217](../planning/pick_place_controller.py#L217) — `PickPlace.hand_target`
- [simulation/multi_object_runtime.py:55](../simulation/multi_object_runtime.py#L55) — `MultiObjectRuntime.state`

**输入输出：** 输入：期望 TCP 基座位置、实测 hand_to_tcp 和固定向下夹持姿态。输出：MoveIt 目标 panda_hand Pose。

**算法/公式：** 实测 USD panda_hand/tool_center 局部 Z=0.1034 m；R_hand=diag(1,-1,-1)；p_hand=p_tcp−R_hand·t_hand_tcp。对实测偏移与配置值进行一致性检查。

**面试问题：** 为什么不继续用58.4 mm？

**参考回答：** 58.4 mm是历史临时TCP/另一局部几何参考；物理夹持采用USD实际tool_center的103.4 mm。偏移必须随手的旋转变换，不能简单总在世界Z加一个常数。

## 7. IK 和 MoveIt2 运动规划

- [planning/pick_place_controller.py:225](../planning/pick_place_controller.py#L225) — `PickPlace.move_tcp`
- [planning/pick_place_controller.py:122](../planning/pick_place_controller.py#L122) — `PickPlace.action`
- [planning/pick_place.launch.py:13](../planning/pick_place.launch.py#L13) — `generate_launch_description`

**输入输出：** 输入：期望 TCP、当前关节反馈、碰撞场景。输出：经 IK/OMPL 或 CartesianPath 得到并执行的轨迹；异常、返回码或 TCP 超差导致失败。

**算法/公式：** 预抓取/搬运：GetPositionIK → 7关节约束 → MoveGroup；接近/抬升/下降/撤离：GetCartesianPath(max_step=0.005, avoid_collisions=True) → ExecuteTrajectory。fraction≥0.999，实际 TCP误差≤12 mm。

**面试问题：** Action成功为什么仍可能失败？

**参考回答：** 控制器/MoveIt报告完成不保证实际末端或物体到位。绿色012在搬运Action后实际TCP误差14.78 mm，触发12 mm检查而退出，没有跳过检查继续放置。

## 8. ROS2 通信及夹爪控制

- [planning/pick_place_controller.py:285](../planning/pick_place_controller.py#L285) — `PickPlace.grip`
- [planning/pick_place_controller.py:80](../planning/pick_place_controller.py#L80) — `PickPlace.on_joints`
- [planning/pick_place.launch.py:13](../planning/pick_place.launch.py#L13) — `generate_launch_description`

**输入输出：** 输入：每指开合位置目标和ROS关节反馈。输出：GripperCommand结果、实际双指位置检查；ros2_control 和 Bridge 下发模拟关节命令。

**算法/公式：** 手臂话题 isaac_joint_commands 与夹爪 isaac_gripper_commands 分离；只驱动 panda_finger_joint1，joint2保持USD mimic。名义每指0..40 mm，双指最大开口约80 mm。配置见 configs/pick_place.urdf.xacro 与 pick_place_controllers.yaml。

**面试问题：** 为什么要拆开两个命令话题？

**参考回答：** 实测同一个按tick接收的JointState命令流中，手臂包会覆盖夹爪包，导致ROS已发闭爪但SDK目标仍张开。独立订阅/控制节点修复覆盖；Action stall只表示接触/停止候选，仍需实际指位和物理试抬验收。

## 9. 物理抓取与碰撞管理

- [simulation/pick_place_physics.py:52](../simulation/pick_place_physics.py#L52) — `configure_physics`
- [simulation/pick_place_physics.py:77](../simulation/pick_place_physics.py#L77) — `StabilityMonitor.update`
- [planning/pick_place_controller.py:166](../planning/pick_place_controller.py#L166) — `PickPlace.contacts`
- [planning/pick_place_controller.py:182](../planning/pick_place_controller.py#L182) — `PickPlace.attach`
- [planning/pick_place_controller.py:206](../planning/pick_place_controller.py#L206) — `PickPlace.detach`

**输入输出：** 输入：40 mm/50 g场景参数、实际刚体状态、视觉世界碰撞对象。输出：真实动态刚体/静态支撑Collider，以及独立的MoveIt世界/附着碰撞表示。

**算法/公式：** SDK施加质量、Collider、非kinematic刚体、摩擦0.8/0.6和恢复系数0；惯量由PhysX依几何/质量计算。ACM仅允许目标与指定手指/必要支撑接触，非目标碰撞约束保留。

**面试问题：** AttachedCollisionObject是不是把物体绑在手上？

**参考回答：** 只改变MoveIt规划几何。Isaac中没有新增焊接、固定约束或逐帧物体位姿写入。闭爪位置检查后为试抬建立规划附着，试抬再验证真实物体离面和随手稳定运动，之后才长距离搬运。

## 10. Pick and Place 状态机

- [planning/pick_place_controller.py:300](../planning/pick_place_controller.py#L300) — `PickPlace.run`
- [planning/pick_place_controller.py:84](../planning/pick_place_controller.py#L84) — `PickPlace.state`
- [planning/pick_place_controller.py:108](../planning/pick_place_controller.py#L108) — `PickPlace.wait`
- [planning/pick_place_controller.py:389](../planning/pick_place_controller.py#L389) — `main`

**输入输出：** 输入：当前场景/感知/命令结果。输出：INIT到SUCCESS的逐阶段事件、抓取/搬运/放置标志；任何关键异常进入FAILED并记录原阶段和原因。

**算法/公式：** OPEN→PREGRASP→APPROACH→CLOSE→TEST_LIFT→LIFT→TRANSFER→PLACE→RELEASE→RETREAT→VERIFY；服务/Action及全任务有超时；持物时每次等待都监控滑移，异常取消自身当前目标。

**面试问题：** 为什么先小幅试抬？

**参考回答：** 仅双指停在物体宽度附近不能证明夹住。先约40 mm试抬，检查真实物体离面，再升至安全高度搬运；低预载绿色011在此阶段检测到14.6 mm滑移，按真实失败退出。

## 11. 成功判定与误差计算

- [evaluate_pick_place.py:11](../evaluate_pick_place.py#L11) — `evaluate`
- [task_io.py:28](../task_io.py#L28) — `relative_pose`
- [planning/pick_place_controller.py:91](../planning/pick_place_controller.py#L91) — `PickPlace.observe_hold`

**输入输出：** 输入：实际遥测JSONL、初始状态、状态机时间戳、视觉结果和场景尺寸。输出：独立success及定位、滑移、抬升、放置、分离、稳定性和非目标位移指标。

**算法/公式：** T_h_o=T_w_h⁻¹·T_w_o；平移滑移为相对位置变化范数；角误差 acos(clamp((trace(R_rel R_refᵀ)−1)/2))；放置误差为基座XY欧氏距离。旋转立方体底部z=p_z−s/2·Σ|R_zj|，再与支撑面比较。

**面试问题：** 如何防止伪成功？

**参考回答：** 独立读取实际动态刚体，而非MoveIt附着对象位置；要求真实抬升、持物相对位姿稳定、最终区域/支撑/分离/稳定以及非目标不被拖走。测试含Action宣称成功但物体始终未抬起的负例，必须拒绝。

## 12. 实际技术问题及解决方案

- [task_io.py:33](../task_io.py#L33) — `validate_capture`
- [planning/pick_place_controller.py:182](../planning/pick_place_controller.py#L182) — `PickPlace.attach`
- [planning/pick_place_controller.py:285](../planning/pick_place_controller.py#L285) — `PickPlace.grip`
- [planning/pick_place_controller.py:225](../planning/pick_place_controller.py#L225) — `PickPlace.move_tcp`

**输入输出：** 依据：实际失败日志、当前实现和三次最终验收。输出：可解释的修复与明确未解决的边界，而不是把所有异常都称为修复完成。

**算法/公式：** 已修复：光学/基座/TCP不一致；同话题手臂包覆盖夹爪；附着时重复world REMOVE；旧录像相机外参混用；接触抖动的Action stall判定。未解决为通用能力：任意放置点跟踪与扰动鲁棒性。物理滑移/TCP验收阈值未为失败放宽。

**面试问题：** 面试时最应强调什么边界？

**参考回答：** 模型与机器人基础库来自第三方，贡献是可靠集成、坐标与控制排错及物理验收；三次成功是有限示例，不是100%统计成功率。当前单帧、已知颜色立方体、每次一个目标，只在仿真验证。

## 建议阅读顺序

1. `README.md` 架构图、三次实测和限制，再看 `configs/multi_object_pick_place.json` 参数。
2. `experiment_io.py` 的数据路径/尺寸约定，`scene_frames.py` 的矩阵方向与校验。
3. `simulation/pick_place_physics.py` 和 `simulation/multi_object_runtime.py`，理解物理状态与采集元数据。
4. `vision/select_targets.py`：先读 `color_mask`，再读 CAD、官方 FoundationPose 调用和三层坐标变换。
5. `planning/pick_place_controller.py`：先读 `run` 看阶段，再依次读 `hand_target`、`move_tcp`、`grip`、`contacts/attach/detach`。
6. `configs/pick_place.urdf.xacro`、`configs/pick_place_controllers.yaml` 和 `planning/pick_place.launch.py`；对照仿真入口中的 `SubscribeGripper` 图节点，解释命令/状态链。
7. `task_io.py`、`evaluate_pick_place.py` 和 `tests/test_multi_object.py`：区分动作返回成功与任务真实成功。
8. `experiments/verified/` 与 `experiments/failures/`，用实测JSON/日志回答“哪里成功、哪里失败、证据是什么”。
9. 最后读 `planning/run_pick_place.py`、`runtime_paths.py`、`tests/test_runtime_paths.py`，理解跨环境编排、超时和发布可移植性。

补充：上游模型内部候选/Refiner/Scorer实现不在此仓库中。实际已用源文件的哈希在 `docs/dependencies.json`，本地已用 `estimater.py` 中 `generate_random_pose_hypo` / `register` / `refiner.predict` / `scorer.predict` 是第三方实现；不能将这些模块说成自行设计或训练。不要用运行目录外未核实的行号冒充本仓库代码索引。
