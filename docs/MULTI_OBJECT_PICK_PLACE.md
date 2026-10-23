> Publication note: this is the accepted-runtime implementation report. Its original backup/video locations are not Git assets. See [stable manifest](STABLE_RUNTIME_MANIFEST.json), [media availability](DEMONSTRATION.md), and [release validation](PUBLICATION_VALIDATION.md). Runtime manifests record the frozen source version; the publication orchestration has configurable deployment paths and has only CPU validation. No new physical grasp logic is introduced.

# 多物体视觉引导物理抓取与放置

工程根目录：`/path/to/this/repository`。本次实验限定于 AutoDL 中的 Isaac Sim 虚拟机器人，没有连接真实机械臂。原有 120 mm 预抓取入口及历史录像保留。

## 运行与验收

在已有 AutoDL 环境中运行，无需安装依赖或手动打开多个终端：

```bash
cd /path/to/this/repository
python3 -B planning/run_pick_place.py \
  --target red_cube --place-x 0.40 --place-y 0.25 --record
```

`--target` 支持 `red_cube`、`blue_cube`、`green_cube`。每次创建独立的 `experiments/pick_place_日期_时间/`，也可以用 `--run-dir experiments/新的名字` 显式指定；已有目录会被拒绝。放置坐标单位为米，属于 `panda_link0`。放置区域必须位于支撑面内，且不能被其他物体占据。

编排器使用已有 Isaac Sim / FoundationPose Conda 环境及 ROS2 Humble，不修改环境。默认 `ROS_DOMAIN_ID=47`、`ROS_LOCALHOST_ONLY=1`，总超时 1000 秒，可用 `--timeout 60..1800` 调整。检测到现有 Isaac/MoveIt 控制进程时拒绝启动重复实例，不能用全局 `pkill` 处理冲突。完成或失败后只清理本次创建的进程组。旧 domain 0 的相机 TF 进程未被终止，也不参与新实验。

所有试验均录制。规范视频路径只有一个：`<run>/videos/pick_place.mp4`。FFmpeg 关闭封装后，检查 H.264、1280×720，并完整解码检查，结果写入 `videos/ffprobe.json`。录像按采样帧以 10 fps 编码；负载高时可能比真实墙钟时间短，精确阶段时间以 `task_result.json` 事件和 `telemetry.jsonl` 为准。

独立重算某次实验的任务级验收（不启动仿真）：

```bash
python3 -B evaluate_pick_place.py --run-dir experiments/verified/multi_red_016
```

此命令只更新该实验的派生 `acceptance.json`，不改写原始遥测、轨迹日志、图像或录像。完整成功须同时满足控制器任务报告、物理遥测独立验收和视频封装检查；编排器返回 0 表示这些检查均通过。

## 实现与文件

| 文件 | 职责 |
|---|---|
| `configs/multi_object_pick_place.json` | 三个独立物体、尺寸质量摩擦、支撑面、运动与验收参数 |
| `configs/pick_place.urdf.xacro` | 复用 Panda 模型，增加独立手指硬件接口和命令话题 |
| `configs/pick_place_controllers.yaml` | 原有手臂轨迹控制器与单驱动手指 GripperActionController |
| `simulation/run_multi_object_scene.py` | 新场景入口，复用原仿真脚本 |
| `simulation/run_franka_vision.py` | 本轮唯一修改的已有源码；按新配置启用多物体运行模块及独立夹爪 Bridge，默认旧入口保留 |
| `simulation/multi_object_runtime.py` | 动态物体、实际状态采集、RGB-D/实测外参、录像与物理遥测 |
| `vision/select_targets.py` | 颜色连通域 Mask、尺寸一致 CAD、三物体 FoundationPose 位姿及投影 |
| `planning/pick_place.launch.py` | 独立 MoveIt/ros2_control 启动，使用本批次实测基座外参 |
| `planning/pick_place_controller.py` | IK、笛卡尔/关节规划、夹爪动作、局部 ACM、状态机及实时滑移检测 |
| `planning/run_pick_place.py` | 单入口跨环境编排、超时、隔离、日志、代码/权重 SHA256 与控制器配置快照 |
| `task_io.py` | 原子 JSON、时效/批次/视觉相机外参一致性检查、夹爪—物体 SE(3) 相对变换 |
| `evaluate_pick_place.py` | 基于实际刚体遥测的独立任务验收 |
| `tests/test_multi_object.py` | 目标选择、歧义/过期数据拒绝、SE(3)、假成功负例测试 |

继续复用已有 `scene_frames.py`、`experiment_io.py`、`simulation/pick_place_physics.py`。40 mm 单物体阶段的修改记录见 `docs/pick_place_phase1.md`。本轮备份位于 `backups/multi_object_20261023_2332/`；历史 44 个受保护文件的 SHA256 清单用于确认旧数据未被覆盖。

新场景从 `multi_blue_015` 起，根目录 `camera_extrinsics.json` 与 RGB-D `metadata.json` 都来自采集时的同一视觉相机，并在启动 MoveIt 前交叉校验。之前开发批次的根目录文件仍为旧录像相机外参，未参与 FoundationPose 或运动目标计算；它们的视觉元数据和物理成功记录仍有效，但不得把那个辅助文件用于复现 TF。历史批次保持原样，新验收优先使用最终批次。

## 数据流与技术依据

1. `MultiObjectRuntime.update/capture`（`simulation/multi_object_runtime.py:82`）等待三个真实刚体全部落稳后保存同一批 RGB、Depth、K 和实际世界位姿。世界支撑面顶部为 -0.005 m，40 mm 方块稳定中心约 0.015 m。50 g 刚体具有动态 RigidBody、Collision、Mass、摩擦 0.8/0.6、恢复系数 0，惯量由 PhysX 根据质量和立方体碰撞几何计算；均匀立方体理论主惯量约 `1.333e-5 kg m²`。支撑面是静态物理 Collider，与 MoveIt 碰撞盒分别创建。
2. `color_mask`（`vision/select_targets.py:19`）按 RGB 主通道比例与连通域提取单实例，拒绝像素不足和同色歧义。用元数据尺寸生成 CAD。`FoundationPose.register` 输入只有 RGB、Depth、K、Mask、CAD；共享预训练 Refiner/Scorer，为三个独立世界碰撞对象分别估计位姿，仅指定 ID 生成抓取任务。没有重新训练，也未把物体真值位置传入推理。
3. 坐标矩阵统一采用 `T_A_B` 将 B 坐标转换到 A。`T_world_object = T_world_camera_optical @ T_camera_object`，`T_base_object = inv(T_world_base) @ T_world_object`。相机使用 SDK `get_world_pose(camera_axes="ros")`，光学轴 X 向右、Y 向下、Z 向前；不重复翻轴。基座从 `/Franka/panda_link0` 的实际 USD 世界变换采集，由 `scene_frames.load_scene_frames` 提供给新 launch。禁止恢复历史 -0.64 m 偏移。
4. Franka USD `/Franka/panda_hand/tool_center` 实测局部 Z 为 **0.1034 m**；58.4 mm 是另一关节局部几何位置，不能作为抓取 TCP。`PickPlace.hand_target`（`planning/pick_place_controller.py:217`）用 `p_hand = p_tcp - R_hand @ t_hand_tcp` 转换目标。当前策略是立方体顶向下夹持，`R_hand=diag(1,-1,-1)`；估计物体中心用于定位，立方体对称旋转不作为通用抓取朝向算法。
5. `PickPlace.move_tcp`（同文件:225）对预抓取/搬运调用 IK 与 MoveGroup/OMPL；接近、抬升、下降、撤离使用 `GetCartesianPath`，步长 5 mm、碰撞检测开启，完成比例必须 ≥0.999，再经 ExecuteTrajectory 执行。手臂速度比例 0.1，笛卡尔比例 0.08；每段检查实际 TCP 位置误差 ≤12 mm。
6. 夹爪名义双指总开口 80 mm，每指 0..40 mm。`panda_finger_joint1` 由 `GripperCommand → ros2_control → isaac_gripper_commands → ROS2 Bridge` 驱动，第二指保持 USD 原生 mimic。原手臂命令继续走 `isaac_joint_commands`，实际状态通过 `isaac_joint_states` 回传。单话题共享导致手臂数据包覆盖手指命令的实测问题已通过独立命令流修复。
7. `scene_setup/contacts/attach/detach`（同文件:155、166、182、206）分别管理世界碰撞盒、指定接触对和规划附着对象。仅目标与两个手指、必要的支撑面接触被允许；其他物体仍为碰撞障碍，未全局禁用碰撞。附着后读取 PlanningScene 确认目标已从 world 移入 attached，其他物体仍在 world。释放后的碰撞盒位置使用实际刚体状态恢复；这是仿真中的规划场景更新，不作为视觉估计或运动目标。
8. `AttachedCollisionObject` 只影响 MoveIt；Isaac Sim 中没有焊接/固定约束，没有逐帧写物体位置。抓取保持由真实手指接触完成。物体真值用于稳定性、定位误差、滑移与最终任务验收；机器人实际状态用于控制反馈。

## 状态与失败处理

`INIT → PERCEIVE → SELECT → OPEN → PREGRASP → APPROACH → CLOSE → TEST_LIFT → LIFT → TRANSFER → PLACE → RELEASE → RETREAT → VERIFY → SUCCESS`。

每次 ROS 服务/Action 有超时；规划拒绝、笛卡尔路径不完整、手指不到位、TCP 超差、物体不离面或滑移超限均进入 FAILED，并取消当前任务自身的 Action。不会盲目跳过失败阶段继续搬运。IK/OMPL 有限规划尝试，整任务不自动无界重试。小幅测试抬升完成并检查物体真实离面后才允许长距离搬运。

夹爪位置控制的 effort 字段不能当成真实闭环力控；实际驱动力受 USD 原始 drive 参数限制（单驱动手指 maxForce 7.2 N）。当前闭爪目标根据物体宽度和配置的 preload 计算；40 mm / 20 mm preload 对应单指位置 0。接触后允许控制器 stall 完成，但仍必须检查双指实际位置和后续物理抬升/滑移。`stall_velocity_threshold=0.1 m/s` 用于兼容物理接触瞬时速度反馈；它本身不构成抓取成功证据。

独立验收 `evaluate`（`evaluate_pick_place.py:11`）读取动态刚体原始遥测，要求：完整阶段、目标选择正确、实际抬升 ≥35 mm、持物观测 ≥2 秒、相对平移滑移 ≤12 mm、相对旋转 ≤0.3 rad、最终平面位置误差 ≤15 mm、底部支撑高度误差 ≤3 mm、张爪撤离后 TCP 分离 ≥80 mm、末段 ≥5 样本且 ≥0.8 秒的速度/漂移稳定检查、非目标最大位移 ≤10 mm。CPU 测试含“Action 声称成功、物体始终没抬升”的负例，必须拒绝。

## 实验记录与限制

真实实验的机器可读汇总见 `experiments/multi_object_summary.json`，各批次保留 `task_result.json`、`acceptance.json`、`telemetry.jsonl`、`perception.log`、`task.log`、`moveit.log`、`orchestration.json` 和录像。参数、源文件和权重哈希见新批次 `manifest.json`；控制器配置另存 `controllers_used.yaml`。原始失败录像不删除、不伪装成成功。

| 能力 | 状态与边界 |
|---|---|
| 三动态物体落稳、RGB-D、三 Mask/CAD、FoundationPose | 已运行验证；固定已知颜色和几何 |
| 指定红/蓝物体真实夹持、搬运、释放、撤离、独立验收 | 已运行验证；对应实际成功批次 |
| 指定绿色物体真实夹持、搬运、放置与验收 | 已运行验证：`multi_green_017`，最终误差 3.26 mm |
| 其他放置位置、初始姿态和参数组合 | 参数接口已实现，但未完成工作空间/扰动的系统运行验证；已知失败点见下文 |
| 旧 120 mm 预抓取 | 保留源码默认分支和历史数据；本轮未重新执行旧预抓取 |
| 任意颜色/开放词汇、遮挡同色多实例、其他几何体 | 未实现 |
| 多物体连续抓取、动态重规划、视觉伺服、真实硬件 | 未实现 |
| 泛化成功率、扰动鲁棒性、力/接触传感定量验证 | 未建立统计结论；本轮为有限场景演示 |

早期失败包括：ROS bridge 缺少进程环境、OmniGraph 重复建图/节点连接路径、共享手臂命令话题覆盖夹爪包、接触抖动导致 stall Action 超时、附着时多余 world REMOVE、低预载导致物体滑移。每次针对日志改动后使用新实验目录，失败均保留。绿色 011 在 TEST_LIFT 检测到 14.6 mm 滑移，明确失败；没有继续搬运或计为成功。质量、物体尺寸、摩擦和物理验收阈值没有为该失败放宽。绿色 012 虽然已经真实抬升，但向 `(0.40,-0.30)` 搬运后 TCP 误差为 14.78 mm，超过 12 mm 检查而退出；该放置点目前不能列为已验证可用。

## 面试讲解重点

FoundationPose 使用 CAD、RGB-D 和实例 Mask 生成候选 6D 位姿，经预训练 Refiner 迭代修正、Scorer 排序；本工程调用第三方 `register(iteration=5)`，不声称自行实现或训练模型。第三方实现可核查 `third_party/FoundationPose/estimater.py:127` 的候选生成、`:159` 的 register、`:215` 的 refiner.predict 和 `:219` 的 scorer.predict。多目标选择来自颜色连通域与 ID，不等价于开放词汇检测。

三个主要工程难点可直接结合代码与失败日志解释：

1. **坐标/TCP 一致性**：光学轴、实测完整基座变换、103.4 mm tool_center，以及 SE(3) 乘法方向；用坐标 CPU 回归和实际 TCP 反馈检查。
2. **控制与规划场景同步**：独立夹爪命令流避免覆盖、只控制 mimic 驱动指；规划附着的 world/attached 生命周期与局部 ACM 必须正确，物理刚体仍独立运动。
3. **物理成功与软件成功分离**：Gripper/MoveIt Action 成功不等于持物成功；用小幅试抬、实时相对位姿、独立遥测验收和完整视频交叉验证，滑移与超时都必须记录为失败。

CPU 检查命令：

```bash
python3 -B tests/test_coordinate_frames.py
python3 -B tests/test_pick_place_scene.py
python3 -B tests/test_multi_object.py
```

坐标 6 项、多物体 9 项通过；40 mm 场景 11 项通过，普通 FoundationPose 环境下 1 项 USD authoring 测试跳过，该项已另用已安装 Isaac USD CPU 库验证。语法、历史哈希和最终各批次结果的最终记录见备份目录 `validation.txt`。不新增依赖，不改权重，不推送公开仓库。

## 最终同版本实测结果（2026-10-24）

三次实验均从三个物体同时存在的初始场景开始，每次只抓取指定物体，放置点均为基座坐标 `(0.40,0.25)` m；它们的 manifest 源文件哈希与当时冻结的运行版源码逐项一致，视觉外参和批次检查全部通过。以下为实际观测值，不能当作总体精度或成功率。

| 目标/批次 | 视觉位置误差 mm | 放置平面误差 mm | 最大持物滑移 mm | 最大抬升 mm | 状态机用时 s | 完整编排 s |
|---|---:|---:|---:|---:|---:|---:|
| red_cube / `multi_red_016` | 0.49 | 1.21 | 0.34 | 197.5 | 63.31 | 116.35 |
| blue_cube / `multi_blue_015` | 0.11 | 1.22 | 1.65 | 212.6 | 74.20 | 126.81 |
| green_cube / `multi_green_017` | 0.85 | 3.26 | 1.36 | 209.5 | 67.74 | 119.91 |

三个最终批次均达到真实抬升、持物稳定、张爪分离、撤离后稳定、非目标保持的独立判据。抓取/搬运/放置标志与独立验收全部为 true，所有本次子进程退出码为 0。完整阶段事件可从各批次 `task_result.json.events` 核查。

- 录像：`experiments/verified/multi_red_016/videos/pick_place.mp4`；H.264 / 1280×720 / 75.700000 秒，已完整解码验证。
- 录像：`experiments/verified/multi_blue_015/videos/pick_place.mp4`；H.264 / 1280×720 / 87.000000 秒，已完整解码验证。
- 录像：`experiments/verified/multi_green_017/videos/pick_place.mp4`；H.264 / 1280×720 / 80.400000 秒，已完整解码验证。

录像包含初始多物体场景、视觉选择、接近、闭爪、真实抬升、搬运、放置释放、撤离与最终场景；叠加状态和目标 ID。最终刚体底部贴合支撑面，TCP 与物体分离约 150 mm，其他物体位移仅为仿真浮点抖动量级。
