from isaacsim import SimulationApp
import os

# 1. 启动 Isaac Sim 无界面仿真
simulation_app = SimulationApp({
    "headless": True,
    "active_gpu": 0,
    "multi_gpu": False,
})

import time
import traceback

import omni.graph.core as og
import omni.timeline
import omni.usd
import usdrt.Sdf

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdShade, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path

from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.viewports import set_camera_view
import numpy as np
import subprocess
from PIL import Image
from pathlib import Path


ROBOT_PATH = "/Franka"

try:
    # 2. 启用 Isaac Sim ROS2 Bridge
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    # 3. 创建仿真世界
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
    )

    stage = omni.usd.get_context().get_stage()

    # 演示场景照明：环境光 + 定向光
    dome = UsdLux.DomeLight.Define(stage, "/World/RecordDomeLight")
    dome.CreateIntensityAttr(700.0)

    sun = UsdLux.DistantLight.Define(stage, "/World/RecordKeyLight")
    sun.CreateIntensityAttr(1000.0)
    sun.CreateAngleAttr(0.5)

    print("VIDEO: LIGHTS CREATED", flush=True)



    # 深灰色摄影棚地面
    floor = UsdGeom.Cube.Define(stage, "/World/StudioFloor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.035))
    floor.AddScaleOp().Set(Gf.Vec3f(200.0, 200.0, 0.06))

    material = UsdShade.Material.Define(stage, "/World/StudioMaterial")
    shader = UsdShade.Shader.Define(stage, "/World/StudioMaterial/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.12, 0.13, 0.15)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    UsdShade.MaterialBindingAPI.Apply(
        floor.GetPrim()
    ).Bind(material)

    # 降低环境光，避免白色机械臂过曝
    dome.GetIntensityAttr().Set(350.0)
    sun.GetIntensityAttr().Set(1100.0)

    print("VIDEO: DARK STUDIO CREATED", flush=True)

    # 4. 加载官方 Franka Panda USD
    assets_root = get_assets_root_path()

    if not assets_root:
        raise RuntimeError("Isaac Sim assets root not found")

    usd_path = (
        assets_root
        + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    )

    print("TEST: Loading Franka:", usd_path, flush=True)

    prim = stage.DefinePrim(ROBOT_PATH, "Xform")
    prim.GetReferences().AddReference(usd_path)

    # 与 NVIDIA 官方 MoveIt 示例保持一致的基座位置和朝向
    xform = UsdGeom.Xformable(prim)

    simulation_app.update()



    # 验证 Isaac Sim 实际机器人根节点的世界变换
    from pxr import UsdGeom as _UsdGeom

    for check_path in [
        "/Franka",
        "/Franka/panda_link0",
        "/Franka/panda_link0/visuals",
    ]:
        check_prim = stage.GetPrimAtPath(check_path)

        if not check_prim.IsValid():
            print("BASE_CHECK: MISSING", check_path, flush=True)
            continue

        matrix = _UsdGeom.Xformable(
            check_prim
        ).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

        print(
            "BASE_CHECK:", check_path,
            "world_translation:",
            matrix.ExtractTranslation(),
            flush=True,
        )

    # 创建独立的观察相机
    camera = Camera(
        prim_path="/World/RecordCamera",
        resolution=(1280, 720),
        frequency=10,
    )
    world.scene.add(camera)


    # FoundationPose 测试物体：红色立方体
    object_prim = UsdGeom.Cube.Define(stage, "/World/TargetObject")
    object_prim.CreateSizeAttr(0.12)
    object_prim.AddTranslateOp().Set(
        Gf.Vec3d(0.55, -0.40, 0.065)
    )
    object_prim.CreateDisplayColorAttr([
        Gf.Vec3f(0.85, 0.12, 0.10)
    ])

    # 独立视觉相机，不影响展示相机
    vision_camera = Camera(
        prim_path="/World/VisionCamera",
        name="vision_camera",
        resolution=(640, 480),
        frequency=10,
    )
    world.scene.add(vision_camera)

    # 5. 构建 ROS2 Action Graph
    og.Controller.edit(
        {
            "graph_path": "/ActionGraph",
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ReadSimTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
                (
                    "Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "PublishJointState",
                    "isaacsim.ros2.bridge.ROS2PublishJointState",
                ),
                (
                    "SubscribeJointState",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock",
                ),
            ],

            og.Controller.Keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishJointState.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeJointState.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "ArticulationController.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn",
                ),
                (
                    "Context.outputs:context",
                    "PublishJointState.inputs:context",
                ),
                (
                    "Context.outputs:context",
                    "SubscribeJointState.inputs:context",
                ),
                (
                    "Context.outputs:context",
                    "PublishClock.inputs:context",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],

            og.Controller.Keys.SET_VALUES: [
                (
                    "ArticulationController.inputs:robotPath",
                    ROBOT_PATH,
                ),
                (
                    "PublishJointState.inputs:topicName",
                    "isaac_joint_states",
                ),
                (
                    "SubscribeJointState.inputs:topicName",
                    "isaac_joint_commands",
                ),
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(ROBOT_PATH)],
                ),
            ],
        },
    )

    print("TEST: ROS2 Action Graph created", flush=True)

    # 6. 初始化物理世界，开始仿真
    world.reset()

    camera.initialize()
    # 根据观察点自动计算相机朝向，避免手动欧拉角指向错误
    from scipy.spatial.transform import Rotation

    eye = np.array([6.2, 5.8, 3.2], dtype=float)
    target = np.array([0.0, 0.0, 0.18], dtype=float)

    forward = target - eye
    forward /= np.linalg.norm(forward)

    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)

    up = np.cross(right, forward)

    # USD 相机约定：X 向右、Y 向上、-Z 为光轴方向
    rotation_matrix = np.column_stack((right, up, -forward))
    quat_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()
    quat_wxyz = quat_xyzw[[3, 0, 1, 2]]

    camera.set_world_pose(
        position=eye,
        orientation=quat_wxyz,
        camera_axes="usd",
    )
    camera.set_focal_length(9.0)

    print("VIDEO: CAMERA LOOK-AT SET", flush=True)

    # 获取相机光学坐标系相对于仿真 world 的真实外参
    import json

    cam_position, cam_quat = camera.get_world_pose(camera_axes="ros")

    extrinsics = {
        "parent_frame": "world",
        "child_frame": "camera_optical_frame",
        "translation": [float(v) for v in cam_position],
        "rotation_xyzw": [
            float(cam_quat[1]),
            float(cam_quat[2]),
            float(cam_quat[3]),
            float(cam_quat[0]),
        ],
    }

    output = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion")) / "camera_extrinsics.json"
    output.write_text(json.dumps(extrinsics, indent=2))

    print("TF2: CAMERA EXTRINSICS =", extrinsics, flush=True)
    print("TF2: EXTRINSICS SAVED =", output, flush=True)



    # 初始化视觉相机并开启深度采集
    vision_camera.initialize()
    vision_camera.add_distance_to_image_plane_to_frame()

    from scipy.spatial.transform import Rotation

    v_eye = np.array([1.75, 1.15, 1.45], dtype=float)
    v_target = np.array([0.55, -0.40, 0.065], dtype=float)

    forward = v_target - v_eye
    forward /= np.linalg.norm(forward)

    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)

    up = np.cross(right, forward)

    rotation_matrix = np.column_stack((right, up, -forward))
    q_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()

    vision_camera.set_world_pose(
        position=v_eye,
        orientation=q_xyzw[[3, 0, 1, 2]],
        camera_axes="usd",
    )
    vision_camera.set_focal_length(18.0)

    print("VISION: CAMERA INITIALIZED", flush=True)

    video_dir = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion")) / "videos"
    video_dir.mkdir(parents=True, exist_ok=True)
    trigger = video_dir / "START_RECORDING"
    trigger.unlink(missing_ok=True)
    stop_trigger = video_dir / "STOP_RECORDING"
    stop_trigger.unlink(missing_ok=True)

    ffmpeg_process = None
    record_start = None
    last_frame_time = 0.0
    frame_count = 0

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print("TEST: FRANKA SIMULATION RUNNING", flush=True)

    start = time.monotonic()
    steps = 0

    # 持续运行，直到用户停止程序或 SimulationApp 退出
    while simulation_app.is_running():

        world.step(render=True)
        steps += 1

        if steps == 120:
            import json

            out = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion")) / "vision_data"
            out.mkdir(parents=True, exist_ok=True)

            rgba = vision_camera.get_rgba()
            depth = vision_camera.get_depth()

            if rgba is None or depth is None:
                print(
                    "VISION: ERROR camera frame unavailable",
                    flush=True,
                )
            else:
                rgb = np.asarray(rgba[:, :, :3], dtype=np.uint8)
                depth = np.asarray(depth, dtype=np.float32)

                Image.fromarray(rgb).save(out / "rgb.png")
                np.save(out / "depth.npy", depth)

                valid = np.isfinite(depth) & (depth > 0)
                depth_vis = np.zeros_like(depth, dtype=np.uint8)

                if np.any(valid):
                    low, high = np.percentile(
                        depth[valid], [5, 95]
                    )
                    if high > low:
                        depth_vis[valid] = np.uint8(
                            np.clip(
                                (depth[valid] - low)
                                / (high - low) * 255,
                                0, 255
                            )
                        )

                Image.fromarray(depth_vis).save(
                    out / "depth_preview.png"
                )

                width, height = vision_camera.get_resolution()
                focal = vision_camera.get_focal_length()
                aperture = vision_camera.get_horizontal_aperture()

                fx = focal * width / aperture
                fy = focal * width / aperture

                K = np.array([
                    [fx, 0.0, width / 2.0],
                    [0.0, fy, height / 2.0],
                    [0.0, 0.0, 1.0],
                ], dtype=np.float32)
                np.save(out / "intrinsics.npy", K)

                pos, quat = vision_camera.get_world_pose(
                    camera_axes="ros"
                )

                metadata = {
                    "resolution": [640, 480],
                    "depth_unit": "meters",
                    "depth_type": "distance_to_image_plane",
                    "intrinsics": K.tolist(),
                    "camera_position_world": pos.tolist(),
                    "camera_rotation_wxyz": quat.tolist(),
                    "target_position_world": [0.55, -0.40, 0.065],
                    "target_size_m": 0.12,
                }

                (out / "metadata.json").write_text(
                    json.dumps(metadata, indent=2)
                )

                print(
                    "VISION: RGB", rgb.shape,
                    "DEPTH", depth.shape,
                    "VALID DEPTH", int(valid.sum()),
                    flush=True,
                )
                print("VISION: DATA EXPORTED", flush=True)



        if trigger.exists() and ffmpeg_process is None:
            video_path = video_dir / "franka_moveit_showcase.mp4"

            ffmpeg_process = subprocess.Popen([
                "ffmpeg", "-y", "-loglevel", "error",
                "-f", "rawvideo",
                "-pixel_format", "rgb24",
                "-video_size", "1280x720",
                "-framerate", "10",
                "-i", "-",
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-pix_fmt", "yuv420p",
                str(video_path),
            ], stdin=subprocess.PIPE)

            trigger.unlink(missing_ok=True)
            record_start = time.monotonic()
            last_frame_time = record_start - 0.1
            print("VIDEO: RECORDING STARTED", flush=True)

        if ffmpeg_process is not None:
            now = time.monotonic()

            if now - last_frame_time >= 0.1:
                rgba = camera.get_rgba()

                if rgba is not None:
                    rgb = np.asarray(
                        rgba[:, :, :3], dtype=np.uint8
                    )

                    if frame_count in (0, 10):
                        print(
                            "VIDEO: RGB stats",
                            "min=", int(rgb.min()),
                            "max=", int(rgb.max()),
                            "mean=", round(float(rgb.mean()), 2),
                            "std=", round(float(rgb.std()), 2),
                            flush=True,
                        )

                    ffmpeg_process.stdin.write(rgb.tobytes())
                    frame_count += 1

                    if frame_count == 1:
                        Image.fromarray(rgb).save(
                            str(video_dir / "showcase_preview.png")
                        )
                        print("VIDEO: PREVIEW SAVED", flush=True)

                last_frame_time = now

            if stop_trigger.exists():
                stop_trigger.unlink()
                ffmpeg_process.stdin.close()
                ret = ffmpeg_process.wait()

                print("VIDEO: FRAME COUNT =", frame_count, flush=True)
                print("VIDEO: FFMPEG EXIT =", ret, flush=True)
                print("VIDEO: OUTPUT =", video_path, flush=True)

                ffmpeg_process = None
                frame_count = 0
                print("VIDEO: RECORDING STOPPED - SIMULATION CONTINUES", flush=True)


        if steps % 600 == 0:
            print(
                f"TEST: Simulation steps = {steps}",
                flush=True,
            )


except Exception:
    traceback.print_exc()
    raise

finally:
    print("TEST: Shutting down", flush=True)
    simulation_app.close()
