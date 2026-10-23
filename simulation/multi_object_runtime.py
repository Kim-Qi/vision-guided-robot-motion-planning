"""Physical scene telemetry/capture/video only. No object-following constraints."""
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image, ImageDraw

from scene_frames import camera_transform, rigid_transform
from simulation.pick_place_physics import configure_physics, initial_position, StabilityMonitor, support_top
from task_io import atomic_json


class MultiObjectRuntime:
    def __init__(self, world, stage, config, paths, first_body, floor):
        from pxr import Gf, UsdGeom
        from isaacsim.core.prims import SingleRigidPrim
        self.world, self.stage, self.config, self.paths = world, stage, config, paths
        self.started = time.monotonic()
        self.bodies = {}
        self.monitors = {}
        self.initial = None
        self.captured = False
        self.last_frame = 0
        self.frames = 0
        self.encoder = None
        self.telemetry = None
        self.video = paths.root / "videos/pick_place.mp4"
        for i, item in enumerate(config["objects"]):
            local = dict(config, cube_xy_world_m=item["xy"])
            self.monitors[item["id"]] = StabilityMonitor(local)
            if i == 0:
                self.bodies[item["id"]] = first_body
                continue
            prim_path = "/World/"+item["id"]
            cube = UsdGeom.Cube.Define(stage, prim_path)
            cube.CreateSizeAttr(config["cube_size_m"])
            cube.AddTranslateOp().Set(Gf.Vec3d(*initial_position(local)))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*item["rgb"])])
            configure_physics(stage, cube.GetPrim(), floor.GetPrim(), local)
            self.bodies[item["id"]] = world.scene.add(SingleRigidPrim(
                prim_path=prim_path, name=item["id"]))

    def initialize(self, vision, display):
        from isaacsim.core.prims import SingleRigidPrim, SingleArticulation
        self.vision, self.display = vision, display
        self.hand = SingleRigidPrim("/Franka/panda_hand", name="task_hand_reader")
        self.hand.initialize()
        self.robot = SingleArticulation("/Franka", name="task_robot_reader")
        self.robot.initialize()
        (self.paths.root / "videos").mkdir(exist_ok=True)
        self.telemetry = (self.paths.root / "telemetry.jsonl").open("x")

    def state(self, steps):
        objects = {}
        for name, body in self.bodies.items():
            p, q = body.get_world_pose()
            objects[name] = {"transform_world": camera_transform(p, q).tolist(),
                             "position_world_m": np.asarray(p).tolist(),
                             "rotation_wxyz": np.asarray(q).tolist(),
                             "linear_velocity_m_s": np.asarray(body.get_linear_velocity()).tolist(),
                             "angular_velocity_rad_s": np.asarray(body.get_angular_velocity()).tolist()}
        p, q = self.hand.get_world_pose()
        joints = dict(zip(self.robot.dof_names, map(float, self.robot.get_joint_positions())))
        import omni.graph.core as og
        command_names = list(og.Controller.attribute("/ActionGraph/SubscribeJointState.outputs:jointNames").get())
        command_positions = np.asarray(og.Controller.attribute("/ActionGraph/SubscribeJointState.outputs:positionCommand").get()).tolist()
        hand = camera_transform(p, q)
        from pxr import UsdGeom, Usd
        prim = self.stage.GetPrimAtPath("/Franka/panda_hand/tool_center")
        local = np.asarray(UsdGeom.Xformable(prim).GetLocalTransformation(), dtype=float).T
        return {"wall_time": time.time(), "simulation_time_s": steps/60,
                "objects": objects, "hand_transform_world": hand.tolist(),
                "tcp_transform_world": (hand @ local).tolist(),
                "applied_joint_positions": np.asarray(self.robot.get_applied_action().joint_positions).tolist(),
                "bridge_last_names": command_names, "bridge_last_positions": command_positions,
                "gripper_command": np.asarray(og.Controller.attribute("/ActionGraph/SubscribeGripper.outputs:positionCommand").get()).tolist(),
                "hand_to_tcp": local.tolist(), "joint_positions": joints,
                "joint_velocities": dict(zip(self.robot.dof_names,map(float,self.robot.get_joint_velocities())))}

    def capture(self, state):
        from pxr import UsdGeom, Usd
        self.paths.data.mkdir(exist_ok=True)
        rgb = self.vision.get_rgba()
        depth = self.vision.get_depth()
        if rgb is None or depth is None:
            return False
        rgb = np.asarray(rgb[:, :, :3], dtype=np.uint8)
        depth = np.asarray(depth, dtype=np.float32)
        Image.fromarray(rgb).save(self.paths.data / "rgb.png")
        np.save(self.paths.data / "depth.npy", depth)
        K = np.asarray(self.vision.get_intrinsics_matrix(), dtype=np.float32)
        np.save(self.paths.data / "intrinsics.npy", K)
        p, q = self.vision.get_world_pose(camera_axes="ros")
        base = self.stage.GetPrimAtPath("/Franka/panda_link0")
        matrix = rigid_transform(np.asarray(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()), dtype=float).T, "measured base")
        first = self.config["objects"][0]["id"]
        metadata = {"world_frame": "world", "robot_base_frame": "panda_link0",
                    "camera_frame": "camera_ros_frame", "camera_axes": "ros",
                    "experiment_id": self.paths.root.name, "scene_id": self.config["scene_id"],
                    "dynamic_object": True, "robot_base_transform_world": matrix.tolist(),
                    "camera_position_world": np.asarray(p).tolist(),
                    "camera_rotation_wxyz": np.asarray(q).tolist(),
                    "intrinsics": K.tolist(), "depth_unit": "meters",
                    "depth_type": "distance_to_image_plane", "resolution": [640, 480],
                    "target_size_m": self.config["cube_size_m"],
                    "target_mass_kg": self.config["cube_mass_kg"],
                    "target_position_world": state["objects"][first]["position_world_m"],
                    "objects": {item["id"]: dict(item, size_m=self.config["cube_size_m"],
                                                  ground_truth=state["objects"][item["id"]])
                                for item in self.config["objects"]},
                    "hand_to_tcp": state["hand_to_tcp"],
                    "support_surface": {"center_world_m": self.config["support_center_world_m"],
                                        "dimensions_m": self.config["support_dimensions_m"],
                                        "top_z_m": support_top(self.config)},
                    "capture_time_s": state["simulation_time_s"], "physics_config": self.config}
        atomic_json(self.paths.metadata, metadata)
        atomic_json(self.paths.root / "camera_extrinsics.json", {
            "experiment_id":self.paths.root.name,
            "parent_frame":"world", "child_frame":metadata["camera_frame"],
            "translation":metadata["camera_position_world"],
            "rotation_xyzw":[float(q[1]),float(q[2]),float(q[3]),float(q[0])],
            "source":"RGB-D observation camera at capture; optical axes"})
        atomic_json(self.paths.root / "stability.json", {"success": True, "config": self.config,
                    "scope": "all objects stable before perception", "objects": state["objects"]})
        atomic_json(self.paths.root / "initial_state.json", state)
        self.encoder = subprocess.Popen([
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", "1280x720", "-framerate", str(self.config["record_fps"]), "-i", "-",
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "20", "-pix_fmt", "yuv420p",
            str(self.video)], stdin=subprocess.PIPE)
        atomic_json(self.paths.root / "capture_ready.json", {"success": True})
        print("MULTI: stable RGB-D capture ready", flush=True)
        return True

    def update(self, steps):
        if time.monotonic()-self.started > self.config["runtime_wall_timeout_s"]:
            raise TimeoutError("Multi-object simulation wall timeout")
        state = self.state(steps)
        if not self.captured:
            stable = []
            for name, body in self.bodies.items():
                data = state["objects"][name]
                stable.append(self.monitors[name].update(steps/60, data["position_world_m"],
                    data["rotation_wxyz"], data["linear_velocity_m_s"], data["angular_velocity_rad_s"]))
            if all(stable):
                self.captured = self.capture(state)
            if not self.captured and steps/60 > self.config["settle_timeout_s"]:
                raise RuntimeError("Multi-object settling/capture timeout")
        if steps % 3 == 0:
            atomic_json(self.paths.root / "state.json", state)
            self.telemetry.write(json.dumps(state)+"\n")
            self.telemetry.flush()
        now = time.monotonic()
        if self.encoder and now-self.last_frame >= 1/self.config["record_fps"]:
            rgba = self.display.get_rgba()
            if rgba is not None:
                image = Image.fromarray(np.asarray(rgba[:, :, :3], dtype=np.uint8))
                draw = ImageDraw.Draw(image)
                status = {"state": "PERCEIVE", "target": "pending"}
                status_path = self.paths.root / "task_status.json"
                if status_path.exists():
                    status = json.loads(status_path.read_text())
                draw.rectangle((12, 12, 780, 65), fill=(20, 24, 30))
                draw.text((25, 25), f"VISION-GUIDED PICK AND PLACE | {status['state']} | TARGET: {status['target']}",
                          fill=(255, 255, 255), font_size=22)
                self.encoder.stdin.write(image.tobytes())
                if self.frames == 0:
                    image.save(self.paths.root / "videos/preview.png")
                self.frames += 1
            self.last_frame = now
        return not (self.paths.root / "STOP_SIMULATION").exists()

    def finish(self):
        if self.telemetry:
            self.telemetry.close()
        if self.encoder:
            self.encoder.stdin.close()
            code = self.encoder.wait(timeout=30)
            if code != 0:
                raise RuntimeError(f"FFmpeg failed: {code}")
            probe = subprocess.run(["ffprobe", "-v", "error", "-show_streams", "-show_format",
                                    "-of", "json", str(self.video)], capture_output=True, text=True, timeout=30)
            probe.check_returncode()
            video_info=json.loads(probe.stdout)
            stream=next(item for item in video_info["streams"] if item["codec_type"]=="video")
            if stream["codec_name"]!="h264" or (stream["width"],stream["height"])!=(1280,720):
                raise RuntimeError("Unexpected video encoding or resolution")
            subprocess.run(["ffmpeg","-v","error","-i",str(self.video),"-f","null","-"],
                           check=True,timeout=60)
            atomic_json(self.paths.root / "videos/ffprobe.json", video_info)
            print("MULTI: video finalized", self.video, "frames", self.frames, flush=True)
