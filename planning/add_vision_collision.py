import json
from pathlib import Path
import sys
import numpy as np

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    PlanningSceneComponents,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiment_io import parse_experiment, load_metadata, cube_size
from scene_frames import static_transform_arguments


class VisionCollision(Node):
    def __init__(self, paths):
        super().__init__("vision_collision_scene")
        self.paths = paths
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(
                f"Service unavailable: {client.srv_name}"
            )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=15
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                f"Service failed: {client.srv_name}"
            )
        return future.result()

    def run(self):
        metadata = load_metadata(self.paths)
        data = json.loads((self.paths.results / "pregrasp_target.json").read_text())
        size = cube_size(metadata)
        if "target_size_m" in data and not np.isclose(data["target_size_m"], size):
            raise ValueError("Target size differs from capture metadata")
        xyz = data["object_center_m"]

        assert data["frame_id"] == "panda_link0"
        assert len(xyz) == 3

        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = "foundationpose_target_cube"
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size] * 3

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        if self.paths.root == ROOT:
            pose.orientation.w = 1.0  # preserve the validated legacy box convention
        else:
            transform = np.load(self.paths.results / "object_pose_panda_link0.npy", allow_pickle=False)
            if not np.allclose(transform[:3, 3], xyz, atol=1e-8, rtol=0):
                raise ValueError("Target and estimated object pose do not match")
            arguments = static_transform_arguments(transform, "panda_link0", "object")
            values = dict(zip(arguments[::2], arguments[1::2]))
            for axis in "xyzw":
                setattr(pose.orientation, axis, float(values[f"--q{axis}"]))

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        response = self.call(self.apply_client, request)

        print("SCENE: Apply success =", response.success, flush=True)
        if not response.success:
            raise RuntimeError("MoveIt2 rejected collision object")

        # 从 MoveIt2 读回场景，验证物体确实存在
        query = GetPlanningScene.Request()
        query.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        result = self.call(self.get_client, query)

        matches = [
            o for o in result.scene.world.collision_objects
            if o.id == obj.id
        ]

        if len(matches) != 1:
            raise RuntimeError(
                "Collision object not found in MoveIt2 scene"
            )

        saved = matches[0]
        if not saved.primitives:
            raise RuntimeError("Collision geometry is empty")

        print("SCENE: Object ID =", saved.id, flush=True)
        print(
            "SCENE: Dimensions =",
            list(saved.primitives[0].dimensions),
            flush=True,
        )
        print(
            "SCENE: Object center =",
            [round(float(v), 5) for v in xyz],
            flush=True,
        )
        print("VISION COLLISION SCENE SUCCESS", flush=True)


def main():
    paths = parse_experiment("Add a capture-sized MoveIt collision cube; does not move robot")
    load_metadata(paths)
    rclpy.init()
    node = VisionCollision(paths)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
