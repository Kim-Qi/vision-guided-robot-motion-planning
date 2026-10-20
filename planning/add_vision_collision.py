import os
import json
from pathlib import Path

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


ROOT = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion"))
TARGET = ROOT / "vision_results/pregrasp_target.json"


class VisionCollision(Node):
    def __init__(self):
        super().__init__("vision_collision_scene")
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
        data = json.loads(TARGET.read_text())
        xyz = data["object_center_m"]

        assert data["frame_id"] == "panda_link0"
        assert len(xyz) == 3

        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = "foundationpose_target_cube"
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.12, 0.12, 0.12]

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0

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
    rclpy.init()
    node = VisionCollision()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
