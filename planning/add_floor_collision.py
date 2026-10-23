from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiment_io import ROOT, parse_experiment, load_metadata, support_box
from scene_frames import static_transform_arguments

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

paths = parse_experiment("Add a MoveIt support box; not an Isaac physical collider")
metadata = load_metadata(paths) if paths.root != ROOT else None
rclpy.init()
node = Node("add_floor_collision")

apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
get_client = node.create_client(GetPlanningScene, "/get_planning_scene")

def call(client, request):
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError(f"Service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"Service failed: {client.srv_name}")
    return future.result()

try:
    obj = CollisionObject()
    obj.header.frame_id = "panda_link0"
    obj.id = "studio_floor"
    obj.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    if metadata is None:
        box.dimensions = [20.0, 20.0, 0.06]  # unchanged legacy planning floor
        support_transform = None
    else:
        box.dimensions, support_transform = support_box(metadata)

    pose = Pose()
    # Isaac Sim floor world center: (0, 0, -0.035)
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = -0.035
    pose.orientation.w = 1.0
    if support_transform is not None:
        pose.position.x, pose.position.y, pose.position.z = support_transform[:3, 3].tolist()
        arguments = static_transform_arguments(support_transform, "panda_link0", "support")
        values = dict(zip(arguments[::2], arguments[1::2]))
        for axis in "xyzw":
            setattr(pose.orientation, axis, float(values[f"--q{axis}"]))

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(obj)

    req = ApplyPlanningScene.Request()
    req.scene = scene
    result = call(apply_client, req)

    print("FLOOR: Apply success =", result.success, flush=True)
    if not result.success:
        raise RuntimeError("Failed to add floor")

    query = GetPlanningScene.Request()
    query.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )

    response = call(get_client, query)
    objects = {
        o.id: o for o in response.scene.world.collision_objects
    }

    for name in ["studio_floor", "foundationpose_target_cube"]:
        if name not in objects:
            raise RuntimeError(f"Missing collision object: {name}")
        print("FLOOR: Verified object =", name, flush=True)

    print("PLANNING SCENE VERIFIED", flush=True)

finally:
    node.destroy_node()
    rclpy.shutdown()
