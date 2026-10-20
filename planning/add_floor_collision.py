import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

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
    box.dimensions = [20.0, 20.0, 0.06]

    pose = Pose()
    # Isaac Sim floor world center: (0, 0, -0.035)
    # panda_link0 origin in world: (0, 0, 0)
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = -0.035
    pose.orientation.w = 1.0

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
