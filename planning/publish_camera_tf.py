import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_frames import load_scene_frames
from experiment_io import parse_experiment

def main():
    paths = parse_experiment("Publish optical camera TF from selected capture")
    data, _, _ = load_scene_frames(paths.metadata)

    rclpy.init()
    node = Node("isaac_camera_tf_publisher")
    broadcaster = StaticTransformBroadcaster(node)

    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = data["world_frame"]
    msg.child_frame_id = data["camera_frame"]

    # Use the RGB-D camera measurement, not the legacy display-camera JSON.
    x, y, z = data["camera_position_world"]
    qw, qx, qy, qz = data["camera_rotation_wxyz"]

    msg.transform.translation.x = x
    msg.transform.translation.y = y
    msg.transform.translation.z = z

    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw

    broadcaster.sendTransform(msg)

    print("TF2: world -> camera_ros_frame published", flush=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
