import math
import json
from pathlib import Path
import os
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

TARGET = tuple(json.loads((Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion")) / "vision_results/pregrasp_target.json").read_text())["pregrasp_position_m"])
OFFSET = 0.0584

rclpy.init()
node = Node("check_vertical_tcp")
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=False)

try:
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform(
            "panda_link0", "panda_hand", rclpy.time.Time()
        ):
            break
    else:
        raise RuntimeError("无法获得最新末端 TF")

    tf = buffer.lookup_transform(
        "panda_link0", "panda_hand", rclpy.time.Time()
    )

    t = tf.transform.translation
    q = tf.transform.rotation

    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = [v / norm for v in (x, y, z, w)]

    direction = (
        2 * (x*z + w*y),
        2 * (y*z - w*x),
        1 - 2 * (x*x + y*y),
    )

    hand = (t.x, t.y, t.z)
    tcp = tuple(
        hand[i] + OFFSET * direction[i]
        for i in range(3)
    )

    error_mm = math.dist(tcp, TARGET) * 1000
    angle_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, -direction[2])))
    )

    print("===== VERTICAL PREGRASP RESULT =====")
    print("Panda hand:", tuple(round(v, 5) for v in hand))
    print("TCP target:", TARGET)
    print("TCP actual:", tuple(round(v, 5) for v in tcp))
    print("TCP position error (mm):", round(error_mm, 2))
    print("Tool +Z direction:", tuple(round(v, 5) for v in direction))
    print("Downward angle error (deg):", round(angle_deg, 2))
    print("TCP VALIDATION COMPLETE")

finally:
    node.destroy_node()
    rclpy.shutdown()
