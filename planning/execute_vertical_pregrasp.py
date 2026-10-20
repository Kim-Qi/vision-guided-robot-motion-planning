import os
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState


BASE = "panda_link0"
TIP = "panda_hand"
GROUP = "panda_arm"
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


class IKPlanner(Node):
    def __init__(self):
        super().__init__("cartesian_ik_planner")
        self.joints = None
        self.joint_rx = None

        self.create_subscription(
            JointState, "/joint_states",
            self.on_joints, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik"
        )
        self.move_client = ActionClient(
            self, MoveGroup, "/move_action"
        )

    def on_joints(self, msg):
        self.joints = msg
        self.joint_rx = time.monotonic()

    def wait_for(self, predicate, seconds=15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def run(self, dx, execute):
        if execute:
            print(
                "VISION: VERTICAL PREGRASP EXECUTION ENABLED",
                flush=True,
            )

        print("TEST: Waiting for joint states and TF", flush=True)

        def ready():
            fresh = (
                self.joint_rx is not None
                and time.monotonic() - self.joint_rx < 0.5
            )
            return fresh and self.tf_buffer.can_transform(
                BASE, TIP, rclpy.time.Time()
            )

        if not self.wait_for(ready):
            raise RuntimeError("没有收到新鲜关节状态或动态 TF")

        tf = self.tf_buffer.lookup_transform(
            BASE, TIP, rclpy.time.Time()
        )

        t = tf.transform.translation
        q = tf.transform.rotation

        print(
            f"TEST: Current hand = "
            f"({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
        )

        # FoundationPose 估计位置 -> 预抓取目标
        import json
        from pathlib import Path

        target_file = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion")) / "vision_results/pregrasp_target.json"

        with target_file.open() as f:
            target_data = json.load(f)

        assert target_data["frame_id"] == BASE
        assert target_data["execute"] is False

        xyz = target_data["pregrasp_position_m"]
        assert len(xyz) == 3
        assert all(__import__("math").isfinite(v) for v in xyz)

        pose = PoseStamped()
        pose.header.frame_id = BASE
        # xyz 为希望临时 TCP 到达的位置
        # TCP 位于 panda_hand 局部 +Z 方向 0.0584 m
        tcp_offset = 0.0584

        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2]) + tcp_offset

        # 夹爪局部 +Z 竖直向下
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        print(
            "VISION: Vertical hand target = "
            f"({pose.pose.position.x:.4f}, "
            f"{pose.pose.position.y:.4f}, "
            f"{pose.pose.position.z:.4f})",
            flush=True,
        )

        print(
            "VISION: FoundationPose pregrasp target = "
            f"({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f})",
            flush=True,
        )
        print(
            "VISION: Orientation = vertical downward",
            flush=True,
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = TIP
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state = self.joints
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = DurationMsg(sec=5)

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=12
        )

        if not future.done():
            raise RuntimeError("IK 请求超时")

        response = future.result()
        if response is None:
            raise RuntimeError("IK 服务调用失败")

        print("TEST: IK error code =", response.error_code.val)

        if response.error_code.val != 1:
            raise RuntimeError("IK 求解失败，未下发运动")

        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position
        ))

        for name in JOINTS:
            if name not in solution:
                raise RuntimeError(f"IK 缺失关节: {name}")
            print(f"  {name}: {solution[name]:.4f}")

        if not self.move_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("/move_action 不可用")

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = GROUP
        request.num_planning_attempts = 5
        request.allowed_planning_time = 8.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        constraints = Constraints()
        for name in JOINTS:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(solution[name])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        request.start_state.is_diff = True

        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = False

        print("TEST: Sending MoveIt2 goal")
        print("TEST: Execute =", execute)

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=15
        )

        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("MoveIt2 Action 请求超时")

        handle = send_future.result()

        if not handle.accepted:
            raise RuntimeError("MoveIt2 拒绝目标")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=90
        )

        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("MoveIt2 规划/执行超时")

        result = result_future.result().result

        print("TEST: MoveIt error code =", result.error_code.val)
        print(
            "TEST: Trajectory points =",
            len(result.planned_trajectory.joint_trajectory.points)
        )

        if result.error_code.val != 1:
            raise RuntimeError("MoveIt2 未成功完成规划/执行")

        if execute:
            print("TEST: CARTESIAN IK EXECUTION SUCCESS")
        else:
            print("TEST: CARTESIAN IK PLANNING SUCCESS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dx", type=float, default=0.05)
    args = parser.parse_args()

    if not 0.0 < args.dx <= 0.08:
        raise ValueError("首次测试只允许 0 < dx <= 0.08 米")

    rclpy.init()
    node = IKPlanner()

    try:
        node.run(args.dx, args.execute)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
