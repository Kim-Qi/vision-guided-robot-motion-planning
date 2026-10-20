import os
import json
from pathlib import Path
import numpy as np

ROOT = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion"))
DATA = ROOT / "vision_data"
OUT = ROOT / "vision_results"

T_opt_object = np.load(OUT / "object_pose_camera.npy")
assert T_opt_object.shape == (4, 4)

# 与 run_franka_vision.py 的视觉相机配置一致
eye = np.array([1.75, 1.15, 1.45], dtype=float)
target = np.array([0.55, -0.40, 0.065], dtype=float)

forward = target - eye
forward /= np.linalg.norm(forward)

right = np.cross(forward, [0.0, 0.0, 1.0])
right /= np.linalg.norm(right)

up = np.cross(right, forward)

# Isaac Sim USD camera: X right, Y up, -Z forward
R_world_usd = np.column_stack((right, up, -forward))

# OpenCV optical: X right, Y down, Z forward
R_usd_opt = np.diag([1.0, -1.0, -1.0])

T_world_opt = np.eye(4)
T_world_opt[:3, :3] = R_world_usd @ R_usd_opt
T_world_opt[:3, 3] = eye

# FoundationPose 输出：物体局部坐标 -> 相机光学坐标
T_world_object = T_world_opt @ T_opt_object

# 已验证的 Isaac Sim / ROS2: world -> panda_link0 为单位变换
T_world_base = np.eye(4)
T_world_base[:3, 3] = [0.0, 0.0, 0.0]

T_base_object = np.linalg.inv(T_world_base) @ T_world_object

# Isaac Sim 测试物体的真实中心
gt_world = np.array([0.55, -0.40, 0.065])
estimated_world = T_world_object[:3, 3]

error_m = np.linalg.norm(estimated_world - gt_world)

np.save(OUT / "object_pose_world.npy", T_world_object)
np.save(OUT / "object_pose_panda_link0.npy", T_base_object)

result = {
    "estimated_world_position_m": estimated_world.tolist(),
    "ground_truth_world_position_m": gt_world.tolist(),
    "translation_error_m": float(error_m),
    "estimated_panda_link0_position_m": T_base_object[:3, 3].tolist(),
    "extrinsics_source": "Reconstructed from Isaac Sim camera configuration",
}

(OUT / "pose_accuracy.json").write_text(
    json.dumps(result, indent=2)
)

print("===== FOUNDATIONPOSE POSITION VALIDATION =====")
print("Estimated world XYZ (m):", np.round(estimated_world, 5))
print("Ground truth XYZ (m):", gt_world)
print("Translation error (mm):", round(error_m * 1000, 2))
print()
print("===== PANDA BASE TARGET =====")
print("Object XYZ in panda_link0 (m):")
print(np.round(T_base_object[:3, 3], 5))
print()
print("FULL TRANSFORM:")
print(np.round(T_base_object, 5))
print("POSE TRANSFORM VALIDATION COMPLETE")
