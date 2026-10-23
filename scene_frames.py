"""Scene-derived transforms shared by vision and ROS launch (CPU only).

T_parent_child maps column vectors from child coordinates into parent.
Camera quaternions use Isaac get_world_pose(camera_axes="ros"), wxyz.
"""
import json
import os
from pathlib import Path

import numpy as np

METADATA_PATH = Path(__file__).resolve().parent / "vision_data/metadata.json"


def rigid_transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name}: expected a finite 4x4 transform")
    rotation = matrix[:3, :3]
    if not (np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            and np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5, rtol=0)):
        raise ValueError(f"{name}: expected a rigid right-handed transform")
    return matrix


def camera_transform(position, quaternion_wxyz):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
            or not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-5, rtol=0)):
        raise ValueError("Invalid measured camera position/quaternion (wxyz)")
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = position
    return rigid_transform(matrix, "T_world_camera_optical")


def load_scene_frames(path=None):
    # Optional launch-time selection; default retains the legacy capture location.
    path = Path(path) if path is not None else Path(os.environ.get("ROBOT_SCENE_METADATA", METADATA_PATH))
    data = json.loads(Path(path).read_text())
    required = ("world_frame", "robot_base_frame", "camera_frame",
                "robot_base_transform_world", "camera_position_world",
                "camera_rotation_wxyz")
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(
            f"{path}: missing scene measurements {missing}; "
            "recapture with simulation/run_franka_vision.py; no legacy offset fallback")
    if (data["world_frame"], data["robot_base_frame"], data["camera_frame"]) != (
            "world", "panda_link0", "camera_ros_frame"):
        raise ValueError("Unexpected world/base/optical frame names in scene metadata")
    base = rigid_transform(data["robot_base_transform_world"], "T_world_panda_link0")
    camera = camera_transform(data["camera_position_world"], data["camera_rotation_wxyz"])
    return data, base, camera


def static_transform_arguments(matrix, parent, child):
    """Named ROS2 static_transform_publisher arguments; no Euler ambiguity."""
    matrix = rigid_transform(matrix, f"T_{parent}_{child}")
    rotation = matrix[:3, :3]
    quaternion = np.zeros(4)  # xyzw
    trace = np.trace(rotation)
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        quaternion[:3] = [rotation[2, 1]-rotation[1, 2],
                          rotation[0, 2]-rotation[2, 0],
                          rotation[1, 0]-rotation[0, 1]]
        quaternion[:3] /= scale
        quaternion[3] = scale / 4
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i+1) % 3, (i+2) % 3
        scale = 2 * np.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        quaternion[i] = scale / 4
        quaternion[j] = (rotation[j, i] + rotation[i, j]) / scale
        quaternion[k] = (rotation[k, i] + rotation[i, k]) / scale
        quaternion[3] = (rotation[k, j] - rotation[j, k]) / scale
    quaternion /= np.linalg.norm(quaternion)
    arguments = []
    for flag, value in zip(("--x", "--y", "--z", "--qx", "--qy", "--qz", "--qw"),
                           [*matrix[:3, 3], *quaternion]):
        arguments.extend([flag, str(float(value))])
    return arguments + ["--frame-id", parent, "--child-frame-id", child]
