"""CPU-only regression checks; no ROS, Isaac Sim, GPU or result writes."""
import ast
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scene_frames import (camera_transform, load_scene_frames, rigid_transform,
                          static_transform_arguments)
from vision.verify_world_pose import compute_object_poses


class CoordinateFramesTest(unittest.TestCase):
    def test_nonidentity_base_and_camera(self):
        # Synthetic 90-degree base rotation and nonzero translation: do not
        # assume that the actual scene base has identity orientation.
        base = camera_transform([1, 2, 3], [2**-0.5, 0, 0, 2**-0.5])
        camera = camera_transform([-2, 4, 1], [0.5, 0.5, 0.5, 0.5])
        expected = camera_transform([0.3, -0.2, 0.4], [0, 1, 0, 0])
        world = base @ expected
        observation = np.linalg.solve(camera, world)
        actual_world, actual_base = compute_object_poses(observation, camera, base)
        np.testing.assert_allclose(actual_world, world, atol=1e-12)
        np.testing.assert_allclose(actual_base, expected, atol=1e-12)

    def test_no_legacy_y_offset(self):
        # Identity is a test fixture, not a fallback for missing measurements.
        observation = np.eye(4)
        observation[:3, 3] = [0.55, -0.4, 0.065]
        _, base = compute_object_poses(observation, np.eye(4), np.eye(4))
        np.testing.assert_allclose(base[:3, 3], [0.55, -0.4, 0.065])

    def test_ros_quaternion_roundtrips(self):
        rng = np.random.default_rng(0)
        quaternions = [np.eye(4)[i] for i in range(4)]
        quaternions += list(rng.normal(size=(50, 4)))
        for q in quaternions:
            q = q / np.linalg.norm(q)
            matrix = camera_transform([1, -2, 3], q)
            arguments = static_transform_arguments(matrix, "world", "panda_link0")
            values = dict(zip(arguments[::2], arguments[1::2]))
            restored = camera_transform(
                [float(values[f"--{axis}"]) for axis in "xyz"],
                [float(values[f"--q{axis}"]) for axis in "wxyz"],
            )
            np.testing.assert_allclose(restored, matrix, atol=1e-12)
            self.assertEqual(values["--child-frame-id"], "panda_link0")

    def test_invalid_transforms_rejected(self):
        for matrix in [np.eye(3), np.full((4, 4), np.nan),
                       np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1])]:
            with self.assertRaises(ValueError):
                rigid_transform(matrix, "test")
        with self.assertRaises(ValueError):
            camera_transform([0, 0, 0], [0, 0, 0, 0])

    def test_metadata_requires_measured_base(self):
        old = json.loads((ROOT / "tests/fixtures/pregrasp_120mm_metadata.json").read_text())
        for key in ("world_frame", "robot_base_frame", "camera_frame",
                    "robot_base_transform_world"):
            old.pop(key, None)
        with patch.object(Path, "read_text", return_value=json.dumps(old)):
            with self.assertRaisesRegex(ValueError, "recapture"):
                load_scene_frames()

    def test_metadata_and_launches_share_full_transform(self):
        base = camera_transform([0.2, -0.3, 0.4], [0, 0, 1, 0])
        metadata = {
            "world_frame": "world", "robot_base_frame": "panda_link0",
            "camera_frame": "camera_ros_frame",
            "robot_base_transform_world": base.tolist(),
            "camera_position_world": [1, 2, 3],
            "camera_rotation_wxyz": [1, 0, 0, 0],
        }
        with patch.object(Path, "read_text", return_value=json.dumps(metadata)):
            scene, actual_base, camera = load_scene_frames()
        np.testing.assert_allclose(actual_base, base)
        np.testing.assert_allclose(camera[:3, 3], [1, 2, 3])
        launch_dir = ROOT / "configs"
        expected = static_transform_arguments(base, "world", "panda_link0")
        for name in ("isaac_moveit.launch.py", "isaac_moveit_headless.launch.py"):
            tree = ast.parse((launch_dir / name).read_text())
            assignment = next(n for n in ast.walk(tree)
                              if isinstance(n, ast.Assign)
                              and any(isinstance(t, ast.Name) and t.id == "world2robot_tf_node"
                                      for t in n.targets))
            expression = next(k.value for k in assignment.value.keywords if k.arg == "arguments")
            result = eval(compile(ast.Expression(expression), name, "eval"), {
                "static_transform_arguments": static_transform_arguments,
                "T_world_base": actual_base, "scene": scene,
            })
            self.assertEqual(result, expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
