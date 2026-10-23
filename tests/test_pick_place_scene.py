"""CPU tests. Synthetic fixtures never overwrite baseline data or run GPU models."""
import argparse
import ast
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiment_io import (ExperimentPaths, experiment_paths, build_pregrasp, cube_corners,
                           cube_size, load_metadata, support_box, validate_mesh_size)
from scene_frames import camera_transform, load_scene_frames
from simulation.pick_place_physics import (load_config, initial_position, support_top,
                                           configure_physics, StabilityMonitor)


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "configs/pick_place_40mm.json")

    def test_spawn_and_support_geometry(self):
        self.assertAlmostEqual(support_top(self.config), -0.005)
        self.assertAlmostEqual(initial_position(self.config)[2], 0.035)
        self.assertAlmostEqual(initial_position(self.config)[2]-0.04/2-support_top(self.config), 0.02)
        metadata = {"support_surface": {"center_world_m": self.config["support_center_world_m"],
                    "dimensions_m": self.config["support_dimensions_m"]},
                    "robot_base_transform_world": camera_transform([1, 2, 3], [0, 0, 0, 1]).tolist()}
        dimensions, base = support_box(metadata)
        world = np.array(metadata["robot_base_transform_world"]) @ base
        self.assertEqual(dimensions, [2, 2, 0.06])
        np.testing.assert_allclose(world[:3, 3], self.config["support_center_world_m"])

    def test_stability_needs_support_low_velocity_and_time(self):
        monitor = StabilityMonitor(self.config)
        for i in range(121):
            stable = monitor.update(i/60, [0.55, -0.4, 0.015], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0])
            if i < 120:
                self.assertFalse(stable)
        self.assertTrue(stable)
        self.assertFalse(monitor.update(2.1, [0.55, -0.4, 0.015], [1, 0, 0, 0], [0.1, 0, 0], [0, 0, 0]))

    def test_fallen_suspended_and_slowly_sliding_are_rejected(self):
        for z in [-0.1, 0.1]:
            monitor = StabilityMonitor(self.config)
            for i in range(181):
                stable = monitor.update(i/60, [0.55, -0.4, z], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0])
                self.assertFalse(stable)
        monitor = StabilityMonitor(self.config)
        for i in range(181):
            stable = monitor.update(i/60, [0.55+i/60*0.003, -0.4, 0.015],
                                    [1, 0, 0, 0], [0.003, 0, 0], [0, 0, 0])
            self.assertFalse(stable)

    def test_bad_config_rejected(self):
        for field, value in [("cube_mass_kg", -1), ("restitution", 2),
                             ("support_dimensions_m", [1, -1, 1]),
                             ("cube_xy_world_m", [4, 4])]:
            config = deepcopy(self.config)
            config[field] = value
            with patch.object(Path, "read_text", return_value=json.dumps(config)):
                with self.assertRaises(ValueError):
                    load_config("fixture.json")

    def test_defaults_and_protected_paths(self):
        self.assertEqual(experiment_paths(argparse.Namespace(run_dir=None)).root, ROOT)
        for path in (ROOT, ROOT / "vision_data", ROOT / "videos", ROOT / "experiments"):
            with self.assertRaises(ValueError):
                experiment_paths(argparse.Namespace(run_dir=path))

    def test_cube_and_pregrasp_use_capture_size(self):
        for size in [0.04, 0.12]:
            metadata = {"target_size_m": size}
            corners = cube_corners(cube_size(metadata))
            np.testing.assert_allclose(np.ptp(corners, axis=0), [size]*3)
            transform = np.eye(4)
            transform[:3, 3] = [0.55, -0.4, 0.015]
            target = build_pregrasp(transform, metadata)
            self.assertAlmostEqual(target["pregrasp_position_m"][2], 0.015+size/2+0.12)
            self.assertFalse(target["execute"])
            class Mesh:
                bounds = np.array([[-size/2]*3, [size/2]*3])
            validate_mesh_size(Mesh(), metadata)
            with self.assertRaises(ValueError):
                validate_mesh_size(Mesh(), {"target_size_m": size*2})

    def test_legacy_pregrasp_numerical_parity(self):
        metadata = json.loads((ROOT / "tests/fixtures/pregrasp_120mm_metadata.json").read_text())
        transform = np.array(json.loads((ROOT / "tests/fixtures/pregrasp_120mm_pose_base.json").read_text()))
        target = build_pregrasp(transform, metadata)
        np.testing.assert_allclose(target["pregrasp_position_m"],
                                   transform[:3, 3] + [0, 0, metadata["target_size_m"]/2+0.12], atol=1e-12)

    def test_no_pose_writes_in_simulation_loop(self):
        tree = ast.parse((ROOT / "simulation/run_franka_vision.py").read_text())
        loop = next(node for node in ast.walk(tree) if isinstance(node, ast.While)
                    and isinstance(node.test, ast.Call)
                    and isinstance(node.test.func, ast.Attribute)
                    and node.test.func.attr == "is_running")
        for node in ast.walk(loop):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, ["set_world_pose", "set_local_pose", "AddTranslateOp"])

    def test_installed_camera_optical_axes_without_importing_camera(self):
        import os
        prefix = Path(os.environ.get("ISAACSIM_ENV", ROOT / ".envs/isaacsim"))
        candidates = sorted(prefix.glob("lib/python*/site-packages/isaacsim/exts/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py"))
        camera_source = candidates[0] if candidates else prefix / "camera_source_unavailable"
        if not camera_source.exists():
            self.skipTest("Installed Isaac camera source unavailable")
        tree = ast.parse(camera_source.read_text())
        assignment = next(n for n in tree.body if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == "U_R_TRANSFORM" for t in n.targets))
        transform = np.array(ast.literal_eval(assignment.value.args[0]))
        # Optical +Z forward -> USD -Z; optical +Y down -> USD -Y.
        np.testing.assert_array_equal(transform @ [0, 0, 1, 0], [0, 0, -1, 0])
        np.testing.assert_array_equal(transform @ [0, 1, 0, 0], [0, -1, 0, 0])

    def test_usd_physics_authoring(self):
        try:
            from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
        except ImportError:
            self.skipTest("Run again with the installed Isaac USD library on CPU")
        stage = Usd.Stage.CreateInMemory()
        cube = UsdGeom.Cube.Define(stage, "/World/Cube").GetPrim()
        floor = UsdGeom.Cube.Define(stage, "/World/Floor").GetPrim()
        configure_physics(stage, cube, floor, self.config)
        self.assertTrue(UsdPhysics.RigidBodyAPI(cube).GetRigidBodyEnabledAttr().Get())
        self.assertFalse(UsdPhysics.RigidBodyAPI(cube).GetKinematicEnabledAttr().Get())
        self.assertAlmostEqual(UsdPhysics.MassAPI(cube).GetMassAttr().Get(), 0.05, places=6)
        self.assertFalse(floor.HasAPI(UsdPhysics.RigidBodyAPI))
        for prim in (cube, floor):
            self.assertTrue(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            physics = UsdPhysics.MaterialAPI(material.GetPrim())
            self.assertAlmostEqual(physics.GetStaticFrictionAttr().Get(), 0.8, places=6)
            self.assertAlmostEqual(physics.GetDynamicFrictionAttr().Get(), 0.6, places=6)
            self.assertEqual(physics.GetRestitutionAttr().Get(), 0)


class OfflinePipelineTests(unittest.TestCase):
    def test_synthetic_capture_to_cad_projection_and_target(self):
        try:
            import cv2
            import trimesh
            from PIL import Image
        except ImportError:
            self.skipTest("Run in existing FoundationPose environment for CPU image/CAD checks")
        # Only this newly created disposable fixture is removed by TemporaryDirectory.
        (ROOT / "experiments").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="cpu-fixture-", dir=ROOT / "experiments") as directory:
            paths = ExperimentPaths(Path(directory))
            paths.data.mkdir()
            paths.results.mkdir()
            config = load_config(ROOT / "configs/pick_place_40mm.json")
            metadata = {"experiment_id": paths.root.name, "target_size_m": 0.04,
                        "dynamic_object": True, "target_position_world": [0, 0, 1],
                        "world_frame": "world", "robot_base_frame": "panda_link0",
                        "camera_frame": "camera_ros_frame", "camera_axes": "ros",
                        "robot_base_transform_world": np.eye(4).tolist(),
                        "camera_position_world": [0, 0, 0], "camera_rotation_wxyz": [1, 0, 0, 0]}
            paths.metadata.write_text(json.dumps(metadata))
            (paths.root / "stability.json").write_text(json.dumps({"success": True, "config": config}))
            rgb = np.zeros((64, 64, 3), dtype=np.uint8)
            rgb[24:40, 24:40] = [220, 20, 20]
            Image.fromarray(rgb).save(paths.data / "rgb.png")
            np.save(paths.data / "intrinsics.npy", [[400, 0, 32], [0, 400, 32], [0, 0, 1]])
            np.save(paths.data / "depth.npy", np.ones((64, 64), dtype=np.float32))
            pose = np.eye(4)
            pose[2, 3] = 1
            np.save(paths.results / "object_pose_camera.npy", pose)
            for script in ["vision_data/prepare_object.py", "vision/verify_world_pose.py",
                           "vision/make_pregrasp_target.py", "vision/check_pose.py"]:
                result = subprocess.run([sys.executable, "-B", str(ROOT / script),
                                         "--run-dir", directory], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertGreater(np.count_nonzero(np.asarray(Image.open(paths.data / "mask.png"))), 100)
            mesh = trimesh.load(paths.data / "object.obj", force="mesh", process=False)
            validate_mesh_size(mesh, metadata)
            target = json.loads((paths.results / "pregrasp_target.json").read_text())
            self.assertEqual(target["target_size_m"], 0.04)
            self.assertAlmostEqual(target["pregrasp_position_m"][2], 1.14)
            self.assertTrue((paths.results / "pose_validation.png").exists())
            (paths.root / "stability.json").write_text(json.dumps({"success": False, "config": config}))
            with self.assertRaisesRegex(ValueError, "stability"):
                load_metadata(paths)

    def test_missing_or_existing_run_refused_before_simulator_import(self):
        result = subprocess.run([sys.executable, "-B", str(ROOT / "simulation/run_pick_place_scene.py")],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("requires a NEW --run-dir", result.stderr)
        (ROOT / "experiments").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="cpu-guard-", dir=ROOT / "experiments") as directory:
            result = subprocess.run([sys.executable, "-B", str(ROOT / "simulation/run_pick_place_scene.py"),
                                     "--run-dir", directory], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("FileExistsError", result.stderr)
            self.assertFalse((Path(directory) / "scene_config.json").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
