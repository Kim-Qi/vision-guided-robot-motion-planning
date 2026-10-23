"""CPU-only paths and geometry shared by the legacy and isolated experiments."""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class ExperimentPaths:
    root: Path = ROOT

    @property
    def data(self):
        return self.root / "vision_data"

    @property
    def results(self):
        return self.root / "vision_results"

    @property
    def metadata(self):
        return self.data / "metadata.json"


def add_run_argument(parser):
    parser.add_argument("--run-dir", type=Path,
                        help="Isolated experiment directory; omitted = legacy 120 mm paths")


def experiment_paths(args):
    if args.run_dir is None:
        return ExperimentPaths()
    run = args.run_dir.expanduser().resolve()
    if ROOT / "experiments" not in run.parents:
        raise ValueError("--run-dir must be a child of the project experiments/ directory")
    return ExperimentPaths(run)


def parse_experiment(description):
    parser = argparse.ArgumentParser(description=description)
    add_run_argument(parser)
    return experiment_paths(parser.parse_args())


def load_metadata(paths):
    metadata = json.loads(paths.metadata.read_text())
    cube_size(metadata)
    if paths.root != ROOT and metadata.get("experiment_id") != paths.root.name:
        raise ValueError("Capture experiment_id does not match --run-dir")
    if paths.root != ROOT:
        report = json.loads((paths.root / "stability.json").read_text())
        if report.get("success") is not True:
            raise ValueError("Physical stability validation has not succeeded")
        if not np.isclose(cube_size(metadata), report["config"]["cube_size_m"], atol=1e-9, rtol=0):
            raise ValueError("Capture size differs from validated physical scene")
    return metadata


def cube_size(metadata):
    size = float(metadata["target_size_m"])
    if not np.isfinite(size) or size <= 0:
        raise ValueError("target_size_m must be positive and finite")
    return size


def cube_corners(size):
    half = cube_size({"target_size_m": size}) / 2
    return np.array([[x, y, z] for x in [-half, half]
                     for y in [-half, half] for z in [-half, half]], dtype=np.float32)


def validate_mesh_size(mesh, metadata):
    size = cube_size(metadata)
    expected = np.array([[-size/2] * 3, [size/2] * 3])
    if not np.allclose(mesh.bounds, expected, atol=1e-6, rtol=0):
        raise ValueError("CAD bounds do not match capture target_size_m / local origin")


def build_pregrasp(transform, metadata):
    from scene_frames import rigid_transform, static_transform_arguments
    transform = rigid_transform(transform, "T_base_object")
    size = cube_size(metadata)
    clearance = float(metadata.get("pregrasp_clearance_m", 0.12))
    if not np.isfinite(clearance) or clearance <= 0:
        raise ValueError("pregrasp_clearance_m must be positive")
    center = transform[:3, 3]
    # Height above the oriented box, along base +Z; orientation is not a grasp policy.
    half_height = (size / 2 * np.abs(transform[2, :3]).sum()
                   if metadata.get("dynamic_object") else size / 2)
    args = static_transform_arguments(transform, "panda_link0", "object")
    values = dict(zip(args[::2], args[1::2]))
    return {
        "frame_id": "panda_link0", "source": "FoundationPose object position",
        "object_center_m": center.tolist(), "target_size_m": size,
        "object_rotation_xyzw": [float(values[f"--q{k}"]) for k in "xyzw"],
        "pregrasp_position_m": (center + [0, 0, half_height + clearance]).tolist(),
        "position_only": True, "execute": False,
    }


def support_box(metadata):
    from scene_frames import rigid_transform
    support = metadata["support_surface"]
    dimensions = np.asarray(support["dimensions_m"], dtype=float)
    center = np.asarray(support["center_world_m"], dtype=float)
    if (dimensions.shape != (3,) or center.shape != (3,)
            or not np.isfinite(dimensions).all() or not np.isfinite(center).all()
            or np.any(dimensions <= 0)):
        raise ValueError("Invalid physical support geometry")
    world = np.eye(4)
    world[:3, 3] = center
    base = rigid_transform(metadata["robot_base_transform_world"], "T_world_base")
    return dimensions.tolist(), np.linalg.solve(base, world)
