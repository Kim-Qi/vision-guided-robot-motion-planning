import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scene_frames import load_scene_frames, rigid_transform
from experiment_io import add_run_argument, experiment_paths, load_metadata

OUT = ROOT / "vision_results"


def compute_object_poses(T_opt_object, T_world_opt, T_world_base):
    T_opt_object = rigid_transform(T_opt_object, "T_opt_object")
    T_world_opt = rigid_transform(T_world_opt, "T_world_opt")
    T_world_base = rigid_transform(T_world_base, "T_world_base")
    world = T_world_opt @ T_opt_object
    base = np.linalg.solve(T_world_base, world)
    return world, base


def main():
    parser = argparse.ArgumentParser()
    add_run_argument(parser)
    parser.add_argument("--check-only", action="store_true",
                        help="Validate transforms and print poses without writing results")
    args = parser.parse_args()
    paths = experiment_paths(args)
    out = paths.results
    load_metadata(paths)

    # Fail before writing anything if a legacy capture lacks a measured base pose.
    metadata, T_world_base, T_world_opt = load_scene_frames(paths.metadata)
    T_opt_object = np.load(out / "object_pose_camera.npy", allow_pickle=False)
    T_world_object, T_base_object = compute_object_poses(
        T_opt_object, T_world_opt, T_world_base
    )
    gt_world = np.asarray(metadata["target_position_world"], dtype=float)
    if gt_world.shape != (3,) or not np.isfinite(gt_world).all():
        raise ValueError("Invalid target_position_world in capture metadata")
    estimated_world = T_world_object[:3, 3]
    error_m = np.linalg.norm(estimated_world - gt_world)

    result = {
        "estimated_world_position_m": estimated_world.tolist(),
        "ground_truth_world_position_m": gt_world.tolist(),
        "translation_error_m": float(error_m),
        "estimated_panda_link0_position_m": T_base_object[:3, 3].tolist(),
        "extrinsics_source": str(paths.metadata),
        "robot_base_transform_world": T_world_base.tolist(),
    }
    if not args.check_only:
        np.save(out / "object_pose_world.npy", T_world_object)
        np.save(out / "object_pose_panda_link0.npy", T_base_object)
        (out / "pose_accuracy.json").write_text(json.dumps(result, indent=2))

    print("===== FOUNDATIONPOSE POSITION VALIDATION =====")
    print("Estimated world XYZ (m):", np.round(estimated_world, 5))
    print("Ground truth XYZ (m):", gt_world)
    print("Translation error (mm):", round(error_m * 1000, 2))
    print("Object XYZ in panda_link0 (m):", np.round(T_base_object[:3, 3], 5))
    print("FULL BASE TRANSFORM:")
    print(np.round(T_world_base, 8))
    print("FULL OBJECT TRANSFORM IN BASE:")
    print(np.round(T_base_object, 5))
    print("POSE TRANSFORM VALIDATION COMPLETE", "(no files written)" if args.check_only else "")


if __name__ == "__main__":
    main()
