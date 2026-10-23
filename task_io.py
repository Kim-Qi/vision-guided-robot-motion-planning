"""Small file protocol shared by simulator, planner and independent evaluation."""
import json
import os
from pathlib import Path
import time

import numpy as np


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False))
    temporary.replace(path)


def read_state(run, max_age=3):
    state = json.loads((Path(run) / "state.json").read_text())
    if time.time()-state["wall_time"] > max_age:
        raise RuntimeError("Simulator telemetry is stale")
    return state


def translation(matrix):
    return np.asarray(matrix, dtype=float)[:3, 3]


def relative_pose(state, target):
    return np.linalg.solve(np.asarray(state["hand_transform_world"]),
                           np.asarray(state["objects"][target]["transform_world"]))


def validate_capture(run):
    """Reject mixed batches or video-camera extrinsics before any robot action."""
    from scene_frames import camera_transform
    run=Path(run)
    metadata=json.loads((run/"vision_data/metadata.json").read_text())
    extrinsics=json.loads((run/"camera_extrinsics.json").read_text())
    if any(data.get("experiment_id")!=run.name for data in [metadata,extrinsics]):
        raise ValueError("Capture calibration experiment batch mismatch")
    if extrinsics["parent_frame"]!="world" or extrinsics["child_frame"]!=metadata["camera_frame"]:
        raise ValueError("Capture camera frame mismatch")
    q=extrinsics["rotation_xyzw"]
    recorded=camera_transform(extrinsics["translation"],[q[3],*q[:3]])
    optical=camera_transform(metadata["camera_position_world"],metadata["camera_rotation_wxyz"])
    if not np.allclose(recorded,optical,atol=1e-6,rtol=0):
        raise ValueError("Capture optical camera extrinsics mismatch")
