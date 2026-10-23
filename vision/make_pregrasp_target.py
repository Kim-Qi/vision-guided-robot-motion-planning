import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiment_io import parse_experiment, load_metadata, build_pregrasp

paths = parse_experiment("Generate a position-only pregrasp; never executes motion")
metadata = load_metadata(paths)
T = np.load(paths.results / "object_pose_panda_link0.npy", allow_pickle=False)
result = build_pregrasp(T, metadata)
path = paths.results / "pregrasp_target.json"
path.write_text(json.dumps(result, indent=2))
print("Object center:", np.round(result["object_center_m"], 5))
print("Pregrasp target:", np.round(result["pregrasp_position_m"], 5))
print("PREGRASP TARGET READY:", path)
