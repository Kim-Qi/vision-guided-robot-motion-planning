import os
import json
from pathlib import Path
import numpy as np

root = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion"))
out = root / "vision_results"

T = np.load(out / "object_pose_panda_link0.npy")
assert T.shape == (4, 4)
assert np.isfinite(T).all()

center = T[:3, 3].copy()

# 物体顶部上方 12 cm；暂不使用对称立方体的不确定朝向
cube_size = 0.12
clearance = 0.12
target = center + [0.0, 0.0, cube_size / 2 + clearance]

result = {
    "frame_id": "panda_link0",
    "source": "FoundationPose object position",
    "object_center_m": center.tolist(),
    "pregrasp_position_m": target.tolist(),
    "position_only": True,
    "execute": False
}

path = out / "pregrasp_target.json"
path.write_text(json.dumps(result, indent=2))

print("Object center:", np.round(center, 5))
print("Pregrasp target:", np.round(target, 5))
print("PREGRASP TARGET READY:", path)
