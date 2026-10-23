from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiment_io import parse_experiment, load_metadata, cube_corners, cube_size
paths = parse_experiment("Project estimated cube using capture dimensions")
metadata = load_metadata(paths)
DATA = paths.data
OUT = paths.results

rgb = cv2.imread(str(DATA / "rgb.png"))
mask = cv2.imread(str(DATA / "mask.png"), 0)
pose = np.load(OUT / "object_pose_camera.npy")
K = np.load(DATA / "intrinsics.npy")

assert rgb is not None and mask is not None
assert pose.shape == (4, 4)
assert K.shape == (3, 3)

# 红色轮廓：实际物体 Mask
contours, _ = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)
cv2.drawContours(rgb, contours, -1, (0, 0, 255), 2)

# 绿色线框：FoundationPose 预测的立方体
corners = cube_corners(cube_size(metadata))

points = corners @ pose[:3, :3].T + pose[:3, 3]

assert np.all(points[:, 2] > 0), "预测物体位于相机后方"

uv = points @ K.T
uv = np.round(uv[:, :2] / uv[:, 2:3]).astype(int)

for i in range(8):
    for bit in [1, 2, 4]:
        j = i ^ bit
        if i < j:
            cv2.line(
                rgb,
                tuple(uv[i]),
                tuple(uv[j]),
                (0, 255, 0),
                2,
            )

# 黄色点：预测物体中心
center = pose[:3, 3]
center_px = K @ center
center_px = np.round(center_px[:2] / center_px[2]).astype(int)

cv2.circle(rgb, tuple(center_px), 5, (0, 255, 255), -1)

output = OUT / "pose_validation.png"
cv2.imwrite(str(output), rgb)

ys, xs = np.nonzero(mask > 127)
print("Mask bounding box:", (xs.min(), ys.min(), xs.max(), ys.max()))
print("Predicted 3D box projection:")
print("u range:", (uv[:, 0].min(), uv[:, 0].max()))
print("v range:", (uv[:, 1].min(), uv[:, 1].max()))
print("Predicted center pixel:", center_px)
print("Saved:", output)
print("POSE VALIDATION IMAGE READY")
