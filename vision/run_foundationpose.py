import os
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import trimesh
import nvdiffrast.torch as dr

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
from Utils import set_logging_format, set_seed

ROOT = Path(os.environ.get("ROBOT_PROJECT_ROOT", "/root/autodl-tmp/robot_vision_motion"))
DATA = ROOT / "vision_data"
OUT = ROOT / "vision_results"
OUT.mkdir(parents=True, exist_ok=True)

set_logging_format()
set_seed(0)

print("STEP 1: Loading RGB-D data", flush=True)

rgb = cv2.imread(str(DATA / "rgb.png"))
assert rgb is not None, "RGB image not found"
rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
rgb = np.ascontiguousarray(rgb, dtype=np.uint8)

depth = np.load(DATA / "depth.npy").astype(np.float32)
K = np.load(DATA / "intrinsics.npy").astype(np.float32)

mask_img = cv2.imread(str(DATA / "mask.png"), cv2.IMREAD_GRAYSCALE)
assert mask_img is not None, "Mask image not found"
mask = mask_img > 127

assert K.shape == (3, 3), f"Invalid K shape: {K.shape}"
assert rgb.shape[:2] == depth.shape == mask.shape, (
    f"Shape mismatch: {rgb.shape}, {depth.shape}, {mask.shape}"
)
assert np.isfinite(K).all()
assert K[0, 0] > 0 and K[1, 1] > 0

depth[~np.isfinite(depth)] = 0
depth[depth < 0] = 0

valid = mask & (depth > 0.001)

print("RGB:", rgb.shape, flush=True)
print("DEPTH:", depth.shape, flush=True)
print("K:\n", K, flush=True)
print("MASK pixels:", int(mask.sum()), flush=True)
print("Valid object depth pixels:", int(valid.sum()), flush=True)

assert valid.sum() > 100, "Insufficient valid depth inside mask"

object_depth = depth[valid]
print(
    "Object depth (m), p5/median/p95:",
    np.percentile(object_depth, [5, 50, 95]),
    flush=True,
)

# Mask 覆盖图：红色区域为目标掩码
overlay = rgb.copy()
overlay[mask] = (
    0.55 * overlay[mask] + 0.45 * np.array([255, 0, 0])
).astype(np.uint8)
cv2.imwrite(
    str(OUT / "mask_overlay.png"),
    cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR),
)

print("STEP 2: Loading CAD mesh", flush=True)

mesh = trimesh.load(
    str(DATA / "object.obj"),
    force="mesh",
    process=False,
)

# 与相机内参 K 保持一致，避免 PyTorch float32/float64 冲突
mesh.vertices = np.asarray(mesh.vertices, dtype=np.float32)
mesh.vertex_normals = np.asarray(
    mesh.vertex_normals, dtype=np.float32
)

assert isinstance(mesh, trimesh.Trimesh)
assert len(mesh.vertices) > 0 and len(mesh.faces) > 0

print("Mesh bounds:", mesh.bounds, flush=True)
print("Mesh faces:", len(mesh.faces), flush=True)

print("STEP 3: Loading NVIDIA models", flush=True)

scorer = ScorePredictor()
refiner = PoseRefinePredictor()
glctx = dr.RasterizeCudaContext()

print("STEP 4: Initializing FoundationPose", flush=True)

est = FoundationPose(
    model_pts=mesh.vertices,
    model_normals=mesh.vertex_normals,
    mesh=mesh,
    scorer=scorer,
    refiner=refiner,
    glctx=glctx,
    debug=1,
    debug_dir=str(OUT / "debug"),
)

print("STEP 5: Running 6D pose estimation", flush=True)

pose = est.register(
    K=K,
    rgb=rgb,
    depth=depth,
    ob_mask=mask,
    iteration=5,
)

assert pose.shape == (4, 4)
assert np.isfinite(pose).all()

np.savetxt(OUT / "object_pose_camera.txt", pose, fmt="%.8f")
np.save(OUT / "object_pose_camera.npy", pose)

print("\n===== FOUNDATIONPOSE RESULT =====", flush=True)
print("Object pose in camera frame:", flush=True)
print(pose, flush=True)
print("Translation (meters):", pose[:3, 3], flush=True)

# 根据估计位姿，将 CAD 模型的八个角点投影回图像
corners = np.array([
    [x, y, z]
    for x in [-0.06, 0.06]
    for y in [-0.06, 0.06]
    for z in [-0.06, 0.06]
], dtype=np.float32)

corners_cam = corners @ pose[:3, :3].T + pose[:3, 3]
projection_valid = corners_cam[:, 2] > 0.001

vis = rgb.copy()

if projection_valid.all():
    pixels = corners_cam @ K.T
    pixels = np.rint(pixels[:, :2] / pixels[:, 2:3]).astype(int)

    for i in range(8):
        for axis_bit in [1, 2, 4]:
            j = i ^ axis_bit
            if i < j:
                cv2.line(
                    vis,
                    tuple(pixels[i]),
                    tuple(pixels[j]),
                    (0, 255, 0),
                    2,
                )

    for p in pixels:
        cv2.circle(vis, tuple(p), 4, (255, 255, 0), -1)
else:
    print("WARNING: Some projected mesh corners behind camera", flush=True)

cv2.imwrite(
    str(OUT / "pose_overlay.png"),
    cv2.cvtColor(vis, cv2.COLOR_RGB2BGR),
)

print("POSE FILE:", OUT / "object_pose_camera.txt", flush=True)
print("VIS FILE:", OUT / "pose_overlay.png", flush=True)
print("FOUNDATIONPOSE 6D ESTIMATION SUCCESS", flush=True)
