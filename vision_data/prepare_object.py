import os
from pathlib import Path
import numpy as np
from PIL import Image
import trimesh

root = Path("/root/autodl-tmp/robot_vision_motion/vision_data")

rgb = np.asarray(Image.open(root / "rgb.png").convert("RGB"))

# 当前测试场景使用红色立方体，利用颜色生成初始 Mask
r = rgb[:, :, 0].astype(np.int16)
g = rgb[:, :, 1].astype(np.int16)
b = rgb[:, :, 2].astype(np.int16)

mask = (
    (r > g * 1.35)
    & (r > b * 1.35)
    & (r > 65)
)

Image.fromarray((mask * 255).astype(np.uint8)).save(
    root / "mask.png"
)

# 与仿真目标尺寸一致，中心位于物体局部原点
mesh = trimesh.creation.box(extents=[0.12, 0.12, 0.12])
mesh.export(root / "object.obj")

print("MASK pixels:", int(mask.sum()))
print("MESH bounds:", mesh.bounds)
print("PREPARATION SUCCESS")
