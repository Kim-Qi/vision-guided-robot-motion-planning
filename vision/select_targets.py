"""Distinct-color instance selection + CAD-based FoundationPose, not open vocabulary detection."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiment_io import ExperimentPaths, load_metadata, cube_corners
from scene_frames import load_scene_frames
from task_io import atomic_json


def color_mask(rgb, color):
    index = {"red": 0, "green": 1, "blue": 2}[color]
    values = rgb.astype(np.float32)
    mask = values[:, :, index] > 55
    for other in set(range(3))-{index}:
        mask &= values[:, :, index] > 1.4*values[:, :, other]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if count <= 1:
        raise ValueError(f"No {color} component")
    order = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1]+1
    if stats[order[0], cv2.CC_STAT_AREA] < 100:
        raise ValueError(f"Insufficient {color} pixels")
    if len(order)>1 and stats[order[1],cv2.CC_STAT_AREA] > stats[order[0],cv2.CC_STAT_AREA]*0.5:
        raise ValueError(f"Ambiguous multiple {color} objects")
    return labels == order[0]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-dir",required=True,type=Path)
    parser.add_argument("--target",default="red_cube")
    parser.add_argument("--prepare-only",action="store_true")
    args=parser.parse_args()
    paths=ExperimentPaths(args.run_dir.resolve())
    metadata=load_metadata(paths)
    if args.target not in metadata["objects"]:
        raise ValueError("Unknown target ID")
    rgb=cv2.cvtColor(cv2.imread(str(paths.data/"rgb.png")),cv2.COLOR_BGR2RGB)
    depth=np.load(paths.data/"depth.npy").astype(np.float32)
    depth[~np.isfinite(depth)]=0
    K=np.load(paths.data/"intrinsics.npy").astype(np.float32)
    _,base,camera=load_scene_frames(paths.metadata)
    paths.results.mkdir(exist_ok=True)
    masks={}
    meshes={}
    for name,item in metadata["objects"].items():
        mask=color_mask(rgb,item["color"])
        if np.count_nonzero(mask & (depth>0.001)) < 100:
            raise ValueError(f"Invalid target depth: {name}")
        cv2.imwrite(str(paths.data/f"mask_{name}.png"),mask.astype(np.uint8)*255)
        mesh=trimesh.creation.box(extents=[item["size_m"]]*3)
        mesh.vertices=np.asarray(mesh.vertices,dtype=np.float32)
        mesh.vertex_normals=np.asarray(mesh.vertex_normals,dtype=np.float32)
        mesh.export(paths.data/f"{name}.obj")
        masks[name]=mask
        meshes[name]=mesh
    if args.prepare_only:
        print("Prepared masks and CAD for",list(masks))
        return
    # No simulation ground truth is used in any pose input below.
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    import nvdiffrast.torch as dr
    from Utils import set_seed, set_logging_format
    set_seed(0)
    set_logging_format()
    scorer,refiner,context=ScorePredictor(),PoseRefinePredictor(),dr.RasterizeCudaContext()
    detections={}
    overlay=rgb.copy()
    for name,mesh in meshes.items():
        started=time.monotonic()
        est=FoundationPose(model_pts=mesh.vertices,model_normals=mesh.vertex_normals,mesh=mesh,
                           scorer=scorer,refiner=refiner,glctx=context,debug=0,
                           debug_dir=str(paths.results/f"debug_{name}"))
        pose=est.register(K=K,rgb=rgb,depth=depth,ob_mask=masks[name],iteration=5)
        world=camera@pose
        base_pose=np.linalg.solve(base,world)
        detections[name]={"color":metadata["objects"][name]["color"],
                          "size_m":metadata["objects"][name]["size_m"],
                          "T_camera_object":pose.tolist(),"T_world_object":world.tolist(),
                          "T_base_object":base_pose.tolist(),"inference_seconds":time.monotonic()-started,
                          "mask_pixels":int(masks[name].sum())}
        np.save(paths.results/f"{name}_camera.npy",pose)
        corners=cube_corners(detections[name]["size_m"])
        points=corners@pose[:3,:3].T+pose[:3,3]
        uv=points@K.T
        uv=np.rint(uv[:,:2]/uv[:,2:]).astype(int)
        for i in range(8):
            for bit in [1,2,4]:
                if i<(i^bit):cv2.line(overlay,tuple(uv[i]),tuple(uv[i^bit]),(255,255,0),1)
        cv2.putText(overlay,name,tuple(uv.min(axis=0)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        print("POSE",name,base_pose[:3,3],flush=True)
    atomic_json(paths.results/"detections.json",{"experiment_id":paths.root.name,"target":args.target,
                "method":"color-component mask + CAD FoundationPose", "objects":detections})
    cv2.imwrite(str(paths.results/"multi_pose_overlay.png"),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))


if __name__=="__main__":main()
