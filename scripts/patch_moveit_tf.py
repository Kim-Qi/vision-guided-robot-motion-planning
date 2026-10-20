#!/usr/bin/env python3
"""Safely update upstream Isaac MoveIt headless launch TF to match Isaac Sim base."""
import argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--launch-file",type=Path,required=True);a=p.parse_args()
path=a.launch_file; s=path.read_text()
anchor='name="static_transform_publisher_world_to_robot"'; assert anchor in s
start=s.index(anchor); end=s.index('parameters=',start); segment=s[start:end]
old='"0.0",\n            "-0.64",\n            "0.0"'
new='"0.0",\n            "0.0",\n            "0.0"'
if old in segment:
    backup=path.with_name(path.name+".before_base_tf_fix")
    if not backup.exists():backup.write_text(s)
    path.write_text(s[:start]+segment.replace(old,new,1)+s[end:]);print("PATCHED",path)
elif new in segment:print("ALREADY CORRECT",path)
else:raise SystemExit("Unknown TF values; do not edit launch file automatically")
