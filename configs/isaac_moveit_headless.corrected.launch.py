"""Compatibility entry for the scene-measured pre-grasp launch (no fixed base offset)."""
from pathlib import Path
from runpy import run_path
generate_launch_description=run_path(str(Path(__file__).with_name("isaac_moveit_headless.launch.py")))["generate_launch_description"]
