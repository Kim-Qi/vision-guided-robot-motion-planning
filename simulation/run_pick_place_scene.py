"""40 mm physics-only entry; reuses the original scene and never subscribes to robot commands."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

if __name__ == "__main__":
    sys.argv = [str(ROOT / "simulation/run_franka_vision.py"),
                "--scene-config", str(ROOT / "configs/pick_place_40mm.json"), *sys.argv[1:]]
    runpy.run_path(sys.argv[0], run_name="__main__")
