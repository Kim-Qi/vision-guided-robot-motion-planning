from pathlib import Path
import runpy
import sys
ROOT=Path(__file__).resolve().parents[1]
if __name__ == "__main__":
    sys.argv=[str(ROOT/"simulation/run_franka_vision.py"), "--scene-config",
              str(ROOT/"configs/multi_object_pick_place.json"), *sys.argv[1:]]
    runpy.run_path(sys.argv[0],run_name="__main__")
