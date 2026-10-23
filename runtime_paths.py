"""Deployment paths only; does not import Isaac Sim, ROS, or GPU libraries."""
from dataclasses import dataclass
import os
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent


def configured_path(name, default):
    return Path(os.environ.get(name, str(default))).expanduser().resolve()


@dataclass(frozen=True)
class RuntimePaths:
    conda: Path
    isaac_env: Path
    foundation_env: Path
    foundation_root: Path
    ros_setup: Path
    workspace_setup: Path
    ros_python: Path
    bridge_lib: Path

    @classmethod
    def from_environment(cls):
        conda=os.environ.get("CONDA_EXE") or shutil.which("conda")
        if not conda:
            raise ValueError("Set CONDA_EXE to an existing conda executable; see .env.example")
        isaac=configured_path("ISAACSIM_ENV",ROOT/".envs/isaacsim")
        bridge=os.environ.get("ISAAC_ROS_BRIDGE_LIB")
        candidates=sorted(isaac.glob("lib/python*/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"))
        if not bridge and len(candidates)!=1:
            raise ValueError("Set ISAAC_ROS_BRIDGE_LIB to the installed Isaac Sim Humble bridge lib directory")
        return cls(Path(conda).expanduser().resolve(),isaac,
                   configured_path("FOUNDATIONPOSE_ENV",ROOT/".envs/foundationpose"),
                   configured_path("FOUNDATIONPOSE_ROOT",ROOT/"third_party/FoundationPose"),
                   configured_path("ROS_SETUP","/opt/ros/humble/setup.bash"),
                   configured_path("ISAAC_ROS_WS",ROOT/"third_party/IsaacSim_ros_5.1/humble_ws")/"install/setup.bash",
                   configured_path("ROS_PYTHON","/usr/bin/python3"),
                   Path(bridge).expanduser().resolve() if bridge else candidates[0])

    def validate(self):
        required=[self.conda,self.isaac_env/"bin/python",self.foundation_env/"bin/python",
                  self.foundation_root/"estimater.py",self.ros_setup,self.workspace_setup,self.ros_python]
        missing=[str(p) for p in required if not p.is_file()]
        if not self.bridge_lib.is_dir():missing.append(str(self.bridge_lib))
        if missing:raise ValueError("Missing configured dependencies: "+", ".join(missing))

    def python_command(self, prefix):
        return [str(self.conda),"run","--no-capture-output","-p",str(prefix),"python","-u"]
