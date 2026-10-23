"""CPU deployment checks; fake dependency files never execute a simulator."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from runtime_paths import RuntimePaths
from planning.run_pick_place import ros_command


class DeploymentTests(unittest.TestCase):
    def test_missing_conda_fails_with_actionable_error(self):
        with patch.dict(os.environ,{},clear=True),patch("shutil.which",return_value=None):
            with self.assertRaisesRegex(ValueError,"CONDA_EXE"):
                RuntimePaths.from_environment()

    def test_configurable_prefixes_and_quoted_ros_paths(self):
        with tempfile.TemporaryDirectory(prefix="deployment space ") as tmp:
            base=Path(tmp)
            env={"CONDA_EXE":str(base/"conda"),"ISAACSIM_ENV":str(base/"isaac"),
                 "FOUNDATIONPOSE_ENV":str(base/"fp"),"FOUNDATIONPOSE_ROOT":str(base/"FP source"),
                 "ROS_SETUP":str(base/"ros setup.bash"),"ISAAC_ROS_WS":str(base/"ros workspace"),
                 "ROS_PYTHON":str(base/"system python")}
            bridge=base/"isaac/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
            bridge.mkdir(parents=True)
            for path in [base/"conda",base/"isaac/bin/python",base/"fp/bin/python",base/"FP source/estimater.py",
                         base/"ros setup.bash",base/"ros workspace/install/setup.bash",base/"system python"]:
                path.parent.mkdir(parents=True,exist_ok=True);path.touch()
            with patch.dict(os.environ,env,clear=True):
                paths=RuntimePaths.from_environment();paths.validate()
                self.assertEqual(paths.bridge_lib,bridge)
                self.assertEqual(paths.python_command(paths.isaac_env),[str(base/"conda"),"run","--no-capture-output","-p",str(base/"isaac"),"python","-u"])
                command=ros_command(["printf","%s","literal $()"],paths)
                result=subprocess.run(command,capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertEqual(result.stdout,"literal $()")
                (base/"fp/bin/python").unlink()
                with self.assertRaisesRegex(ValueError,"Missing configured dependencies"):paths.validate()

    def test_cli_help_and_invalid_coordinates_do_not_need_dependencies(self):
        script=ROOT/"planning/run_pick_place.py"
        result=subprocess.run([sys.executable,"-B",str(script),"--help"],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn("--target",result.stdout)
        result=subprocess.run([sys.executable,"-B",str(script),"--place-x","nan"],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertIn("Place coordinates must be finite",result.stderr)


if __name__=="__main__":unittest.main(verbosity=2)
