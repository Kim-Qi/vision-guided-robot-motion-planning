"""One foreground command orchestrates bounded, isolated virtual-robot experiments."""
import argparse
import hashlib
import shutil
from zoneinfo import ZoneInfo
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from task_io import atomic_json, validate_capture
from runtime_paths import RuntimePaths


def active_conflicts():
    conflicts=[]
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:
            fields=(proc/"cmdline").read_bytes().split(b"\0")
            command=[x.decode(errors="replace") for x in fields if x]
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if not command:continue
        executable=Path(command[0]).name
        if executable in ["move_group","ros2_control_node"]:
            conflicts.append((proc.name,command))
        elif executable.startswith("python") and any(Path(v).name in [
            "run_multi_object_scene.py","run_franka_vision.py","run_pick_place_scene.py"] for v in command[1:]):
            conflicts.append((proc.name,command))
    return conflicts


def ros_command(arguments,paths):
    return ["bash","-c",f"source {shlex.quote(str(paths.ros_setup))}\nsource {shlex.quote(str(paths.workspace_setup))}\nexec {shlex.join(arguments)}"]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--target",default="red_cube",choices=["red_cube","blue_cube","green_cube"])
    parser.add_argument("--place-x",type=float,default=0.4)
    parser.add_argument("--place-y",type=float,default=0.25)
    parser.add_argument("--run-dir",type=Path)
    parser.add_argument("--record",action="store_true",help="All trials are recorded, including failed trials")
    parser.add_argument("--timeout",type=float,default=1000)
    parser.add_argument("--ros-domain",type=int,default=47)
    args=parser.parse_args()
    if not 40<=args.ros_domain<=99:parser.error("Use an isolated ROS domain in 40..99")
    if not math.isfinite(args.timeout) or not 60<=args.timeout<=1800:
        parser.error("--timeout must be finite, between 60 and 1800 seconds")
    if not all(math.isfinite(v) for v in [args.place_x,args.place_y]):
        parser.error("Place coordinates must be finite")
    paths=RuntimePaths.from_environment()
    paths.validate()
    conflicts=active_conflicts()
    if conflicts:raise RuntimeError(f"Existing simulation/control processes; no duplicate started: {conflicts}")
    run=(args.run_dir or ROOT/"experiments"/f"pick_place_{datetime.now(ZoneInfo('Asia/Shanghai')):%Y%m%d_%H%M%S}").resolve()
    if ROOT/"experiments" not in run.parents or run.exists():
        raise ValueError("Choose a NEW --run-dir under experiments/")
    run.parent.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.ros_domain),ROS_LOCALHOST_ONLY="1",
             ROS_DISTRO="humble",RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
             ROS_LOG_DIR=str(run/"roslogs"),OMNI_KIT_ACCEPT_EULA="YES",
             ROBOT_SCENE_METADATA=str(run/"vision_data/metadata.json"),
             PICK_PLACE_CONTROLLER_CONFIG=str(run/"controllers_used.yaml"))
    processes=[]
    files=[]
    started=time.monotonic()
    deadline=started+args.timeout
    stage="START_SIMULATION"

    def launch(command,log,environment=env):
        handle=Path(log).open("x")
        files.append(handle)
        child=subprocess.Popen(command,cwd=ROOT,env=environment,stdout=handle,stderr=subprocess.STDOUT,
                               start_new_session=True)
        processes.append(child)
        return child

    def check_deadline():
        if time.monotonic()>deadline:raise TimeoutError(f"Global experiment timeout at {stage}")
        if (run/"simulation_failure.json").exists():
            raise RuntimeError((run/"simulation_failure.json").read_text())
        if simulation.poll() is not None:raise RuntimeError(f"Simulator exited unexpectedly: {simulation.returncode}")

    def wait_file(path):
        while not path.exists():
            check_deadline()
            time.sleep(0.5)

    def wait_child(child,seconds):
        end=time.monotonic()+seconds
        while child.poll() is None:
            check_deadline()
            if time.monotonic()>end:raise TimeoutError(f"Stage timeout: {stage}")
            time.sleep(0.5)
        if child.returncode!=0:raise RuntimeError(f"{stage} worker failed: {child.returncode}")

    code=0
    simulation=None
    try:
        sim_env=dict(env,LD_LIBRARY_PATH=str(paths.bridge_lib))
        simulation=launch(paths.python_command(paths.isaac_env)+["simulation/run_multi_object_scene.py",
            "--run-dir",str(run)],str(run)+".simulation.log",sim_env)
        wait_file(run/"capture_ready.json")
        validate_capture(run)
        stage="START_MOVEIT"
        shutil.copy2(ROOT/"configs/pick_place_controllers.yaml",run/"controllers_used.yaml")
        code_files=["simulation/run_franka_vision.py","simulation/multi_object_runtime.py",
                    "simulation/pick_place_physics.py","planning/run_pick_place.py",
                    "planning/pick_place_controller.py","planning/pick_place.launch.py",
                    "vision/select_targets.py","scene_frames.py","experiment_io.py","task_io.py",
                    "evaluate_pick_place.py","runtime_paths.py","configs/multi_object_pick_place.json",
                    "configs/pick_place_controllers.yaml","configs/pick_place.urdf.xacro"]
        def digest(path):
            result=hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda:stream.read(1024*1024),b""):result.update(block)
            return result.hexdigest()
        atomic_json(run/"manifest.json",{
            "created_at":datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
            "target":args.target,"place_xy_base_m":[args.place_x,args.place_y],
            "source_sha256":{name:digest(ROOT/name) for name in code_files},
            "weights_sha256":{"FoundationPose/"+str(path.relative_to(paths.foundation_root)):digest(path)
                              for path in (paths.foundation_root/"weights").glob("*/*.pth")}})
        launch(ros_command(["ros2","launch",str(ROOT/"planning/pick_place.launch.py")],paths),run/"moveit.log")
        stage="PERCEIVE"
        atomic_json(run/"task_status.json",{"state":stage,"target":args.target})
        fp_env=dict(env,PYTHONPATH=str(paths.foundation_root))
        perception=launch(paths.python_command(paths.foundation_env)+["vision/select_targets.py",
            "--run-dir",str(run),"--target",args.target],run/"perception.log",fp_env)
        wait_child(perception,300)
        stage="PICK_PLACE"
        task=launch(ros_command([str(paths.ros_python),"-u",str(ROOT/"planning/pick_place_controller.py"),
            "--run-dir",str(run),"--target",args.target,"--place-x",str(args.place_x),
            "--place-y",str(args.place_y)],paths),run/"task.log")
        wait_child(task,500)
    except Exception as error:
        code=1
        print(f"FAILED {stage}: {error}",flush=True)
        if run.exists():
            atomic_json(run/"orchestration_failure.json",{"stage":stage,"error":str(error)})
            if not (run/"task_result.json").exists():
                atomic_json(run/"task_result.json",{"success":False,"failure_stage":stage,
                            "failure_reason":str(error),"target":args.target})
    finally:
        if simulation is not None and simulation.poll() is None and run.exists():
            time.sleep(2)
            (run/"STOP_SIMULATION").touch()
            try:simulation.wait(timeout=40)
            except subprocess.TimeoutExpired:pass
        # Only terminate our own newly created process groups. Never pkill global services.
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL)
                        child.wait()
        for handle in files:handle.close()
        if code==0:
            if simulation.returncode!=0 or not (run/"videos/ffprobe.json").exists():
                code=1
                atomic_json(run/"orchestration_failure.json",{"stage":"VIDEO_FINALIZATION",
                            "error":"Simulator or verified video finalization failed"})
            else:
                from evaluate_pick_place import evaluate
                try:
                    acceptance=evaluate(run)
                    atomic_json(run/"acceptance.json",acceptance)
                    if not acceptance["success"]:code=1
                except Exception as error:
                    code=1
                    atomic_json(run/"acceptance.json",{"success":False,"error":str(error)})
        if run.exists():
            atomic_json(run/"orchestration.json",{"elapsed_seconds":time.monotonic()-started,
                        "exit_code":code,"ros_domain":args.ros_domain,"run_dir":str(run),
                        "process_returncodes":[p.returncode for p in processes]})
        print("RUN",run,flush=True)
    raise SystemExit(code)


if __name__=="__main__":main()
