# Reproduction

## Multi-object task

Prepare the external dependencies in [environment.md](environment.md), copy/edit `.env.example` to a private `.env`, then from the repository root:

```bash
set -a
source .env
set +a
python3 -B planning/run_pick_place.py --target red_cube --place-x 0.40 --place-y 0.25 --record
```

Use `blue_cube` or `green_cube` to select another known-color object. Each run starts all three objects afresh and creates a NEW `experiments/pick_place_DATE_TIME/`. An explicit `--run-dir experiments/new_name` must not already exist. Place XY is in `panda_link0`, meters; support height and object size determine descent height. Domain 47/localhost-only is the default; `--ros-domain` supports isolated domains 40..99. Global `--timeout` defaults to 1000 seconds; individual stages also have bounds. All trials are recorded, including failures.

The entry checks configured dependency paths and conflicts before starting its own processes. It never terminates arbitrary pre-existing simulations/controllers. Its cleanup applies only to process groups it created. Do not use this pipeline with real hardware.

Artifacts include `scene_config.json`, RGB-D and measured calibration under `vision_data/`, per-object masks/CAD/poses, `manifest.json`, controller snapshot, `task_result.json`, `acceptance.json`, actual `telemetry.jsonl`, logs, and `videos/pick_place.mp4` plus `ffprobe.json`. Final exit code 0 requires action/state-machine success, video finalization and independent physics checks.

CPU-only re-evaluation of a new local run:

```bash
python3 -B evaluate_pick_place.py --run-dir experiments/YOUR_NEW_RUN
```

For the published historical runs, download/extract `pick_place_evidence.tar.gz` from the release-assets handoff when available; it restores original telemetry under the same `experiments/verified/` paths as the compact JSON records. Upload is pending authentication; [DEMONSTRATION.md](DEMONSTRATION.md) states current availability. Then:

```bash
python3 -B evaluate_pick_place.py --run-dir experiments/verified/multi_red_016
```

This updates only derived acceptance JSON. Verify telemetry SHA256 against each `telemetry.sha256` before relying on external downloads. The historical physical-source manifests refer to the frozen runtime, not the path-adapted publisher script.

## Original 120 mm pre-grasp flow

The baseline is retained as a separate experiment. Its temporary 58.4 mm TCP check is historical and is not the 103.4 mm grasp-center model. Do not substitute one task's target JSON or TF into another session. Stop/close your own previous simulated task through its normal interface before deliberately starting a different flow; these commands do not kill existing processes.

After provisioning dependencies and loading your private `.env`:

1. In an Isaac environment terminal, start `python -u simulation/run_franka_vision.py` without `--scene-config` or `--run-dir`. Configure its ROS bridge library environment as required by your Isaac installation. Wait for `VISION: DATA EXPORTED`; the default 120 mm branch records a full measured base transform and optical camera metadata.
2. In a ROS terminal, source `${ROS_SETUP:-/opt/ros/humble/setup.bash}` and `${ISAAC_ROS_WS:-$PWD/third_party/IsaacSim_ros_5.1/humble_ws}/install/setup.bash`. Use the same ROS domain as the simulator. Launch the packaged file AFTER capture:

   ```bash
   ros2 launch "$PWD/configs/isaac_moveit_headless.launch.py" ros2_control_hardware_type:=isaac use_sim_time:=true
   ```

   `ROBOT_SCENE_METADATA` may select a capture; default is this repository's `vision_data/metadata.json`. The old `configs/isaac_moveit_headless.corrected.launch.py` filename delegates to the measured launch. Never patch an assumed zero or -0.64 m offset into an upstream launch. Full TF must match actual scene measurements.
3. `bash scripts/prepare_target.sh` generates a size-matched mask/CAD, calls official FoundationPose using the configured environment, transforms the result and creates `vision_results/pregrasp_target.json` with `execute=false`.
4. `bash scripts/plan_only.sh` adds collision geometry and requests IK/planning without execution. Inspect its return codes and the target JSON.
5. Only for deliberate simulated execution, with the correct current scene, run the ROS Python interpreter on `planning/execute_vertical_pregrasp.py --execute`; the legacy `scripts/record_vertical_demo.sh` retains the recording workflow.

The accepted 120 mm runtime source/history is protected and has CPU numerical-parity coverage. It was not re-executed on a GPU during this release task. Legacy numeric examples remain historical samples, not inputs for a newly captured multi-object task.

## CPU checks

With CPU image/geometry dependencies available:

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
```

SDK camera-source and USD-authoring tests are optional if those installed libraries are unavailable. Tests never import SimulationApp, run pretrained inference or start ROS/GPU work. [INTERVIEW_CODE_GUIDE.md](INTERVIEW_CODE_GUIDE.md) lists real code entry points and a reading order.
