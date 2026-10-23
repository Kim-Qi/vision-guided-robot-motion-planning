# Publication validation — 2026-10-24 (Asia/Shanghai)

## Scope

No simulator, GPU inference, ROS controller or robot motion was started during publication. The original runtime tree remains unchanged. Its restorable source archive contains 84 files including the input-preparation source and six small original baseline calibration/pose fixtures, each SHA256-verified against the tar member bytes; `STABLE_RUNTIME_MANIFEST.json` is the portable inventory. The local handoff gives the archive location and archive SHA256. The stable name is a snapshot identifier, **not an already-published Git tag**.

Thirteen core sources/configurations match the accepted runtime byte-for-byte (`ACCEPTED_CORE_SHA256.json`). The public orchestration adds configurable dependency paths/preflight; legacy launch files are relocated with the same measured-transform logic, and CPU tests use committed tiny fixtures. No grasp physics, target generation, state-machine decisions, collision acceptance or task thresholds are changed. The publication tree has not been re-executed end-to-end on GPU.

The complete archive was also extracted into a new temporary directory: all 84 hashes matched, and the frozen coordinate/scene/multi-object CPU scripts ran successfully (6 + 12 + 9 tests, with the optional USD test skipped there). The legacy GPU entry tests were deliberately not discovered or run.

## Completed CPU checks

- `python -B -m unittest discover -s tests -p 'test_*.py' -v`: **30 discovered, 29 passed, 1 optional USD check skipped** in the existing FoundationPose environment, with `ISAACSIM_ENV` selecting the installed camera source for its CPU-only axis check.
- The skipped `SceneTests.test_usd_physics_authoring` separately **passed** using installed Isaac USD libraries on CPU; no SimulationApp was imported.
- Configurable environment/quoted-path handling, CLI help/invalid coordinates, camera/batch mismatch rejection, independent hand command stream and fake Action-success/no-object-lift rejection passed.
- Original physical telemetry extracted from the prepared evidence archive re-evaluated all three final runs with **exact metric dictionary equality** to their original acceptance JSON; both failed examples stayed false.
- Actual Xacro expansion resolved the Panda dependencies and exposed only the driver finger command. Constructed all four new/compatible launch descriptions without executing them; model/config dependencies resolved in the existing ROS workspace.
- Final syntax, JSON/XML, shell parsing, relative documentation links, interview-symbol anchors, publication file allowlist, private-path/secret scan and `git diff --check` are checked by `scripts/check_publication.py` and Git before commit.

SDK/camera checks can skip on a machine lacking those optional installations. Core image/geometry tests require only `requirements-cpu.txt`; it is not a full robotics dependency installer.

## Evidence and limitations

Three verified runs use the same accepted runtime version, actual object contact and independent physics acceptance. Errors are red 0.49/1.21 mm, blue 0.11/1.22 mm, green 0.85/3.26 mm (vision/place XY). There is no aggregate success-rate claim. Original telemetry is packaged outside Git, with source hashes and compact records in the repository. Media/evidence upload and remote PR creation are pending authentication.

Original 120 mm pre-grasp files/examples/video and Git history are retained. Numerical parity and nonidentity coordinate regression passed; no new baseline GPU run is claimed. This is a simulated research/interview project, not verified hardware control or arbitrary-workspace grasping.
