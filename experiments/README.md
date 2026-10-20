# Evaluation plan

**Currently there is ONE documented successful fixed-scene run. Do not claim multi-scene success yet.** Reported metrics: simulated object position error 1.414 mm, approx TCP 6.86 mm, downward axis error 0.77°, execution success; 77 motion trajectory points. No robust averages or rates available.

For each independent trial change cube pose and re-export RGB-D, mask, known model geometry, camera calibration and simulation ground truth, **then rerun** FoundationPose and planning. Record run ID, seed, object GT/estimated XYZ, pose error, IK success, planning success, execution status, TF TCP XYZ/error, planning/execution time, video, and failure reason. Restore all collision objects and ensure no stale pre-grasp JSON before each new trial. Use `results_template.csv`; do not populate fabricated measurements.
