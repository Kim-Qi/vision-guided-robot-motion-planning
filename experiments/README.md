# Actual experiment records

`multi_object_summary.json` contains three verified same-runtime-version demonstrations and two retained failed examples. `verified/` and `failures/` include compact real metadata, state-machine events, estimated poses, actual measured errors, initial state, sanitized trace and source/config fingerprints. No raw RGB-D, model weights, caches or environments are included.

Original runtime source manifests refer to the frozen accepted runtime; `docs/ACCEPTED_CORE_SHA256.json` identifies unchanged core sources in this publication tree. `export_provenance.json` records original/published SHA256 values. The only JSON transformation is replacing the orchestration `run_dir` with its publication-relative path; video-probe filenames and textual runtime paths are also made portable. No measured coordinates, errors or task outcomes are changed.

Full original physical telemetry is packaged separately in `pick_place_evidence.tar.gz` with the three videos as release assets. Their upload is pending authentication. Each compact run contains its original telemetry SHA256. Extract the evidence archive into the repository to independently run `evaluate_pick_place.py`; compact records alone do not pretend to contain missing telemetry.

All three final targets were visually reviewed by the user. Each was the only selected target in a scene containing all three cubes; these are separate reset runs, not a continuous clearing sequence or a statistical 100% success-rate benchmark.

Failures: green 011 stopped at TEST_LIFT after 14.6 mm relative slip; green 012 stopped at TRANSFER because actual TCP error 14.78 mm exceeded the 12 mm guard at place XY=(0.40,-0.30). Failed outcomes remain false. Bounds were not relaxed to turn those failures into successes.

The original pre-grasp template/history remains available in Git history and `results_template.csv`; its metrics must not be mixed with 40 mm physical grasping.
