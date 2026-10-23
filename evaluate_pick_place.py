"""Independent acceptance using recorded physical body telemetry, never MoveIt attachments."""
import argparse
import json
from pathlib import Path

import numpy as np

from task_io import atomic_json, relative_pose


def evaluate(run):
    run=Path(run)
    report=json.loads((run/"task_result.json").read_text())
    if not report.get("success"):
        return {"success":False,"reason":report.get("failure_reason","Task did not succeed"),
                "failure_stage":report.get("failure_stage"),"task_report_success":False}
    metadata=json.loads((run/"vision_data/metadata.json").read_text())
    detections=json.loads((run/"vision_results/detections.json").read_text())
    initial=json.loads((run/"initial_state.json").read_text())
    with (run/"telemetry.jsonl").open() as stream:
        samples=[json.loads(line) for line in stream]
    events={event["state"]:event for event in report["events"]}
    required=["OPEN","PREGRASP","APPROACH","CLOSE","TEST_LIFT","LIFT","TRANSFER",
              "PLACE","RELEASE","RETREAT","VERIFY","SUCCESS"]
    if not all(name in events for name in required):
        return {"success":False,"reason":"Required stages missing","task_report_success":report["success"]}
    target=report["target"]
    hold=[s for s in samples if events["TEST_LIFT"]["wall_time"]<=s["wall_time"]<events["RELEASE"]["wall_time"]]
    if not hold:return {"success":False,"reason":"No physical holding samples"}
    reference=relative_pose(hold[0],target)
    slips=[float(np.linalg.norm(relative_pose(s,target)[:3,3]-reference[:3,3])) for s in hold]
    rotations=[]
    for s in hold:
        rotation=relative_pose(s,target)[:3,:3]@reference[:3,:3].T
        rotations.append(float(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
    final=samples[-1]
    final_window=[s for s in samples if final["wall_time"]-1<=s["wall_time"]]
    point=np.asarray(final["objects"][target]["position_world_m"])
    base=np.asarray(metadata["robot_base_transform_world"])
    base_point=np.linalg.solve(base,np.r_[point,1])[:3]
    place=np.asarray(report["place_xy_base_m"])
    error=float(np.linalg.norm(base_point[:2]-place))
    initial_point=np.asarray(initial["objects"][target]["position_world_m"])
    max_lift=max(s["objects"][target]["position_world_m"][2]-initial_point[2] for s in hold)
    separation=float(np.linalg.norm(np.asarray(final["tcp_transform_world"])[:3,3]-point))
    speed=max(float(np.linalg.norm(s["objects"][target]["linear_velocity_m_s"])) for s in final_window)
    drift=max(float(np.linalg.norm(np.asarray(s["objects"][target]["position_world_m"])-point)) for s in final_window)
    non_target={name:max(float(np.linalg.norm(np.asarray(s["objects"][name]["position_world_m"])-
                      initial["objects"][name]["position_world_m"])) for s in samples
                if s["wall_time"]>=events["OPEN"]["wall_time"])
                for name in initial["objects"] if name!=target}
    vision=np.asarray(detections["objects"][target]["T_world_object"])[:3,3]
    distances={name:float(np.linalg.norm(vision-np.asarray(data["ground_truth"]["position_world_m"])))
               for name,data in metadata["objects"].items()}
    correct=min(distances,key=distances.get)==target
    size=metadata["target_size_m"]
    rotation=np.asarray(final["objects"][target]["transform_world"])[:3,:3]
    bottom=point[2]-size/2*np.abs(rotation[2]).sum()
    support_error=abs(bottom-metadata["support_surface"]["top_z_m"])
    params=metadata["physics_config"]["task_parameters"]
    stable_span=final_window[-1]["wall_time"]-final_window[0]["wall_time"]
    hold_span=hold[-1]["wall_time"]-hold[0]["wall_time"]
    success=(report["success"] and correct and max_lift>=0.035
             and len(final_window)>=5 and stable_span>=0.8 and hold_span>=2.0
             and max(slips)<=params["slip_tolerance_m"] and max(rotations)<=0.3
             and error<=params["place_tolerance_m"] and separation>=0.08
             and speed<=0.01 and drift<=0.002 and support_error<=0.003
             and max(non_target.values(),default=0)<=params["non_target_tolerance_m"])
    return {"success":bool(success),"source":"Isaac dynamic-body telemetry; independent of planning attachment",
            "target_selection_correct":correct,"target_position_error_m":distances[target],
            "maximum_lift_m":float(max_lift),"maximum_relative_translation_slip_m":max(slips),
            "maximum_relative_angle_rad":max(rotations),"final_place_error_m":error,
            "final_world_position_m":point.tolist(),"final_support_bottom_error_m":float(support_error),
            "final_max_speed_m_s":speed,"final_position_drift_m":drift,
            "tcp_object_separation_m":separation,"non_target_max_displacement_m":non_target,
            "holding_samples":len(hold),"holding_duration_s":hold_span,
            "final_stability_samples":len(final_window),"final_stability_span_s":stable_span,
            "task_seconds":report["task_seconds"]}


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-dir",type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(args.run_dir)
    atomic_json(args.run_dir/"acceptance.json",result)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result["success"] else 1)
