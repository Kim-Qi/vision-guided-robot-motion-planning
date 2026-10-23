"""CPU-only target-selection and telemetry acceptance regression tests."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vision.select_targets import color_mask
from task_io import atomic_json, read_state, relative_pose
from scene_frames import camera_transform


class MultiObjectTests(unittest.TestCase):
    def test_video_camera_or_mixed_batch_calibration_rejected(self):
        from task_io import validate_capture
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory)
            (run/"vision_data").mkdir()
            metadata={"experiment_id":run.name,"camera_frame":"camera_ros_frame",
                      "camera_position_world":[1.2,0.8,1.4],"camera_rotation_wxyz":[1,0,0,0]}
            extrinsics={"experiment_id":run.name,"parent_frame":"world","child_frame":"camera_ros_frame",
                        "translation":[1.2,0.8,1.4],"rotation_xyzw":[0,0,0,1]}
            atomic_json(run/"vision_data/metadata.json",metadata)
            atomic_json(run/"camera_extrinsics.json",extrinsics)
            validate_capture(run)
            extrinsics["translation"]=[6.2,5.8,3.2]
            atomic_json(run/"camera_extrinsics.json",extrinsics)
            with self.assertRaisesRegex(ValueError,"optical camera extrinsics mismatch"):validate_capture(run)
            extrinsics["experiment_id"]="older_batch"
            atomic_json(run/"camera_extrinsics.json",extrinsics)
            with self.assertRaisesRegex(ValueError,"batch mismatch"):validate_capture(run)

    def test_physical_objects_fit_support_and_gripper(self):
        from simulation.pick_place_physics import load_config, initial_position, support_top
        config=load_config(ROOT/"configs/multi_object_pick_place.json")
        self.assertEqual(len({item["id"] for item in config["objects"]}),3)
        self.assertLess(config["cube_size_m"],0.08)
        self.assertEqual(config["cube_mass_kg"],0.05)
        points=[]
        for item in config["objects"]:
            local=dict(config,cube_xy_world_m=item["xy"])
            point=np.asarray(initial_position(local))
            self.assertGreater(point[2]-config["cube_size_m"]/2,support_top(config))
            extent=np.asarray(config["support_dimensions_m"][:2])/2
            self.assertTrue(np.all(np.abs(point[:2]-config["support_center_world_m"][:2])+
                                   config["cube_size_m"]/2<extent))
            points.append(point)
        for i,p in enumerate(points):
            for q in points[i+1:]:self.assertGreater(np.linalg.norm(p-q),config["cube_size_m"]*2)

    def test_hand_has_independent_command_stream_and_one_mimic_driver(self):
        import xml.etree.ElementTree as ET
        tree=ET.parse(ROOT/"configs/pick_place.urdf.xacro")
        hardware=tree.find("ros2_control")
        self.assertEqual([j.attrib["name"] for j in hardware.findall("joint")],["panda_finger_joint1"])
        parameters={p.attrib["name"]:p.text for p in hardware.findall("hardware/param")}
        self.assertEqual(parameters["joint_commands_topic"],"isaac_gripper_commands")
        self.assertNotEqual(parameters["joint_commands_topic"],"isaac_joint_commands")
        self.assertEqual(parameters["joint_states_topic"],"isaac_joint_states")

    def test_three_colors_select_only_requested_component(self):
        rgb=np.zeros((100,160,3),dtype=np.uint8)
        rgb[10:30,10:30]=[220,20,20]
        rgb[40:60,60:80]=[20,220,20]
        rgb[60:80,110:130]=[20,20,220]
        masks=[color_mask(rgb,name) for name in ["red","green","blue"]]
        self.assertTrue(all(mask.sum()==400 for mask in masks))
        self.assertEqual(np.count_nonzero(masks[0]&masks[1]),0)
        self.assertTrue(masks[0][15,15])
        self.assertFalse(masks[0][50,70])

    def test_same_color_ambiguity_rejected(self):
        rgb=np.zeros((100,100,3),dtype=np.uint8)
        rgb[10:30,10:30]=[220,20,20]
        rgb[60:80,60:80]=[220,20,20]
        with self.assertRaisesRegex(ValueError,"Ambiguous"):
            color_mask(rgb,"red")

    def test_stale_telemetry_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            atomic_json(Path(directory)/"state.json",{"wall_time":time.time()-10})
            with self.assertRaisesRegex(RuntimeError,"stale"):
                read_state(directory)

    def test_relative_pose_preserved_under_shared_rigid_motion(self):
        hand=camera_transform([0.4,-0.2,0.3],[0,1,0,0])
        local=camera_transform([0,0,0.1034],[1,0,0,0])
        motion=camera_transform([0.1,0.4,0.1],[2**-.5,0,0,2**-.5])
        state={"hand_transform_world":hand.tolist(),"objects":{"red_cube":{"transform_world":(hand@local).tolist()}}}
        before=relative_pose(state,"red_cube")
        state["hand_transform_world"]=(motion@hand).tolist()
        state["objects"]["red_cube"]["transform_world"]=(motion@hand@local).tolist()
        np.testing.assert_allclose(relative_pose(state,"red_cube"),before,atol=1e-12)

    def test_independent_acceptance_rejects_action_success_without_lift(self):
        from evaluate_pick_place import evaluate
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory)
            (run/"vision_data").mkdir()
            (run/"vision_results").mkdir()
            states=["OPEN","PREGRASP","APPROACH","CLOSE","TEST_LIFT","LIFT","TRANSFER",
                    "PLACE","RELEASE","RETREAT","VERIFY","SUCCESS"]
            report={"success":True,"target":"red_cube","place_xy_base_m":[0.4,0.25],
                    "task_seconds":12,"events":[{"state":name,"wall_time":i} for i,name in enumerate(states)]}
            atomic_json(run/"task_result.json",report)
            objects={"red_cube":[0.5,-0.15,0.015],"blue_cube":[0.62,-0.35,0.015]}
            metadata={"robot_base_transform_world":np.eye(4).tolist(),"target_size_m":0.04,
                      "objects":{name:{"ground_truth":{"position_world_m":point}} for name,point in objects.items()},
                      "support_surface":{"top_z_m":-0.005},"physics_config":{"task_parameters":{
                          "slip_tolerance_m":0.012,"place_tolerance_m":0.015,"non_target_tolerance_m":0.01}}}
            atomic_json(run/"vision_data/metadata.json",metadata)
            red=np.eye(4);red[:3,3]=objects["red_cube"]
            atomic_json(run/"vision_results/detections.json",{"objects":{"red_cube":{"T_world_object":red.tolist()}}})
            samples=[]
            for t in np.arange(0,12.01,0.1):
                point=np.array(objects["red_cube"],dtype=float)
                if 4<=t<6:point[2]=0.015+(t-4)*0.1
                elif 6<=t<8:point=np.array([0.4,0.25,0.215-(t-6)*0.1])
                elif t>=8:point=np.array([0.4,0.25,0.015])
                hand=camera_transform(point+[0,0,0.1034+(0.15 if t>=9 else 0)],[0,1,0,0])
                tcp=hand@camera_transform([0,0,0.1034],[1,0,0,0])
                bodies={}
                for name,initial in objects.items():
                    position=point if name=="red_cube" else initial
                    pose=np.eye(4);pose[:3,3]=position
                    bodies[name]={"transform_world":pose.tolist(),"position_world_m":list(position),"linear_velocity_m_s":[0,0,0]}
                samples.append({"wall_time":float(t),"hand_transform_world":hand.tolist(),
                                "tcp_transform_world":tcp.tolist(),"objects":bodies})
            atomic_json(run/"initial_state.json",samples[0])
            (run/"telemetry.jsonl").write_text("".join(json.dumps(row)+"\n" for row in samples))
            self.assertTrue(evaluate(run)["success"])
            # The planner reports success but the actual object stays on the table.
            for row in samples:
                row["objects"]["red_cube"]=json.loads(json.dumps(samples[0]["objects"]["red_cube"]))
            (run/"telemetry.jsonl").write_text("".join(json.dumps(row)+"\n" for row in samples))
            self.assertFalse(evaluate(run)["success"])

    def test_unknown_target_has_no_mask_fallback(self):
        with self.assertRaises(KeyError):color_mask(np.zeros((20,20,3),dtype=np.uint8),"yellow")


if __name__=="__main__":unittest.main(verbosity=2)
