"""MoveIt/GripperCommand state machine for the isolated simulated robot only."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK, GetCartesianPath, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (RobotState, Constraints, JointConstraint, CollisionObject,
                             PlanningScene, PlanningSceneComponents, AttachedCollisionObject,
                             AllowedCollisionEntry)
from shape_msgs.msg import SolidPrimitive

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scene_frames import static_transform_arguments
from experiment_io import support_box
from task_io import atomic_json, read_state, relative_pose


def matrix_pose(matrix):
    args=static_transform_arguments(matrix,"base","object")
    values=dict(zip(args[::2],args[1::2]))
    pose=Pose()
    for axis in "xyz":setattr(pose.position,axis,float(values[f"--{axis}"]))
    for axis in "xyzw":setattr(pose.orientation,axis,float(values[f"--q{axis}"]))
    return pose


class PickPlace(Node):
    def __init__(self,args):
        super().__init__("vision_pick_place")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time",value=True)])
        self.args=args
        self.run_dir=args.run_dir.resolve()
        self.metadata=json.loads((self.run_dir/"vision_data/metadata.json").read_text())
        self.config=json.loads((self.run_dir/"scene_config.json").read_text())
        self.params=self.config["task_parameters"]
        self.detections=json.loads((self.run_dir/"vision_results/detections.json").read_text())
        if any(data.get("experiment_id") != self.run_dir.name
               for data in [self.metadata,self.detections]):
            raise ValueError("Perception/metadata experiment batch mismatch")
        if self.detections["target"]!=args.target:raise ValueError("Perception target mismatch")
        self.target=args.target
        self.base=np.asarray(self.metadata["robot_base_transform_world"])
        self.base_inverse=np.linalg.inv(self.base)
        self.joints=None
        self.joint_time=0
        self.create_subscription(JointState,"/joint_states",self.on_joints,10)
        self.ik=self.create_client(GetPositionIK,"/compute_ik")
        self.cart=self.create_client(GetCartesianPath,"/compute_cartesian_path")
        self.apply=self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.get_scene=self.create_client(GetPlanningScene,"/get_planning_scene")
        self.move=ActionClient(self,MoveGroup,"/move_action")
        self.execute=ActionClient(self,ExecuteTrajectory,"/execute_trajectory")
        self.gripper=ActionClient(self,GripperCommand,"/panda_hand_controller/gripper_cmd")
        self.start=time.monotonic()
        self.current="INIT"
        self.holding=False
        self.reference=None
        self.max_slip=0
        self.max_lift=0
        self.initial=read_state(self.run_dir)
        self.events=[]
        self.report={"target":self.target,"place_xy_base_m":[args.place_x,args.place_y],
                     "success":False,"grasp_success":False,"transport_success":False,
                     "place_success":False,"failure_stage":None,"failure_reason":None}

    def on_joints(self,msg):
        self.joints=msg
        self.joint_time=time.monotonic()

    def state(self,name,**details):
        self.current=name
        event={"state":name,"elapsed_s":time.monotonic()-self.start,"wall_time":time.time(),**details}
        self.events.append(event)
        atomic_json(self.run_dir/"task_status.json",dict(event,target=self.target))
        print("STATE",json.dumps(event),flush=True)

    def observe_hold(self):
        state=read_state(self.run_dir)
        relative=relative_pose(state,self.target)
        slip=float(np.linalg.norm(relative[:3,3]-self.reference[:3,3]))
        self.max_slip=max(self.max_slip,slip)
        height=state["objects"][self.target]["position_world_m"][2]
        initial_height=self.initial["objects"][self.target]["position_world_m"][2]
        self.max_lift=max(self.max_lift,height-initial_height)
        if slip>self.params["slip_tolerance_m"]:
            raise RuntimeError(f"Physical grip slipped {slip*1000:.1f} mm")

    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.03)
            if self.holding:self.observe_hold()

    def wait(self,future,timeout=60):
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError(f"{self.current}: ROS operation timed out")
            rclpy.spin_once(self,timeout_sec=0.03)
            if self.holding:self.observe_hold()
        result=future.result()
        if result is None:raise RuntimeError("ROS returned no result")
        return result

    def call(self,client,request):
        if not client.wait_for_service(timeout_sec=30):raise RuntimeError(f"Service unavailable: {client.srv_name}")
        return self.wait(client.call_async(request),30)

    def action(self,client,goal,timeout=120):
        if not client.wait_for_server(timeout_sec=30):raise RuntimeError("Action server unavailable")
        handle=self.wait(client.send_goal_async(goal),15)
        if not handle.accepted:raise RuntimeError("Action goal rejected")
        try:
            result=self.wait(handle.get_result_async(),timeout)
        except Exception:
            # Cancel only this goal; do not let a failed state continue moving.
            self.holding=False
            try:self.wait(handle.cancel_goal_async(),3)
            except Exception:pass
            raise
        return result.result

    def apply_scene(self,scene):
        scene.is_diff=True
        scene.robot_state.is_diff=True
        request=ApplyPlanningScene.Request()
        request.scene=scene
        if not self.call(self.apply,request).success:raise RuntimeError("Planning scene rejected")

    def cube(self,name,size,transform,frame="panda_link0"):
        obj=CollisionObject()
        obj.id=name
        obj.header.frame_id=frame
        obj.operation=CollisionObject.ADD
        primitive=SolidPrimitive()
        primitive.type=SolidPrimitive.BOX
        primitive.dimensions=[size]*3 if isinstance(size,(float,int)) else size
        obj.primitives=[primitive]
        obj.primitive_poses=[matrix_pose(transform)]
        return obj

    def scene_setup(self):
        scene=PlanningScene()
        for name,item in self.detections["objects"].items():
            scene.world.collision_objects.append(self.cube(name,item["size_m"],np.array(item["T_base_object"])))
        dimensions,transform=support_box(self.metadata)
        scene.world.collision_objects.append(self.cube("studio_floor",dimensions,transform))
        self.apply_scene(scene)
        request=GetPlanningScene.Request()
        request.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.acm=deepcopy(self.call(self.get_scene,request).scene.allowed_collision_matrix)

    def contacts(self,pairs):
        for a,b,enabled in pairs:
            for name in [a,b]:
                if name not in self.acm.entry_names:
                    self.acm.entry_names.append(name)
                    for row in self.acm.entry_values:row.enabled.append(False)
                    row=AllowedCollisionEntry()
                    row.enabled=[False]*len(self.acm.entry_names)
                    self.acm.entry_values.append(row)
            i,j=self.acm.entry_names.index(a),self.acm.entry_names.index(b)
            self.acm.entry_values[i].enabled[j]=enabled
            self.acm.entry_values[j].enabled[i]=enabled
        scene=PlanningScene()
        scene.allowed_collision_matrix=self.acm
        self.apply_scene(scene)

    def attach(self):
        hand=self.base_inverse@np.array(read_state(self.run_dir)["hand_transform_world"])
        object_pose=np.array(self.detections["objects"][self.target]["T_base_object"])
        obj=self.cube(self.target,self.metadata["target_size_m"],np.linalg.solve(hand,object_pose),"panda_hand")
        attached=AttachedCollisionObject()
        attached.link_name="panda_hand"
        attached.object=obj
        attached.touch_links=["panda_leftfinger","panda_rightfinger"]
        scene=PlanningScene()
        # MoveIt automatically removes the matching world object on attachment.
        # A simultaneous explicit REMOVE is processed afterwards and falsely fails.
        scene.robot_state.attached_collision_objects=[attached]
        self.apply_scene(scene)
        request=GetPlanningScene.Request()
        request.components.components=(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                                       | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        result=self.call(self.get_scene,request).scene
        attached_ids={item.object.id for item in result.robot_state.attached_collision_objects}
        world_ids={item.id for item in result.world.collision_objects}
        if self.target not in attached_ids or self.target in world_ids:
            raise RuntimeError("Planning attachment read-back failed")
        if not (set(self.detections["objects"])-{self.target}).issubset(world_ids):
            raise RuntimeError("Non-target collision objects were lost")

    def detach(self):
        scene=PlanningScene()
        remove=AttachedCollisionObject()
        remove.link_name="panda_hand"
        remove.object.id=self.target
        remove.object.operation=CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects=[remove]
        actual=self.base_inverse@np.array(read_state(self.run_dir)["objects"][self.target]["transform_world"])
        scene.world.collision_objects=[self.cube(self.target,self.metadata["target_size_m"],actual)]
        self.apply_scene(scene)

    def hand_target(self,tcp):
        hand=np.diag([1.,-1.,-1.,1.])
        offset=np.array(self.metadata["hand_to_tcp"])[:3,3]
        if abs(offset[2]-self.params["tcp_offset_m"])>0.001:
            raise ValueError("Measured tool_center differs from configured grasp TCP")
        hand[:3,3]=tcp-hand[:3,:3]@offset
        return matrix_pose(hand)

    def move_tcp(self,tcp,cartesian=False):
        tcp=np.asarray(tcp,dtype=float)
        target=self.hand_target(tcp)
        if cartesian:
            request=GetCartesianPath.Request()
            request.header.frame_id="panda_link0"
            request.group_name="panda_arm"
            request.link_name="panda_hand"
            request.start_state.joint_state=self.joints
            request.waypoints=[target]
            request.max_step=self.params["cartesian_step_m"]
            request.jump_threshold=2.0
            request.avoid_collisions=True
            request.max_velocity_scaling_factor=self.params["cartesian_velocity_scaling"]
            request.max_acceleration_scaling_factor=0.1
            response=self.call(self.cart,request)
            print("CARTESIAN",self.current,"fraction",response.fraction,"code",response.error_code.val,flush=True)
            if response.error_code.val!=1 or response.fraction<0.999:
                raise RuntimeError(f"Cartesian path incomplete: {response.fraction}")
            goal=ExecuteTrajectory.Goal()
            goal.trajectory=response.solution
            result=self.action(self.execute,goal)
        else:
            request=GetPositionIK.Request()
            request.ik_request.group_name="panda_arm"
            request.ik_request.ik_link_name="panda_hand"
            request.ik_request.pose_stamped.header.frame_id="panda_link0"
            request.ik_request.pose_stamped.pose=target
            request.ik_request.robot_state.joint_state=self.joints
            request.ik_request.avoid_collisions=True
            request.ik_request.timeout=Duration(sec=5)
            response=self.call(self.ik,request)
            if response.error_code.val!=1:raise RuntimeError(f"IK failed: {response.error_code.val}")
            solution=dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
            goal=MoveGroup.Goal()
            goal.request.group_name="panda_arm"
            goal.request.num_planning_attempts=5
            goal.request.allowed_planning_time=10.
            goal.request.max_velocity_scaling_factor=self.params["velocity_scaling"]
            goal.request.max_acceleration_scaling_factor=0.1
            constraints=Constraints()
            for i in range(1,8):
                constraint=JointConstraint()
                constraint.joint_name=f"panda_joint{i}"
                constraint.position=solution[constraint.joint_name]
                constraint.tolerance_above=constraint.tolerance_below=0.001
                constraint.weight=1.
                constraints.joint_constraints.append(constraint)
            goal.request.goal_constraints=[constraints]
            goal.request.start_state.is_diff=True
            goal.planning_options.plan_only=False
            result=self.action(self.move,goal)
        if result.error_code.val!=1:raise RuntimeError(f"MoveIt execution failed: {result.error_code.val}")
        self.spin(0.6)
        state=read_state(self.run_dir)
        actual=(self.base_inverse@np.array(state["tcp_transform_world"]))[:3,3]
        error=float(np.linalg.norm(actual-tcp))
        print("TCP",self.current,"target",tcp.tolist(),"actual",actual.tolist(),"error_m",error,flush=True)
        if error>0.012:raise RuntimeError(f"TCP tracking error {error}")

    def grip(self,position):
        goal=GripperCommand.Goal()
        goal.command.position=position
        result=self.action(self.gripper,goal,45)
        self.spin(0.8)
        joints=read_state(self.run_dir)["joint_positions"]
        fingers=[joints[f"panda_finger_joint{i}"] for i in [1,2]]
        print("GRIP",position,"actual",fingers,"reached",result.reached_goal,"stalled",result.stalled,flush=True)
        if position>0.03:
            if min(fingers)<0.035:raise RuntimeError("Gripper did not open")
        else:
            half=self.metadata["target_size_m"]/2
            if any(abs(v-half)>0.006 for v in fingers):
                raise RuntimeError(f"Fingers did not stop around object: {fingers}")

    def run(self):
        self.state("INIT")
        deadline=time.monotonic()+60
        while self.joints is None or time.monotonic()-self.joint_time>0.5:
            self.spin(0.1)
            if time.monotonic()>deadline:raise RuntimeError("No fresh ROS joint states")
        self.state("PERCEIVE")
        truth=self.metadata["objects"]
        errors={name:float(np.linalg.norm(np.array(item["T_world_object"])[:3,3]-
                   truth[name]["ground_truth"]["position_world_m"]))
                for name,item in self.detections["objects"].items()}
        self.report["foundationpose_position_errors_m"]=errors
        self.report["target_selection_correct"]=self.detections["objects"][self.target]["color"]==truth[self.target]["color"]
        if max(errors.values())>0.02:raise RuntimeError(f"Visual pose quality gate failed: {errors}")
        self.state("SELECT",target=self.target)
        center=np.array(self.detections["objects"][self.target]["T_base_object"])[:3,3]
        place=np.array([self.args.place_x,self.args.place_y,center[2]])
        support=self.metadata["support_surface"]
        place_world=(self.base@np.r_[place,1])[:3]
        if (not np.isfinite(place).all() or np.any(np.abs(place_world[:2]-support["center_world_m"][:2])
                +self.metadata["target_size_m"]/2+0.02 >= np.array(support["dimensions_m"][:2])/2)):
            raise ValueError("Place target lies outside the physical support")
        for name,item in self.detections["objects"].items():
            if name!=self.target and np.linalg.norm(np.array(item["T_base_object"])[:2,3]-place[:2])<0.09:
                raise ValueError("Place area occupied")
        self.scene_setup()
        self.state("OPEN")
        self.grip(self.params["open_position_m"])
        self.state("PREGRASP")
        self.move_tcp(center+[0,0,self.params["pregrasp_clearance_m"]])
        self.contacts([(self.target,name,True) for name in ["panda_leftfinger","panda_rightfinger","studio_floor"]])
        grasp=center+[0,0,self.params["approach_z_bias_m"]]
        self.state("APPROACH")
        self.move_tcp(grasp,True)
        self.state("CLOSE")
        # Size-relative closing target. Actual force is bounded by the USD drive;
        # contact/stall completion still requires independent physical lift tests.
        self.grip(max(0.,self.metadata["target_size_m"]/2-self.params["close_preload_m"]))
        self.reference=relative_pose(read_state(self.run_dir),self.target)
        self.holding=True
        self.attach()  # Planning representation only; simulator body remains dynamic.
        self.state("TEST_LIFT")
        self.move_tcp(grasp+[0,0,self.params["test_lift_m"]],True)
        self.spin(1.)
        if self.max_lift<0.025:raise RuntimeError("Physical object did not leave support")
        self.report["grasp_success"]=True
        self.contacts([(self.target,"studio_floor",False)])
        safe_z=self.metadata["support_surface"]["top_z_m"]+self.params["safe_tcp_height_above_support_m"]
        self.state("LIFT")
        self.move_tcp([*center[:2],safe_z],True)
        self.state("TRANSFER")
        self.move_tcp([self.args.place_x,self.args.place_y,safe_z])
        self.report["transport_success"]=True
        self.state("PLACE")
        self.contacts([(self.target,"studio_floor",True)])
        support_base=self.base_inverse@np.array([0,0,self.metadata["support_surface"]["top_z_m"],1])
        place_z=support_base[2]+self.metadata["target_size_m"]/2+self.params["approach_z_bias_m"]
        self.move_tcp([self.args.place_x,self.args.place_y,place_z],True)
        self.state("RELEASE")
        self.holding=False
        self.grip(self.params["open_position_m"])
        self.spin(1.)
        self.detach()
        self.state("RETREAT")
        self.move_tcp([self.args.place_x,self.args.place_y,place_z+0.15],True)
        self.contacts([(self.target,name,False) for name in ["panda_leftfinger","panda_rightfinger","studio_floor"]])
        self.state("VERIFY")
        self.spin(2.)
        final=read_state(self.run_dir)
        obj=final["objects"][self.target]
        point=(self.base_inverse@np.array(obj["transform_world"]))[:3,3]
        error=float(np.linalg.norm(point[:2]-place[:2]))
        speed=float(np.linalg.norm(obj["linear_velocity_m_s"]))
        separation=float(np.linalg.norm(np.array(final["tcp_transform_world"])[:3,3]-obj["position_world_m"]))
        disturbance={name:float(np.linalg.norm(np.array(data["position_world_m"])-
                     self.initial["objects"][name]["position_world_m"]))
                     for name,data in final["objects"].items() if name!=self.target}
        self.report.update(final_place_error_m=error,final_position_base_m=point.tolist(),
                           final_speed_m_s=speed,final_tcp_separation_m=separation,
                           non_target_displacement_m=disturbance,maximum_relative_slip_m=self.max_slip,
                           maximum_lift_m=self.max_lift)
        if (error>self.params["place_tolerance_m"] or speed>0.01 or separation<0.08
                or abs(point[2]-(place_z-self.params["approach_z_bias_m"]))>0.005
                or max(disturbance.values(),default=0)>self.params["non_target_tolerance_m"]):
            raise RuntimeError("Independent physical placement verification failed")
        self.report["success"]=self.report["place_success"]=True
        self.state("SUCCESS")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-dir",type=Path,required=True)
    parser.add_argument("--target",default="red_cube")
    parser.add_argument("--place-x",type=float,default=0.4)
    parser.add_argument("--place-y",type=float,default=0.25)
    args=parser.parse_args()
    rclpy.init()
    node=PickPlace(args)
    code=0
    try:node.run()
    except Exception as error:
        code=1
        node.report.update(failure_stage=node.current,failure_reason=str(error))
        node.state("FAILED",reason=str(error))
        traceback.print_exc()
    finally:
        node.report.update(task_seconds=time.monotonic()-node.start,events=node.events)
        atomic_json(node.run_dir/"task_result.json",node.report)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__=="__main__":main()
