"""Isolated virtual-robot launch; reuses the existing Panda model and OMPL configs."""
from pathlib import Path
import os
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scene_frames import load_scene_frames, static_transform_arguments
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    metadata,base,_=load_scene_frames()
    config=(MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path=str(ROOT/"configs/pick_place.urdf.xacro"))
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    return LaunchDescription([
        Node(package="tf2_ros",executable="static_transform_publisher",
             arguments=static_transform_arguments(base,"world","panda_link0"),
             parameters=[{"use_sim_time":True}]),
        Node(package="robot_state_publisher",executable="robot_state_publisher",
             parameters=[config.robot_description,{"use_sim_time":True}]),
        Node(package="moveit_ros_move_group",executable="move_group",output="screen",
             parameters=[config.to_dict(),{"use_sim_time":True}]),
        Node(package="controller_manager",executable="ros2_control_node",output="screen",
             parameters=[os.environ.get("PICK_PLACE_CONTROLLER_CONFIG",str(ROOT/"configs/pick_place_controllers.yaml")),{"use_sim_time":True}],
             remappings=[("/controller_manager/robot_description","/robot_description")]),
        *[Node(package="controller_manager",executable="spawner",output="screen",
               arguments=[name,"--controller-manager","/controller_manager","--controller-manager-timeout","60"])
          for name in ["joint_state_broadcaster","panda_arm_controller","panda_hand_controller"]]
    ])
