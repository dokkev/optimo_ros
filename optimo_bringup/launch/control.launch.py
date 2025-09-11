# Copyright 2024 Roboligent, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import (
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import xacro
import yaml


def str2bool(s):
    return s.lower() in ["true", "1", "t", "y", "yes"]


# This function modifies controllers.yaml. This has to be done through actual writing to a file,
# because gazebo's ros2 control plugin only supports adding parameters through a param file provided using the urdf file.
# Since the effort controller needs to know which robot (sim or real) it is modeling, we need that information to appear in this param file.
# Otherwise, we would just have these parameters defined in the launch file, without the param file involved.
def modify_yaml_file(prefix, robot_index, use_sim_hardware, ns):
    # Path to your original YAML file
    pkg_share_dir = get_package_share_directory("optimo_bringup")
    original_yaml_path = os.path.join(pkg_share_dir, "config", "controllers.yaml")

    # Path to save the modified YAML file
    modified_yaml_path = os.path.join(
        pkg_share_dir, "config", "modified_control_" + robot_index + ".yaml"
    )

    # Load the original YAML content
    with open(original_yaml_path, "r") as file:
        yaml_content = yaml.safe_load(file)

    # Modify the YAML content
    controller_name = "/**/optimo_effort_controller"
    if controller_name in yaml_content:
        yaml_content[controller_name]["ros__parameters"]["use_sim_hardware"] = str2bool(
            use_sim_hardware
        )

    os.makedirs(os.path.dirname(modified_yaml_path), exist_ok=True)

    # Write the modified YAML content to a new file
    with open(modified_yaml_path, "w") as file:
        yaml.dump(yaml_content, file)

    return


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("ns").perform(context)
    use_gripper = LaunchConfiguration("use_gripper").perform(context)
    robot_index = LaunchConfiguration("robot_index").perform(context)
    use_sim_hardware = LaunchConfiguration("use_sim_hardware")
    prefix = LaunchConfiguration("prefix").perform(context)
    controller_yaml_path = LaunchConfiguration("controller_yaml_path").perform(context)

    # Load robot description
    pkg_desc_dir = get_package_share_directory("optimo_description")
    xacro_file = os.path.join(pkg_desc_dir, "urdf", "optimo.urdf.xacro")
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "ns": ns,
            "use_gripper": use_gripper,
            "use_sim_hardware": use_sim_hardware.perform(context),
            "robot_index": robot_index,
            "prefix": prefix,
        },
    )
    robot_desc = doc.toprettyxml(indent="  ")

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_desc,
            }
        ],
    )

    modify_control_yaml = OpaqueFunction(
        function=lambda context, *args, **kwargs: (
            modify_yaml_file(
                prefix, robot_index, use_sim_hardware.perform(context), ns
            ),
            [],
        )[1]
    )

    # Determine which YAML file to use
    if controller_yaml_path:
        yaml_path = controller_yaml_path
    else:
        yaml_path = PathJoinSubstitution(
            [
                FindPackageShare("optimo_bringup"),
                "config",
                "modified_control_" + robot_index + ".yaml",
            ]
        )

    control_node = Node(
        condition=UnlessCondition(use_sim_hardware),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            yaml_path,
            {
                "robot_description": robot_desc,
                "robot_index": int(robot_index),
                "use_sim_hardware": use_sim_hardware,
            },
        ],
        output="both",
    )

    # Load controllers using controller_manager
    # Namespace is manually added because PushROSNamespace does not work on RegisterEventHandler
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        namespace=ns,
    )
    load_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["optimo_effort_controller"],
        namespace=ns,
    )
    load_gripper_position_controller = Node(
        condition=IfCondition(LaunchConfiguration("use_gripper")),
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller"],
        namespace=ns,
    )
    load_gripper_effort_controller = Node(
        condition=IfCondition(LaunchConfiguration("use_gripper")),
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_effort_controller"],
        namespace=ns,
    )
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                load_joint_effort_controller,
                load_gripper_position_controller,
                load_gripper_effort_controller,
            ],
        )
    )
    # Delay start of controllers for 20 seconds, hardware takes 20 seconds to start.
    delay_joint_state_broadcaster = TimerAction(
        period=18.0,
        actions=[load_joint_state_broadcaster],
        condition=UnlessCondition(use_sim_hardware),
    )
    # Spawners don't like competing with each other?
    delay_joint_state_broadcaster_sim = TimerAction(
        period=10.0,
        actions=[load_joint_state_broadcaster],
        condition=IfCondition(use_sim_hardware),
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=use_sim_hardware),
                PushRosNamespace(ns),
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                modify_control_yaml,
                control_node,
                robot_state_pub_node,
                delay_joint_state_broadcaster,
                delay_joint_state_broadcaster_sim,
                load_controllers,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ns", default_value="optimo", description="Namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_hardware",
                description="If simulated hardware should be used, aka Gazebo.",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "robot_index",
                description="The robot index. Should be 0 or 1, based off the EtherCAT master index desired.",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "use_gripper",
                default_value="False",
                description="If the gripper should be used",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="The prefix for the joints. Should be left_arm or right_arm.",
            ),
            DeclareLaunchArgument(
                "controller_yaml_path",
                default_value="",
                description="The path to the controller yaml file.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
