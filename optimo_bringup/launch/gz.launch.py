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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch.actions import GroupAction


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("ns").perform(context)

    # Gazebo resources
    pkg_desc_dir = get_package_share_directory("optimo_description")
    pkg_sim_dir = get_package_share_directory("optimo_sim")
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = pkg_desc_dir + "/..:" + pkg_desc_dir
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = (
        pkg_sim_dir + "/..:" + pkg_sim_dir + ":" + pkg_sim_dir + "/../../lib/optimo_sim"
    )
    os.environ["IGN_GAZEBO_PLUGIN_PATH"] = (
        pkg_sim_dir + "/..:" + pkg_sim_dir + ":" + pkg_sim_dir + "/lib/optimo_sim"
    )

    # Gazebo itself
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [" -r -v 1 " + os.path.join(pkg_desc_dir, "urdf", "camera_world.sdf")],
            )
        ],
        condition=UnlessCondition(LaunchConfiguration("spawn_only")),
    )

    # Bridge for Gazebo
    camera_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("spawn_only")),
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("ns"),
                    "' or 'optimo'",
                ]
            ),
            "-allow_renaming",
            "true",
        ],
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=True),
                PushRosNamespace(ns),
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                gazebo,
                camera_bridge,
                ignition_spawn_entity,
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
                "spawn_only",
                default_value="False",
                description="Whether to only spawn the robot with description at robot_description topic",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
