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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def load_xacro(xacro_path, mappings=None):
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toprettyxml(indent="  ")


def launch_setup(context, *args, **kwargs):
    # Get the namespace
    namespace = LaunchConfiguration("ns").perform(context)

    # Get the path to the URDF file
    pkg_optimo_description = get_package_share_directory("optimo_description")
    urdf_path = os.path.join(pkg_optimo_description, "urdf", "optimo.urdf.xacro")

    # Process the XACRO file
    robot_description_content = load_xacro(
        urdf_path,  # mappings={"prefix": namespace + "/"}
    )

    # Create the Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # Create the Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Include the RViz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_optimo_description, "launch", "rviz.launch.py")]
        )
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                robot_state_publisher_node,
                joint_state_publisher_gui_node,
            ]
        ),
        rviz_launch,
    ]


def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        "ns", default_value="optimo", description="Namespace for the nodes"
    )

    # Create and return the launch description
    return LaunchDescription(
        [
            namespace_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
