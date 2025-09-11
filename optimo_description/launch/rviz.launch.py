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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        "ns", default_value="optimo", description="Namespace for the RViz node"
    )

    # Get the path to the RViz configuration file
    rviz_config_path = (
        os.path.join(
            get_package_share_directory("optimo_description"),
            "config",
            "urdf.rviz",
        ),
    )

    # Create the RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration("ns"),
        arguments=["-d", rviz_config_path],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
    )

    # Create and return the launch description
    return LaunchDescription([namespace_arg, rviz_node])
