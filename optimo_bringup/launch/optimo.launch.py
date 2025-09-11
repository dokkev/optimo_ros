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
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch_ros.actions import PushRosNamespace, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "ns", default_value="optimo", description="Namespace"
    )

    use_sim_hardware_arg = DeclareLaunchArgument(
        "use_sim_hardware",
        description="If simulated hardware should be used, aka Gazebo.",
        default_value="False",
    )
    return LaunchDescription(
        [
            ns_arg,
            use_sim_hardware_arg,
            # GroupAction(
            #     actions=[
            #         PushRosNamespace(ns_arg),
            #         SetParameter(name="use_sim_time", value=use_sim_hardware_arg),
            #         SetRemap(src="/tf", dst="tf"),
            #         SetRemap(src="/tf_static", dst="tf_static"),
            #         IncludeLaunchDescription(
            #             PythonLaunchDescriptionSource(
            #                 [
            #                     os.path.join(
            #                         get_package_share_directory("optimo_moveit_config"),
            #                         "launch",
            #                         "move_group_servo.launch.py",
            #                     )
            #                 ]
            #             ),
            #         ),
            #     ]
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("optimo_description"),
                            "launch",
                            "rviz.launch.py",
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("optimo_bringup"),
                            "launch",
                            "control.launch.py",
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("optimo_bringup"),
                            "launch",
                            "gz.launch.py",
                        )
                    ]
                ),
                condition=IfCondition(LaunchConfiguration("use_sim_hardware")),
            ),
        ]
    )
