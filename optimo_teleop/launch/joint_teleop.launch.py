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
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First: Immediately call the servo_cb service with cartesian=false
        ExecuteProcess(
            cmd=[
                "ros2", "service", "call",
                "/optimo/optimo_effort_controller/servo_cb",
                "optimo_msgs/srv/ServoCb",
                "{duration: 0, topic_name: '/optimo/joint_teleop/desired_joint_state', cartesian: false}"
            ],
            output="screen"
        ),

        # Then: Delay launching the joint teleop node by 2 seconds
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="optimo_teleop",
                    executable="joint_teleop_node",
                    namespace="optimo",
                    output="screen",
                    parameters=[
                        {"velocity_scale": 0.01},  # Adjust for sensitivity
                        {"num_joints": 7}
                    ]
                )
            ]
        )
    ])
