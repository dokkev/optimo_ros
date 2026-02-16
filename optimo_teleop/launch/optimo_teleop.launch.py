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
    # Shutdown handler node (calls stop service on Ctrl+C)
    shutdown_handler_node = Node(
        package="optimo_teleop",
        executable="shutdown_handler",
        name="shutdown_handler",
        output="screen",
    )

    # Twist teleop node (converts twist commands to PoseElbow messages)
    # This node subscribes to TwistStamped messages and publishes PoseElbow commands
    twist_teleop_node = Node(
        package="optimo_teleop",
        executable="twist_teleop_node",
        namespace="optimo",
        output="screen",
        remappings=[
            ("/tf", "/optimo/tf"),
            ("/tf_static", "/optimo/tf_static")
        ]
    )

    # Call servo service to start servoing
    start_servo_service = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/optimo/optimo_effort_controller/servo_cb",
            "optimo_msgs/srv/ServoCb",
            "{duration: 0, topic_name: '/optimo/servo/ee_pose_desired', cartesian: true}"
        ],
        output="screen"
    )

    return LaunchDescription([
        # Shutdown handler must be launched first to catch Ctrl+C
        shutdown_handler_node,

        # Start servo service immediately
        start_servo_service,

        # Launch twist teleop node after 2 second delay
        TimerAction(
            period=2.0,
            actions=[
                twist_teleop_node
            ]
        )
    ])
