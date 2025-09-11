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
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="spaceMouse_navigator",
            executable="spaceMouse_publisher",
            name="spaceMouse_publisher",
            output="screen"
        ),
        Node(
            package="plato_teleop",
            executable="controller_spaceMouse",
            name="controller_spaceMouse",
            output="screen"
        ),
        Node(
            package="spaceMouse_navigator",
            executable="spaceMouse_state_machine",
            name="spaceMouse_state_machine",
            output="screen"
        )
    ])
