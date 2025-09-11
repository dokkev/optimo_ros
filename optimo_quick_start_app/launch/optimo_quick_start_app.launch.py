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
    RegisterEventHandler,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
import os
import signal
import psutil


def generate_launch_description():

    os.environ["ROS_LOG_DIR"] = "./execution_logs"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        "ns", default_value="", description="Namespace for the Optimo Quick Start App."
    )

    # Define the node
    optimo_quick_start_app_node = Node(
        package="optimo_quick_start_app",
        executable="optimo_quick_start_app",
        name="optimo_quick_start_app",
        namespace=LaunchConfiguration("ns"),
        output="screen",
    )

    def shutdown_func(event, context):
        pid = optimo_quick_start_app_node.process_details.get("pid")
        if pid is not None:
            try:
                p = psutil.Process(pid)
                if p.is_running():
                    os.kill(pid, signal.SIGINT)
            except psutil.NoSuchProcess:
                return [
                    LogInfo(
                        msg=[
                            "Launch was asked to shutdown: ",
                            LocalSubstitution("event.reason"),
                        ]
                    )
                ]
        return [
            LogInfo(
                msg=[
                    "Sending SIGINT to all nodes, Launch was asked to shutdown: ",
                    LocalSubstitution("event.reason"),
                ]
            )
        ]

    shutdown_event_handler = RegisterEventHandler(OnShutdown(on_shutdown=shutdown_func))

    return LaunchDescription(
        [
            namespace_arg,
            optimo_quick_start_app_node,
            shutdown_event_handler,
        ]
    )
