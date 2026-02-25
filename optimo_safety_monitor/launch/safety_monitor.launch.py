from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="optimo"),
        DeclareLaunchArgument("check_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("wrench_force_threshold", default_value="10.0"),
        DeclareLaunchArgument("baseline_file", default_value=""),

        # Usage:
        #   Terminal 1: ros2 launch optimo_bringup optimo.launch.py
        #   Terminal 2: ros2 launch optimo_safety_monitor safety_monitor.launch.py
        #
        #   Record baseline (no external contact):
        #     ros2 service call /optimo/safety_monitor/record_baseline std_srvs/srv/Trigger
        #     <play trajectory>
        #     ros2 service call /optimo/safety_monitor/save_baseline std_srvs/srv/Trigger
        #
        #   Run with baseline:
        #     ros2 launch optimo_safety_monitor safety_monitor.launch.py \
        #       baseline_file:=/tmp/optimo_wrench_baseline.csv

        Node(
            package="optimo_safety_monitor",
            executable="safety_monitor_node",
            name="safety_monitor",
            namespace=LaunchConfiguration("ns"),
            output="screen",
            parameters=[{
                "check_rate_hz": LaunchConfiguration("check_rate_hz"),
                "wrench_force_threshold": LaunchConfiguration("wrench_force_threshold"),
                "baseline_file": LaunchConfiguration("baseline_file"),
            }],
        ),
    ])
