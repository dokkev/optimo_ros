from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="optimo"),
        DeclareLaunchArgument("check_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("force_threshold", default_value="30.0"),

        # Usage:
        #   Terminal 1: ros2 launch optimo_bringup optimo.launch.py
        #   Terminal 2: ros2 launch optimo_safety_monitor safety_monitor.launch.py
        #   Terminal 3: ros2 service call /optimo/optimo_effort_controller/play_traj_cb \
        #     optimo_msgs/srv/PlayTrajCb \
        #     "{duration: 0, goal: {filepath: '/path/to/traj', \
        #       param: {imp: {impedance: 5.0, max_force: 5.0, stop_sensitivity: 5.0}, \
        #       recovery_speed: 5.0, accept_diff_start: true}}}"

        Node(
            package="optimo_safety_monitor",
            executable="safety_monitor_node",
            name="safety_monitor",
            namespace=LaunchConfiguration("ns"),
            output="screen",
            parameters=[{
                "check_rate_hz": LaunchConfiguration("check_rate_hz"),
                "force_threshold": LaunchConfiguration("force_threshold"),
            }],
        ),
    ])
