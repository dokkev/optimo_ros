import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Launches move group node and servo.
# Servo part taken from https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/launch/servo_example.launch.py


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    moveit_config = (
        MoveItConfigsBuilder("optimo").robot_description().to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("optimo_moveit_config", "config/servo.yaml")

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            {
                "moveit_servo.joint_topic": "servo_node/servo_joint_states",  # assigning this here grants the namespace
                "use_sim_time": use_sim_time,
                "moveit_servo": servo_yaml,
            },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("optimo_moveit_config"),
                            "launch",
                            "move_group.launch.py",
                        )
                    ]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            servo_node,
        ]
    )
