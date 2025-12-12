from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("optimo_state_estimator"), "rviz", "state_estimator.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="optimo_state_estimator",
                executable="state_estimator_node",
                name="optimo_state_estimator",
                namespace="optimo",
                output="screen",
                parameters=[
                    {
                        "use_sim_hardware": True,
                        "robot_index": 0,
                        "robot_prefix": "",
                        "base_frame": "base_link",
                        "link0_frame": "link0",
                        "ee_frame": "ee",
                        "publish_rate": 30.0,
                        "tf_timeout": 0.05,
                        "publish_markers": True,
                    }
                ],
                remappings=[
                    ("/joint_states", "/optimo/joint_states"),
                    ("/tf", "/optimo/tf"),
                    ("/tf_static", "/optimo/tf_static"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition("true"),
            ),
        ]
    )
