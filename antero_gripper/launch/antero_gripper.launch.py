from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='optimo'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rate', default_value='25.0'),

        Node(
            package='antero_gripper',
            executable='antero_gripper_node',
            name='antero_gripper',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'rate': LaunchConfiguration('rate'),
            }],
        ),
    ])
