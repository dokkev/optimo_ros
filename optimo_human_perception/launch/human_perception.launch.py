from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs_share = get_package_share_directory('realsense2_camera')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='optimo'),
        DeclareLaunchArgument('min_detection_confidence', default_value='0.5'),
        DeclareLaunchArgument('min_tracking_confidence', default_value='0.5'),

        # Launch RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
            }.items(),
        ),

        # Launch perception node
        Node(
            package='optimo_human_perception',
            executable='human_perception_node',
            name='human_perception',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{
                'min_detection_confidence': LaunchConfiguration('min_detection_confidence'),
                'min_tracking_confidence': LaunchConfiguration('min_tracking_confidence'),
                'image_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
            }],
        ),
    ])
