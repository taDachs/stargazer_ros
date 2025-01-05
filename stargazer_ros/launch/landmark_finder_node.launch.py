from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the share directory of the package
    package_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'map_config',
            default_value=os.path.join(package_share_dir, 'param', 'map.yaml'),
            description='Path to the map configuration file'
        ),
        DeclareLaunchArgument(
            'detection_config',
            default_value=os.path.join(package_share_dir, 'param', 'landmark_finder.yaml'),
            description='Path to the landmark finder configuration file'
        ),
        DeclareLaunchArgument(
            'undistorted_image_topic',
            default_value='/camera_top/image_raw',
            description='Topic for undistorted images'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks_seen',
            description='Topic for landmarks'
        ),
    ]

    # Node definition
    landmark_finder_node = Node(
        package='stargazer_ros_tool',
        executable='landmark_finder',
        name='landmark_finder',
        output='screen',
        parameters=[
            {'map_config': LaunchConfiguration('map_config')},
            {'landmark_topic': LaunchConfiguration('landmark_topic')},
            {'undistorted_image_topic': LaunchConfiguration('undistorted_image_topic')},
            {'debug_mode': LaunchConfiguration('debug_mode')},
            LaunchConfiguration('detection_config'),  # Load detection_config as a ROS parameters file
        ],
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(launch_arguments + [landmark_finder_node])
