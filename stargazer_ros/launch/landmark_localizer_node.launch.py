from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the share directory of the package
    package_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument('debug_mode', default_value='false'),
        DeclareLaunchArgument('estimate_2d_pose', default_value='false'),
        DeclareLaunchArgument('cam_config', default_value=os.path.join(package_share_dir, 'param', 'cam.yaml')),
        DeclareLaunchArgument('map_config', default_value=os.path.join(package_share_dir, 'param', 'map.yaml')),
        DeclareLaunchArgument('landmark_topic', default_value='/landmarks_seen'),
        DeclareLaunchArgument('pose_topic', default_value='/stargazer/camera_pose'),
        DeclareLaunchArgument('map_frame', default_value='world'),
        DeclareLaunchArgument('camera_frame', default_value='camera'),
    ]

    # Node definition
    landmark_localizer_node = Node(
        package='stargazer_ros_tool',
        executable='landmark_localizer',
        name='landmark_localizer',
        output='screen',
        parameters=[
            {'cam_config': LaunchConfiguration('cam_config')},
            {'map_config': LaunchConfiguration('map_config')},
            {'estimate_2d_pose': LaunchConfiguration('estimate_2d_pose')},
            {'pose_topic': LaunchConfiguration('pose_topic')},
            {'landmark_topic': LaunchConfiguration('landmark_topic')},
            {'map_frame': LaunchConfiguration('map_frame')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'debug_mode': LaunchConfiguration('debug_mode')},
        ],
    )

    return LaunchDescription(launch_arguments + [landmark_localizer_node])
