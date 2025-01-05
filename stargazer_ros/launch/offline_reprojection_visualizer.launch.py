from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the share directory of the package
    stargazer_ros_tool_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'cam_config',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'cam.yaml'),
            description='Path to the camera configuration file'
        ),
        DeclareLaunchArgument(
            'map_config',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'map.yaml'),
            description='Path to the map configuration file'
        ),
        DeclareLaunchArgument(
            'bag_file',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'res', 'Stargazer.bag'),
            description='Path to the input bag file'
        ),
    ]

    # Reprojection Visualizer Node
    reprojection_visualizer_node = Node(
        package='stargazer_ros_tool',
        executable='reprojection_visualizer',
        name='reprojection_visualizer',
        output='screen',
        parameters=[
            {'cam_config': LaunchConfiguration('cam_config')},
            {'map_config': LaunchConfiguration('map_config')},
            {'bag_file': LaunchConfiguration('bag_file')},
            {'landmark_topic': '/landmarks_seen'},
            {'pose_topic': '/stargazer/camera_pose'},
            {'img_topic': '/image_undistorted'},
            {'wait_time': 10},
        ],
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(launch_arguments + [reprojection_visualizer_node])

