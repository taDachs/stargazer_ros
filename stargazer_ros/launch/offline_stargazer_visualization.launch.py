from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'res', 'Stargazer.bag_optimized.bag'),
            description='Path to the optimized bag file'
        ),
    ]

    # Include Landmark Visualizer
    landmark_visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'landmark_visualizer.launch.py')
        ),
        launch_arguments={
            'map_config': LaunchConfiguration('map_config'),
            'start_rviz': 'false',
        }.items(),
    )

    # Include Pose Visualizer (first instance)
    pose_visualizer_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'offline_pose_visualizer.launch.py')
        ),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
            'cam_config': LaunchConfiguration('cam_config'),
            'map_config': LaunchConfiguration('map_config'),
            'pose_pub_topic': '/poses',
            'start_rviz': 'true',
        }.items(),
    )

    # Include Pose Visualizer (second instance)
    pose_visualizer_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'offline_pose_visualizer.launch.py')
        ),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
            'cam_config': os.path.join(stargazer_ros_tool_share_dir, 'out', 'cam_optimized.yaml'),
            'map_config': os.path.join(stargazer_ros_tool_share_dir, 'out', 'map_optimized.yaml'),
            'pose_pub_topic': '/poses_optimized',
            'start_rviz': 'false',
        }.items(),
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(
        launch_arguments + [
            landmark_visualizer_launch,
            pose_visualizer_launch_1,
            pose_visualizer_launch_2,
        ]
    )

