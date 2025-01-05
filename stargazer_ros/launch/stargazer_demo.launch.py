from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the share directory of the packages
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
    ]

    # Rosbag play process
    rosbag_play_process = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            os.path.join(stargazer_ros_tool_share_dir, 'res', 'stargazer.bag'),
            '--clock'
        ],
        output='screen'
    )

    # Include Stargazer nodes
    stargazer_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'stargazer_nodes.launch.py')
        ),
        launch_arguments={
            'cam_config': LaunchConfiguration('cam_config'),
            'map_config': LaunchConfiguration('map_config'),
        }.items(),
    )

    # Include the visualizer
    landmark_visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'landmark_visualizer.launch.py')
        ),
        launch_arguments={
            'map_config': LaunchConfiguration('map_config'),
            'start_rviz': 'false',
        }.items(),
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(launch_arguments + [rosbag_play_process, stargazer_nodes_launch, landmark_visualizer_launch])

