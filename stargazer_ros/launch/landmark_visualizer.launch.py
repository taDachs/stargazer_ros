from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the share directory of the package
    stargazer_ros_tool_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'map_config',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'map.yaml'),
            description='Path to the map configuration file'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks',
            description='Topic for landmarks'
        ),
        DeclareLaunchArgument(
            'map_frame_id',
            default_value='stargazer',
            description='Frame ID for the map'
        ),
    ]

    # Landmark Visualizer Node
    landmark_visualizer_node = Node(
        package='stargazer_ros_tool',
        executable='landmark_visualizer',
        name='landmark_visualizer',
        output='screen',
        parameters=[
            {'map_config': LaunchConfiguration('map_config')},
            {'landmark_topic': LaunchConfiguration('landmark_topic')},
            {'rate': 0.2},
            {'map_frame_id': LaunchConfiguration('map_frame_id')},
        ],
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(stargazer_ros_tool_share_dir, 'res', 'stargazer.rviz')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(launch_arguments + [landmark_visualizer_node, rviz_node])

