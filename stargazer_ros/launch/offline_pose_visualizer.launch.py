from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the share directory of the package
    stargazer_ros_tool_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'bag_file',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'res', 'Stargazer.bag_optimized.bag'),
            description='Path to the optimized bag file'
        ),
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
            'map_frame',
            default_value='world',
            description='Frame ID for the map'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera',
            description='Frame ID for the camera'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/stargazer/camera_pose',
            description='Topic for the camera pose'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks_seen',
            description='Topic for landmarks'
        ),
        DeclareLaunchArgument(
            'pose_pub_topic',
            default_value='/poses',
            description='Topic for publishing poses'
        ),
    ]

    # Include extrinsic calibration launch file
    extrinsic_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'extrinsic_calibration.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    # Pose Visualizer Node
    pose_visualizer_node = Node(
        package='stargazer_ros_tool',
        executable='pose_visualizer',
        output='screen',
        name="pose_visualizer",
        parameters=[
            {'bag_file': LaunchConfiguration('bag_file')},
            {'cam_config': LaunchConfiguration('cam_config')},
            {'map_config': LaunchConfiguration('map_config')},
            {'map_frame': LaunchConfiguration('map_frame')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'landmark_topic': LaunchConfiguration('landmark_topic')},
            {'pose_pub_topic': LaunchConfiguration('pose_pub_topic')},
            {'rate': 0.2},
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
    return LaunchDescription(
        launch_arguments + [extrinsic_calibration_launch, pose_visualizer_node, rviz_node]
    )
