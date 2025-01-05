from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
            'landmark_finder_config',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'landmark_finder.yaml'),
            description='Path to the landmark finder configuration file'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks_seen',
            description='Topic for landmarks'
        ),
        DeclareLaunchArgument(
            'img_topic',
            default_value='/camera/image_raw_stargazer',
            description='Topic for the input image'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/stargazer/camera_pose',
            description='Topic for the pose output'
        ),
        DeclareLaunchArgument(
            'in_raw_bag',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'res', 'stargazer_raw.bag'),
            description='Path to the input raw rosbag file'
        ),
        DeclareLaunchArgument(
            'out_bag',
            default_value='/tmp/stargazer_bag_for_calib.bag',
            description='Path to the output rosbag file'
        ),
    ]

    # Rosbag Play Process
    rosbag_play_process = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            LaunchConfiguration('in_raw_bag'),
            '--clock',
            '--rate',
            '0.2',
        ],
        output='screen',
    )

    # Include Stargazer Nodes
    stargazer_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stargazer_ros_tool_share_dir, 'launch', 'stargazer_nodes.launch.py')
        ),
        launch_arguments={
            'estimate_2d_pose': 'false',
            'undistorted_image_topic': LaunchConfiguration('img_topic'),
            'landmark_topic': LaunchConfiguration('landmark_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'map_frame': 'map_stargazer',
            'camera_frame': 'camera_stargazer',
            'map_config': LaunchConfiguration('map_config'),
            'cam_config': LaunchConfiguration('cam_config'),
            'detection_config': LaunchConfiguration('landmark_finder_config'),
            'debug_finder': 'false',
            'debug_localizer': 'false',
        }.items(),
    )

    # Rosbag Record Process
    rosbag_record_process = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            LaunchConfiguration('out_bag'),
            LaunchConfiguration('img_topic'),
            LaunchConfiguration('pose_topic'),
            LaunchConfiguration('landmark_topic'),
        ],
        output='screen',
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(
        launch_arguments + [rosbag_play_process, stargazer_nodes_launch, rosbag_record_process]
    )

