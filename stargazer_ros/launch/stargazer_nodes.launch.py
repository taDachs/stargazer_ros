from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Define the package share directory
    package_share_dir = FindPackageShare('stargazer_ros_tool').find('stargazer_ros_tool')

    # Declare launch arguments
    launch_arguments = [
        # Topics
        DeclareLaunchArgument(
            'undistorted_image_topic',
            default_value='/sensors/camera/top/image_raw',
            description='Topic for undistorted images'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks_seen',
            description='Topic for landmarks'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/stargazer/camera_pose',
            description='Topic for camera poses'
        ),

        # TF Frames
        DeclareLaunchArgument(
            'map_frame',
            default_value='stargazer',
            description='Map TF frame'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_top',
            description='Camera TF frame'
        ),

        # Config files
        DeclareLaunchArgument(
            'cam_config',
            default_value=os.path.join(package_share_dir, 'param', 'cam.yaml'),
            description='Camera configuration file'
        ),
        DeclareLaunchArgument(
            'map_config',
            default_value=os.path.join(package_share_dir, 'param', 'map.yaml'),
            description='Map configuration file'
        ),
        DeclareLaunchArgument(
            'detection_config',
            default_value=os.path.join(package_share_dir, 'param', 'landmark_finder.yaml'),
            description='Landmark finder configuration file'
        ),

        # Miscellaneous
        DeclareLaunchArgument(
            'debug_finder',
            default_value='false',
            description='Enable debug mode for landmark finder'
        ),
        DeclareLaunchArgument(
            'debug_localizer',
            default_value='false',
            description='Enable debug mode for landmark localizer'
        ),
        DeclareLaunchArgument(
            'estimate_2d_pose',
            default_value='false',
            description='Estimate 2D pose instead of full pose'
        ),
    ]

    # Include other launch files
    landmark_finder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, 'launch', 'landmark_finder_node.launch.py')
        ),
        launch_arguments={
            'map_config': LaunchConfiguration('map_config'),
            'detection_config': LaunchConfiguration('detection_config'),
            'undistorted_image_topic': LaunchConfiguration('undistorted_image_topic'),
            'landmark_topic': LaunchConfiguration('landmark_topic'),
            'debug_mode': LaunchConfiguration('debug_finder'),
        }.items(),
    )

    landmark_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, 'launch', 'landmark_localizer_node.launch.py')
        ),
        launch_arguments={
            'cam_config': LaunchConfiguration('cam_config'),
            'map_config': LaunchConfiguration('map_config'),
            'estimate_2d_pose': LaunchConfiguration('estimate_2d_pose'),
            'landmark_topic': LaunchConfiguration('landmark_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'debug_mode': LaunchConfiguration('debug_localizer'),
        }.items(),
    )

    return LaunchDescription(launch_arguments + [landmark_finder_launch, landmark_localizer_launch])

