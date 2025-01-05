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
            'cam_in',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'cam.yaml'),
            description='Path to the input camera configuration file'
        ),
        DeclareLaunchArgument(
            'cam_out',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'out', 'cam_optimized.yaml'),
            description='Path to the output optimized camera configuration file'
        ),
        DeclareLaunchArgument(
            'map_in',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'param', 'map.yaml'),
            description='Path to the input map configuration file'
        ),
        DeclareLaunchArgument(
            'map_out',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'out', 'map_optimized.yaml'),
            description='Path to the output optimized map configuration file'
        ),
        DeclareLaunchArgument(
            'bag',
            default_value=os.path.join(stargazer_ros_tool_share_dir, 'res', 'Stargazer.bag'),
            description='Path to the input bag file'
        ),
        DeclareLaunchArgument(
            'constant_intrinsics',
            default_value='true',
            description='Flag to keep camera intrinsics constant'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmarks_seen',
            description='Topic for landmarks'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/stargazer/camera_pose',
            description='Topic for the camera pose'
        ),
    ]

    # Landmark Calibrator Node
    landmark_calibrator_node = Node(
        package='stargazer_ros_tool',
        executable='landmark_calibrator',
        name='landmark_calibrator',
        output='screen',
        parameters=[
            {'constant_intrinsics': LaunchConfiguration('constant_intrinsics')},
            {'cam_cfg_file_in': LaunchConfiguration('cam_in')},
            {'cam_cfg_file_out': LaunchConfiguration('cam_out')},
            {'map_cfg_file_in': LaunchConfiguration('map_in')},
            {'map_cfg_file_out': LaunchConfiguration('map_out')},
            {'bag_file': LaunchConfiguration('bag')},
            {'pose_topic': LaunchConfiguration('pose_topic')},
            {'landmark_topic': LaunchConfiguration('landmark_topic')},
        ],
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription(launch_arguments + [landmark_calibrator_node])

