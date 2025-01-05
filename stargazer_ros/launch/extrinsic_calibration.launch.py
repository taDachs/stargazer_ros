from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static Transform Publisher Node
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot2camTransform',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '+1.570796327', '--pitch', '0',
                   '--yaw', '0', '--child-frame-id', 'camera', '--frame-id', 'vehicle']
    )

    # Combine all components into a LaunchDescription
    return LaunchDescription([static_transform_publisher])
