from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_codesys_bridge',
            executable='bridge_node',
            name='ros2_codesys_bridge',
            output='screen',
            parameters=[
                {'base_frame': 'base_footprint'},
                {'odom_frame': 'odom'},
                {'update_rate': 50.0}
            ]
        )
    ])