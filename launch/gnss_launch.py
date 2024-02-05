from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss',
            executable='gnss_node',
            name='gnss_node',
            output='screen',
        )
    ])
