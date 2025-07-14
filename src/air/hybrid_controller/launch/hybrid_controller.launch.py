from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hybrid_controller',
            executable='hybrid_controller_node',
            name='hybrid_controller',
            output='screen',
        ),
    ])
