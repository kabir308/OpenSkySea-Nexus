from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sea_simulator',
            executable='sea_simulator_node',
            name='sea_simulator',
            output='screen',
        ),
    ])
