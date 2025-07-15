from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='predictive_energy_manager',
            executable='predictive_energy_manager_node',
            name='predictive_energy_manager',
            output='screen',
        ),
    ])
