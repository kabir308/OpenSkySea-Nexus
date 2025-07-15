from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_channel_stack',
            executable='multi_channel_stack_node',
            name='satellite_channel',
            parameters=[{'channels': ['satellite']}]
        ),
        Node(
            package='multi_channel_stack',
            executable='multi_channel_stack_node',
            name='lorawan_channel',
            parameters=[{'channels': ['lorawan']}]
        ),
    ])
