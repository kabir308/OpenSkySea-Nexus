import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Directory containing the Gazebo models
    models_dir = os.path.join(get_package_share_directory('simulation_bringup'), '..', '..', '..', 'sim', 'gazebo', 'models')

    # Path to the SDF file for the rover model
    sdf_file = os.path.join(models_dir, 'rover', 'model.sdf')

    # Node to publish the robot description, using a namespace for the rover
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='rover',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(sdf_file).read()
        }]
    )

    # Node to spawn the entity in Gazebo, specifying the namespaced topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-topic', 'rover/robot_description', '-x', '2', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    # The translator node, configured for MAVROS to control the rover
    translator_node = Node(
        package='nexus_communication_protocol',
        executable='ncp_translator_node',
        name='rover_translator',
        output='screen',
        parameters=[{
            'translator_plugin': 'nexus_communication_protocol/MavrosTranslator'
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity,
        translator_node,
    ])
