import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Directory containing the Gazebo models
    models_dir = os.path.join(get_package_share_directory('simulation_bringup'), '..', '..', '..', 'sim', 'gazebo', 'models')

    # Path to the SDF file for the Iris model
    sdf_file = os.path.join(models_dir, 'quadcopter', 'model.sdf')

    # Node to publish the robot description (content of the SDF file)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='iris',
        output='screen',
        parameters=[{
            'use_sim_time': True, # Use simulation time
            'robot_description': open(sdf_file).read()
        }]
    )

    # Node to spawn the entity in Gazebo. It listens to the namespaced topic.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'iris', '-topic', 'iris/robot_description'],
        output='screen'
    )

    # The controller node for the quadcopter
    controller_node = Node(
        package='hybrid_controller',
        executable='hybrid_controller_node',
        name='hybrid_controller',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity,
        controller_node,
    ])
