import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'homecleaner_description'
    pkg_path = get_package_share_directory(package_name)
    
    # URDF ve World yolları
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    world_path = os.path.join(pkg_path, 'worlds', 'house.world')

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True}]
    )

    # Gazebo Launch (World ile)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # Robotu Spawn Et
    # Robotu Spawn Et (Evin içinde başlatmak için x,y,z verildi)
    # Robotu Spawn Et (X=5, Y=5 koordinatına)
    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'homecleanerbot',
        '-x', '5.0',
        '-y', '5.0',
        '-z', '0.1',
        '-Y', '-1.57'
    ],
    output='screen'
)


    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
