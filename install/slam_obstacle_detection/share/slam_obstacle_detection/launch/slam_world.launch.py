import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    # Obtener la ubicación del mundo mejorado
    world_file = os.path.join(get_package_share_directory('mi_mundo'), 'worlds', 'nuevo_mapa.world')

    # Posición inicial aleatoria
    x_pos = str(round(random.uniform(-2.0, 2.0), 2))
    y_pos = str(round(random.uniform(-2.0, 2.0), 2))
    z_pos = 0.1
    
    # Lanzar Gazebo con el mundo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )
    
    # Lanzar el robot en una posición aleatoria
    spawn_robot = TimerAction(
    	period=5.0,
    	actions=[
    	    Node(
        	package='gazebo_ros',
        	executable='spawn_entity.py',
        	arguments=[
            	    '-entity', 'turtlebot3',
            	    '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            	    '-x', str(x_pos),
            	    '-y', str(y_pos),
            	    '-z', '0.1',
        	],
        	output='screen'
    	     )
	 ]
    )
    # Nodos personalizados


    tf_static_publisher = launch_ros.actions.Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_scan"]
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_node',
        output='screen',
        parameters=[{
            "use_sim_time": True,
            "slam_params_file": os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_sync.yaml')
        }],
    )    


    vision_node = Node(
        package='slam_obstacle_detection',
        executable='vision_node',
        output='screen',
	parameters=[{"use_sim_time":True}]
    )

    move_robot = Node(
        package='slam_obstacle_detection',
        executable='move_robot',
        output='screen'
    )

    wall_detector = Node(
        package='slam_obstacle_detection',
        executable='wall_detector',
        output='screen'
    )

    return LaunchDescription([
        gazebo,

        spawn_robot,
        slam_node,
        vision_node,
        move_robot,
        wall_detector,
    ])
