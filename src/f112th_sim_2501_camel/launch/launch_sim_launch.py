# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='f112th_sim_2501_camel' #<--- CHANGE ME


    #with the help of https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/}

    world_file_name = 'map' # <---Change world if necessary (Te estoy viendo usuario 0_0 )
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')



    gazebo_params_path = os.path.join(
                  get_package_share_directory(package_name),'config','gazebo_params.yaml')

    robot_description_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_description_launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
             )
    

    

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ackerman_drive'],
                        output='screen')
    

    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )
    
    # Launch them all!
    return LaunchDescription([
        robot_description_launch,
        gazebo,
        spawn_entity,
        twist_mux_node
    ])

