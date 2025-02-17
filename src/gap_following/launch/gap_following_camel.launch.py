import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'gap_following'

follow_the_gap = os.path.join(
                get_package_share_directory(package_name),'config','follow_the_gap_params.yaml')

def generate_launch_description():

    #Create node 
    dist_camel = Node(
        package=package_name,
        executable='gap_following',
        parameters=[follow_the_gap]
    )


    # Launch!
    return LaunchDescription([
            dist_camel
    ])