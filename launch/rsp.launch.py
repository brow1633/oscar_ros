import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

import xacro

def launch_setup(context, *args, **kwargs):
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    subdir = LaunchConfiguration('xacro_subdir')
    subdir_val = subdir.perform(context)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('oscar_ros'))
    xacro_file = os.path.join(pkg_path,'description',subdir_val,'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    return [node_robot_state_publisher]


def generate_launch_description():

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
  	DeclareLaunchArgument(
	    'xacro_subdir',
 	    default_value='',
   	    description='Subdirectory to find Xacro files'),
	OpaqueFunction(function=launch_setup)
    ])
