import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    package_name='oscar_ros'
    subdir = 'real'
    sim_time = False

    septentrio_gps_node = Node(
            package='septentrio_gnss_driver',
            executable='septentrio_gnss_driver_node',
            name='septentrio_gnss_driver',
            emulate_tty=True,
            sigterm_timeout='20',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'septentrio.yaml')],
  	    remappings=[('navsatfix', 'gps/fix')])

    # Launch them all!
    return LaunchDescription([
        septentrio_gps_node,
        ])
