from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("teleop_twist_joy"), '/launch/teleop-launch.py'
            ]),
            launch_arguments={'config_filepath': get_package_share_directory("oscar_ros") + '/config/sim/taranis.config.yaml'}.items(),
        ),
        ])
