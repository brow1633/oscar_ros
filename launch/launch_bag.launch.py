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
    sim_time = True

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': str(sim_time), 'xacro_subdir': subdir}.items()
            )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config',subdir,'my_controllers.yaml')

    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                        controller_params_file]
            )

    delayed_controller_manager = TimerAction(period=0.5, actions=[controller_manager])

    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
            )

    delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[diff_drive_spawner],
                )
            )

    joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
            )

    delayed_joint_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_broad_spawner],
                )
            )
    robot_localization_node_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}],
	    remappings=[('odometry/filtered', 'odometry/local')]
            )

    robot_localization_node_map = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}],
	    remappings=[('odometry/filtered', 'odometry/global')]
            )
    delayed_localization_map = TimerAction(period=3.0, actions=[robot_localization_node_map])

    navsat_transform_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}],
            remappings=[('imu', 'zed2i/zed_node/imu/data'),
                        ('odometry/filtered', 'odometry/global')]
            )

#    ublox_gps_node = Node(
#	    package='ublox_gps',
#	    executable='ublox_gps_node',
#	    name='ublox_gps_node',
#	    namespace='gps',
#	    parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ardusimple.yaml')]
#	    )

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
        rsp,
        delayed_controller_manager,
        #delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        robot_localization_node_odom,
	    robot_localization_node_map,
        navsat_transform_node,
        #delayed_localization_map,
        #septentrio_gps_node
        ])
