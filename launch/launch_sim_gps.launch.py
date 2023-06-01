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
    subdir = 'sim'
    sim_time = True

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[os.path.join(get_package_share_directory(package_name), 'config', subdir, 'twist_mux.yaml'), {'use_sim_time': sim_time}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
            )
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
                        controller_params_file, {'use_sim_time': sim_time}]
            )

    delayed_controller_manager = TimerAction(period=0.5, actions=[controller_manager])

    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
            remappings=[('diff_cont/cmd_vel_unstamped', 'cmd_vel')]
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
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}]
            )

    robot_localization_node_map = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')]
            )

    navsat_transform_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name),'config',subdir,'ekf_gps.yaml'), {'use_sim_time': sim_time}],
            remappings=[('imu/data', 'imu'),
                        ('gps/fix', 'gps'),
                        ('odometry/filtered', 'odometry/global')]
            )



    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        robot_localization_node_odom,
        robot_localization_node_map,
        navsat_transform_node
        ])
