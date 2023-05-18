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

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'True', 'use_ros2_control': 'True'}.items()
            )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                        controller_params_file, {'use_sim_time': True}]
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
    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(package_name), 'config/ekf.yaml'), {'use_sim_time': True}]
            )

    delayed_robot_localization_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=diff_drive_spawner,
                on_start=[robot_localization_node],
                )
            )


    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_robot_localization_spawner
        ])
