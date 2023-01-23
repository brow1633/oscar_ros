import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name='oscar_ros'

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
            )
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                '-entity', 'oscar_ros'],
            output='screen')


    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["diff_cont"],
            )

    joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_broad"],
            )

    delayed_diff_drive_spawner = TimerAction(period=10.0, actions=[diff_drive_spawner, joint_broad_spawner])

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        ])
