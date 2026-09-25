import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder('rm_75_description', package_name='rm_75_config')
        .to_moveit_configs()
    )
    rviz_config = os.path.join(get_package_share_directory('rm_75_config'),
                               'config', 'moveit.rviz')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        move_group,
        rviz,
    ])
