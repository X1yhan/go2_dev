import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_eval = LaunchConfiguration('eval')
    use_hold = LaunchConfiguration('hold')
    trajectory = LaunchConfiguration('trajectory')
    speed = LaunchConfiguration('speed')
    amplitude = LaunchConfiguration('amplitude')
    radius = LaunchConfiguration('radius')
    center_x = LaunchConfiguration('center_x')
    center_y = LaunchConfiguration('center_y')
    world_name = LaunchConfiguration('world_name')
    target_name = LaunchConfiguration('target_name')

    director = Node(
        package='go2_vision',
        executable='target_director.py',
        parameters=[{'use_sim_time': True,
                     'world_name': world_name,
                     'target_name': target_name,
                     'trajectory': trajectory,
                     'speed': speed,
                     'amplitude': amplitude,
                     'radius': radius,
                     'center_x': center_x,
                     'center_y': center_y,
                     'publish_ground_truth': True}],
        output='screen',
    )
    detector = Node(
        package='go2_vision',
        executable='ball_detector.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    localizer = Node(
        package='go2_vision',
        executable='target_localizer.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    eval_monitor = Node(
        package='go2_vision',
        executable='eval_monitor.py',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_eval),
        output='screen',
    )
    stand_keeper = Node(
        package='go2_vision',
        executable='stand_keeper.py',
        condition=IfCondition(use_hold),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('trajectory', default_value='sine',
                              description='static|line|sine|circle'),
        DeclareLaunchArgument('speed', default_value='0.5'),
        DeclareLaunchArgument('amplitude', default_value='1.0'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('center_x', default_value='1.5'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('world_name', default_value='go2_sim'),
        DeclareLaunchArgument('target_name', default_value='red_ball'),
        DeclareLaunchArgument('eval', default_value='true'),
        DeclareLaunchArgument('hold', default_value='true',
                              description='Keep the gait node standing'),
        director,
        detector,
        localizer,
        eval_monitor,
        stand_keeper,
    ])
