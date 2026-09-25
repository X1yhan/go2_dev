from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo = Node(
        package='go2_vision',
        executable='visual_servo.py',
        parameters=[{'use_sim_time': True,
                     'd_dock': LaunchConfiguration('d_dock'),
                     'vx_max': LaunchConfiguration('vx_max'),
                     'wz_max': LaunchConfiguration('wz_max'),
                     'kp_yaw': LaunchConfiguration('kp_yaw'),
                     'kp_dist': LaunchConfiguration('kp_dist')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('d_dock', default_value='1.2'),
        DeclareLaunchArgument('vx_max', default_value='0.30'),
        DeclareLaunchArgument('wz_max', default_value='0.8'),
        DeclareLaunchArgument('kp_yaw', default_value='1.2'),
        DeclareLaunchArgument('kp_dist', default_value='0.6'),
        servo,
    ])
