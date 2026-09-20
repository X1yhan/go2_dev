#!/usr/bin/env python3
"""Standalone D435i vision bench.

Pipeline:  Gazebo RGBD camera -> ros_gz bridge -> color detection
           -> depth fusion -> 3D position (camera + world) -> eval vs GT

Usage:
    ros2 launch vision_sim vision_sim.launch.py
    ros2 launch vision_sim vision_sim.launch.py use_gui:=false rviz:=false
    ros2 launch vision_sim vision_sim.launch.py trajectory:=circle speed:=0.7
"""
import os
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CAMERA_ROOT_LINK = 'camera_mount'
MODEL_NAME = 'd435i_vision'
WORLD_NAME = 'vision_bench'
CAMERA_TOPIC = '/viz/camera'


def _prepare_models(pkg_share, mappings):
    """xacro -> URDF (for ROS/TF) and URDF -> SDF (static, for Gazebo)."""
    xacro_file = os.path.join(pkg_share, 'urdf', 'd435i_vision.urdf.xacro')
    urdf = xacro.process_file(xacro_file, mappings=mappings).toxml()
    rs_share = get_package_share_directory('realsense2_description')
    urdf = urdf.replace('package://realsense2_description/',
                        'file://' + rs_share + '/')

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.urdf', prefix='d435i_vision_',
            delete=False) as f:
        f.write(urdf)
        urdf_path = f.name

    sdf = subprocess.run(['ign', 'sdf', '-p', urdf_path],
                         capture_output=True, text=True, check=True).stdout
    # the camera is a fixed stand in the bench: make the model static
    sdf = sdf.replace(f"<model name='{MODEL_NAME}'>",
                      f"<model name='{MODEL_NAME}'><static>true</static>", 1)
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.sdf', prefix='d435i_vision_',
            delete=False) as f:
        f.write(sdf)
        sdf_path = f.name

    return urdf, sdf_path


def _spawn_camera(context, *args, **kwargs):
    pkg_share = get_package_share_directory('vision_sim')

    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    mappings = {
        'width': arg('width'),
        'height': arg('height'),
        'fps': arg('fps'),
        'hfov': arg('hfov'),
        'topic': CAMERA_TOPIC,
        'visualize': arg('visualize'),
    }
    urdf, sdf_path = _prepare_models(pkg_share, mappings)

    cam_x = LaunchConfiguration('camera_x')
    cam_y = LaunchConfiguration('camera_y')
    cam_z = LaunchConfiguration('camera_z')
    cam_roll = LaunchConfiguration('camera_roll')
    cam_pitch = LaunchConfiguration('camera_pitch')
    cam_yaw = LaunchConfiguration('camera_yaw')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', cam_x, '--y', cam_y, '--z', cam_z,
                       '--roll', cam_roll, '--pitch', cam_pitch,
                       '--yaw', cam_yaw,
                       '--frame-id', 'world',
                       '--child-frame-id', CAMERA_ROOT_LINK],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', WORLD_NAME, '-file', sdf_path,
                       '-name', MODEL_NAME,
                       '-x', cam_x, '-y', cam_y, '-z', cam_z,
                       '-R', cam_roll, '-P', cam_pitch, '-Y', cam_yaw],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_sim')

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('rviz')
    pipeline = LaunchConfiguration('pipeline')
    trajectory = LaunchConfiguration('trajectory')
    speed = LaunchConfiguration('speed')
    pipeline_condition = IfCondition(pipeline)

    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'vision_sim.rviz')

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        f'{CAMERA_TOPIC}/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'{CAMERA_TOPIC}/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'{CAMERA_TOPIC}/camera_info@sensor_msgs/msg/CameraInfo'
        '[ignition.msgs.CameraInfo',
        f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pipeline', default_value='true',
                              description='Start detection / localization / eval'),
        DeclareLaunchArgument('visualize', default_value='false',
                              description='Show the camera image in the Gazebo GUI'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'vision_bench.sdf')),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.8'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.26'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='360'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('hfov', default_value='1.518'),
        DeclareLaunchArgument('trajectory', default_value='sine'),
        DeclareLaunchArgument('speed', default_value='0.5'),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '3', LaunchConfiguration('world')],
            output='screen',
            condition=IfCondition(use_gui),
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', '-v', '3',
                 LaunchConfiguration('world')],
            output='screen',
            condition=UnlessCondition(use_gui),
        ),

        OpaqueFunction(function=_spawn_camera),

        # bridge topics right away, service mapping once the world is up
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=bridge_args,
                output='screen',
            ),
        ]),

        Node(
            package='vision_sim',
            executable='target_director.py',
            parameters=[params_file,
                        {'use_sim_time': True,
                         'trajectory': trajectory,
                         'speed': ParameterValue(speed, value_type=float)}],
            condition=pipeline_condition,
            output='screen',
        ),
        Node(
            package='vision_sim',
            executable='target_detector.py',
            parameters=[params_file, {'use_sim_time': True}],
            condition=pipeline_condition,
            output='screen',
        ),
        Node(
            package='vision_sim',
            executable='target_localizer.py',
            parameters=[params_file, {'use_sim_time': True}],
            condition=pipeline_condition,
            output='screen',
        ),
        Node(
            package='vision_sim',
            executable='eval_monitor.py',
            parameters=[params_file, {'use_sim_time': True}],
            condition=pipeline_condition,
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
