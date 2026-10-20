#!/usr/bin/env python3
"""Standalone RM75 arm bench.

Spawns an RM75 arm on a cube table, with a D435i RGB-D camera and a
two-finger gripper mounted on the end effector.

Usage:
    ros2 launch go2_description arm_bench.launch.py
    ros2 launch go2_description arm_bench.launch.py use_gui:=false
    ros2 launch go2_description arm_bench.launch.py camera:=none gripper:=none
"""
import os
import subprocess
import sys

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
GRIPPER_JOINTS = ['gripper_finger1_joint', 'gripper_finger2_joint']

DEFAULT_ARM_VARIANT = 'rm_75'
DEFAULT_CAMERA = 'd435i'
DEFAULT_GRIPPER = 'ag95'
DEFAULT_WORLD_NAME = 'arm_bench'
MODEL_NAME = 'arm_bench'
CAMERA_TOPIC = '/arm/camera'


def _repo_root(pkg_share):
    return os.path.abspath(os.path.join(
        pkg_share, '..', '..', '..', '..', '..'))


def _realsense_share_default():
    try:
        return get_package_share_directory('realsense2_description')
    except Exception:
        return ''


def _gripper_share_default(pkg_share):
    return os.environ.get(
        'DH_GRIPPER_SHARE',
        os.path.join(_repo_root(pkg_share), 'third_party', 'dh_gripper_ros',
                     'dh_robotics_ag95_gripper',
                     'dh_robotics_ag95_description'))


def _arg(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _truthy(context, name):
    return _arg(context, name).lower() in ('1', 'true', 'yes', 'on')


def _sensor_xml(width, height, fps, hfov, near, far, visualize):
    return (
        '<gazebo reference="camera_link">'
        '<sensor name="arm_rgbd_camera" type="rgbd_camera">'
        '<camera>'
        f'<horizontal_fov>{hfov}</horizontal_fov>'
        f'<image><width>{width}</width><height>{height}</height></image>'
        f'<clip><near>{near}</near><far>{far}</far></clip>'
        '</camera>'
        '<always_on>1</always_on>'
        f'<update_rate>{fps}</update_rate>'
        f'<visualize>{visualize}</visualize>'
        f'<topic>{CAMERA_TOPIC}</topic>'
        '</sensor>'
        '</gazebo>')


def _gz_robot_description(urdf, pkg_share, gripper, camera, width, height,
                          fps, hfov, near, far, visualize):
    joints = ARM_JOINTS + (GRIPPER_JOINTS if gripper != 'none' else [])
    controller_yaml = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')

    ros2_control = [
        '<ros2_control name="ArmBenchSystem" type="system">',
        '<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>',
    ]
    for joint in joints:
        ros2_control.append(
            f'<joint name="{joint}">'
            '<command_interface name="position"/>'
            '<state_interface name="position"/>'
            '<state_interface name="velocity"/>'
            '</joint>')
    ros2_control.append('</ros2_control>')

    plugin = (
        '<gazebo><plugin filename="libgz_ros2_control-system.so" '
        'name="gz_ros2_control::GazeboSimROS2ControlPlugin">'
        '<parameters>' + controller_yaml + '</parameters>'
        '<controller_manager_name>controller_manager</controller_manager_name>'
        '</plugin></gazebo>')

    # The arm base is fixed to the world, but keep the official Gazebo demo
    # convention to avoid gravity acting on the camera/gripper attachments.
    gravity_off = ''.join(
        f'<gazebo reference="{link}"><gravity>false</gravity></gazebo>'
        for link in ('base_link', 'Link1', 'Link2', 'Link3', 'Link4',
                     'Link5', 'Link6', 'Link7', 'tool0',
                     'camera_mount_link', 'camera_link',
                     'gripper_base_link', 'gripper_finger1_link',
                     'gripper_finger2_link')
        if f'name="{link}"' in urdf)

    extra = ''.join(ros2_control) + plugin + gravity_off
    if camera != 'none':
        extra += _sensor_xml(width, height, fps, hfov, near, far, visualize)
    return urdf.replace('</robot>', extra + '</robot>')


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')
    prefix = get_package_prefix('go2_description')
    builder = os.path.join(prefix, 'lib', 'go2_description',
                           'build_arm_model.py')
    default_arm_root = os.environ.get(
        'RM_ROBOT_ROOT', os.path.join(_repo_root(pkg_share), 'third_party',
                                      'ros2_rm_robot'))

    gz_plugin_path = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    gz_env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }

    def _robot_nodes(context):
        arm_root = _arg(context, 'arm_root')
        variant = _arg(context, 'arm_variant')
        camera = _arg(context, 'camera')
        gripper = _arg(context, 'gripper')
        arm_share = os.path.join(arm_root, 'rm_description')
        arm_urdf = os.path.join(arm_share, 'urdf', variant + '.urdf')
        if not os.path.exists(arm_urdf):
            raise RuntimeError('arm urdf not found: %s' % arm_urdf)

        command = [
            sys.executable, builder,
            '--standalone',
            '--arm-urdf', arm_urdf,
            '--arm-share', arm_share,
            '--tool-xyz', _arg(context, 'tool_xyz'),
            '--tool-rpy', _arg(context, 'tool_rpy'),
            '--camera', camera,
            '--camera-xyz', _arg(context, 'camera_xyz'),
            '--camera-rpy', _arg(context, 'camera_rpy'),
            '--gripper', gripper,
            '--gripper-xyz', _arg(context, 'gripper_xyz'),
            '--gripper-rpy', _arg(context, 'gripper_rpy'),
        ]
        realsense_share = _arg(context, 'realsense_share')
        if realsense_share:
            command += ['--realsense-share', realsense_share]
        gripper_share = _arg(context, 'gripper_share')
        if gripper_share:
            command += ['--gripper-share', gripper_share]
        if gripper != 'none' and _truthy(context, 'gripper_simple'):
            command += ['--gripper-simple']

        result = subprocess.run(command, capture_output=True, text=True,
                                check=True)
        urdf = _gz_robot_description(
            result.stdout, pkg_share, gripper, camera,
            _arg(context, 'camera_width'), _arg(context, 'camera_height'),
            _arg(context, 'camera_fps'), _arg(context, 'camera_hfov'),
            _arg(context, 'camera_clip_near'),
            _arg(context, 'camera_clip_far'),
            _arg(context, 'camera_visualize'))

        nodes = [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': urdf,
                             'use_sim_time': True}],
                output='screen',
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-world', _arg(context, 'gz_world_name'),
                           '-topic', 'robot_description',
                           '-name', MODEL_NAME,
                           '-x', _arg(context, 'spawn_x'),
                           '-y', _arg(context, 'spawn_y'),
                           '-z', _arg(context, 'spawn_z'),
                           '-Y', _arg(context, 'spawn_yaw')],
                output='screen',
            ),
        ]

        if camera != 'none':
            bridge_args = [
                f'{CAMERA_TOPIC}/image@sensor_msgs/msg/Image'
                '[ignition.msgs.Image',
                f'{CAMERA_TOPIC}/depth_image@sensor_msgs/msg/Image'
                '[ignition.msgs.Image',
                f'{CAMERA_TOPIC}/camera_info@sensor_msgs/msg/CameraInfo'
                '[ignition.msgs.CameraInfo',
                f'{CAMERA_TOPIC}/points@sensor_msgs/msg/PointCloud2'
                '[ignition.msgs.PointCloudPacked',
                '/model/target_ball/odometry@nav_msgs/msg/Odometry'
                '[ignition.msgs.Odometry',
            ]
            nodes.append(TimerAction(period=2.0, actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=bridge_args,
                    output='screen',
                ),
            ]))
            # Gazebo scopes the sensor frame by model/link/sensor.  The RGB-D
            # image uses the D435i optical convention, so attach that scoped
            # frame to the URDF depth optical frame.
            nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--frame-id', 'camera_depth_optical_frame',
                           '--child-frame-id',
                           f'{MODEL_NAME}/Link7/arm_rgbd_camera'],
                output='screen',
            ))
        return nodes

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rm_group_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager-timeout', '60'],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('gripper'), "' != 'none'"])),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz2 with the arm/camera view'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'arm_bench.sdf'),
            description='Gazebo world with the cube table'),
        DeclareLaunchArgument('gz_world_name', default_value=DEFAULT_WORLD_NAME,
                              description='World name inside the SDF'),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_z', default_value='0.6',
                              description='Arm base height (table top)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0'),
        DeclareLaunchArgument('arm_root', default_value=default_arm_root,
                              description='ros2_rm_robot repo path'),
        DeclareLaunchArgument('arm_variant', default_value=DEFAULT_ARM_VARIANT,
                              description='rm_75 | rm_75_6f | rm_75_6fb'),
        DeclareLaunchArgument('tool_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('tool_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('camera', default_value=DEFAULT_CAMERA,
                              description='none | d435i | d435 | d415 ...'),
        DeclareLaunchArgument('camera_xyz', default_value='-0.12 0 0',
                              description='Camera bracket in Link7 frame'),
        DeclareLaunchArgument('camera_rpy', default_value='0 -1.5708 0'),
        DeclareLaunchArgument('realsense_share',
                              default_value=_realsense_share_default()),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='360'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_hfov', default_value='1.518',
                              description='D435i depth horizontal FOV (87 deg)'),
        DeclareLaunchArgument('camera_clip_near', default_value='0.05',
                              description='Camera near clip (tabletop use)'),
        DeclareLaunchArgument('camera_clip_far', default_value='10.0'),
        DeclareLaunchArgument('camera_visualize', default_value='false',
                              description='Show camera image in Gazebo GUI'),
        DeclareLaunchArgument('gripper', default_value=DEFAULT_GRIPPER,
                              description='none | ag95 | ag145'),
        DeclareLaunchArgument('gripper_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('gripper_rpy', default_value='0 -1.5708 0'),
        DeclareLaunchArgument('gripper_share',
                              default_value=_gripper_share_default(pkg_share)),
        DeclareLaunchArgument('gripper_simple', default_value='true',
                              description='Use simplified parallel gripper '
                                          'for physics'),
        DeclareLaunchArgument('start_forward', default_value='false',
                              description='Move the arm to the forward ready '
                                          'pose on startup (default: stay at '
                                          'the natural upright zero pose)'),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '3',
                 LaunchConfiguration('world')],
            output='screen',
            additional_env=gz_env,
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', '-v', '3',
                 LaunchConfiguration('world')],
            output='screen',
            additional_env=gz_env,
            condition=UnlessCondition(LaunchConfiguration('use_gui')),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
        ),
        OpaqueFunction(function=_robot_nodes),
        spawner_jsb,
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_jsb,
                          on_exit=[spawner_arm]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_arm,
                          on_exit=[spawner_gripper]),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz',
                                          'arm_bench.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
        Node(
            package='go2_description',
            executable='arm_bench_home.py',
            condition=IfCondition(LaunchConfiguration('start_forward')),
            output='screen',
        ),
    ])
