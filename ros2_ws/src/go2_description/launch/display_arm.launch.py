import os
import subprocess
import sys

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_ARM_VARIANT = 'rm_75'
DEFAULT_CAMERA = 'd435i'
DEFAULT_GRIPPER = 'ag95'


def _realsense_share_default():
    try:
        return get_package_share_directory('realsense2_description')
    except Exception:
        return ''


def _gripper_share_default(repo_root):
    return os.environ.get(
        'DH_GRIPPER_SHARE',
        os.path.join(repo_root, 'third_party', 'dh_gripper_ros',
                     'dh_robotics_ag95_gripper',
                     'dh_robotics_ag95_description'))


def _repo_root(pkg_share):
    return os.path.abspath(os.path.join(
        pkg_share, '..', '..', '..', '..', '..'))


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')
    prefix = get_package_prefix('go2_description')
    builder = os.path.join(prefix, 'lib', 'go2_description',
                           'build_arm_model.py')
    default_arm_root = os.environ.get(
        'RM_ROBOT_ROOT', os.path.join(_repo_root(pkg_share), 'third_party',
                                      'ros2_rm_robot'))

    def _arg(context, name):
        return context.perform_substitution(LaunchConfiguration(name))

    def _robot_nodes(context):
        arm_root = _arg(context, 'arm_root')
        variant = _arg(context, 'arm_variant')
        arm_share = os.path.join(arm_root, 'rm_description')
        arm_urdf = os.path.join(arm_share, 'urdf', variant + '.urdf')
        if not os.path.exists(arm_urdf):
            raise RuntimeError('arm urdf not found: %s' % arm_urdf)
        command = [
            sys.executable, builder,
            '--go2-urdf', os.path.join(pkg_share, 'urdf',
                                       'go2_description.urdf'),
            '--arm-urdf', arm_urdf,
            '--go2-share', pkg_share,
            '--arm-share', arm_share,
            '--plate-xyz', _arg(context, 'plate_xyz'),
            '--arm-xyz', _arg(context, 'arm_xyz'),
            '--arm-rpy', _arg(context, 'arm_rpy'),
            '--tool-xyz', _arg(context, 'tool_xyz'),
            '--tool-rpy', _arg(context, 'tool_rpy'),
            '--camera', _arg(context, 'camera'),
            '--camera-xyz', _arg(context, 'camera_xyz'),
            '--camera-rpy', _arg(context, 'camera_rpy'),
            '--gripper', _arg(context, 'gripper'),
            '--gripper-xyz', _arg(context, 'gripper_xyz'),
            '--gripper-rpy', _arg(context, 'gripper_rpy'),
        ]
        realsense_share = _arg(context, 'realsense_share')
        if realsense_share:
            command += ['--realsense-share', realsense_share]
        gripper_share = _arg(context, 'gripper_share')
        if gripper_share:
            command += ['--gripper-share', gripper_share]
        result = subprocess.run(command, capture_output=True, text=True,
                                check=True)
        description = result.stdout
        return [
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{'robot_description': description}],
                 output='screen'),
            Node(package='joint_state_publisher_gui',
                 executable='joint_state_publisher_gui',
                 condition=IfCondition(LaunchConfiguration('use_gui')),
                 output='screen'),
            Node(package='joint_state_publisher',
                 executable='joint_state_publisher',
                 condition=UnlessCondition(LaunchConfiguration('use_gui')),
                 output='screen'),
            Node(package='rviz2',
                 executable='rviz2',
                 arguments=['-d', os.path.join(pkg_share, 'rviz', 'go2.rviz')],
                 condition=IfCondition(LaunchConfiguration('use_rviz')),
                 output='screen'),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Start joint_state_publisher_gui'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('arm_root', default_value=default_arm_root,
                              description='ros2_rm_robot 仓库路径'),
        DeclareLaunchArgument('arm_variant', default_value=DEFAULT_ARM_VARIANT,
                              description='rm_75 | rm_75_6f | rm_75_6fb'),
        DeclareLaunchArgument('plate_xyz', default_value='0 0 0.065',
                              description='载荷挂点相对 base 的位置'),
        DeclareLaunchArgument('arm_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('arm_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tool_xyz', default_value='0 0 0',
                              description='夹爪法兰帧 tool0 相对 Link7'),
        DeclareLaunchArgument('tool_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('camera', default_value=DEFAULT_CAMERA,
                              description='none | d435i | d435 | d415 ...'),
        DeclareLaunchArgument('camera_xyz', default_value='0 0.06 -0.02',
                              description='相机支架相对 Link7(法兰旁侧挂)'),
        DeclareLaunchArgument('camera_rpy', default_value='0 -1.5708 0',
                              description='相机支架朝向(默认看向 tool 方向)'),
        DeclareLaunchArgument(
            'realsense_share',
            default_value=_realsense_share_default(),
            description='realsense2_description 路径'),
        DeclareLaunchArgument('gripper', default_value=DEFAULT_GRIPPER,
                              description='none | ag95 | ag145'),
        DeclareLaunchArgument('gripper_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('gripper_rpy', default_value='0 -1.5708 0',
                              description='夹爪法兰对接姿态(默认让爪子沿 tool 轴)'),
        DeclareLaunchArgument(
            'gripper_share',
            default_value=_gripper_share_default(
                _repo_root(get_package_share_directory('go2_description'))),
            description='DH 夹爪描述包路径'),
        OpaqueFunction(function=_robot_nodes),
    ])
