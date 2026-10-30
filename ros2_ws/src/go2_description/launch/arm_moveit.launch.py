"""MoveIt2 for the assembled arm bench (RM75 + D435i + AG95).

The arm_bench.launch.py run writes the assembled URDF (without the Gazebo
world link) to /tmp/arm_bench_moveit.urdf.  MoveIt uses that same URDF, so its
collision model includes the wrist camera and the gripper instead of only the
bare RM75.  The semantic description is the rm_75_config SRDF plus collision
pairs for the fixed camera/gripper attachments.
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_FILE = '/tmp/arm_bench_moveit.urdf'
SRDF_FILE = '/tmp/arm_bench_moveit.srdf'

# Pairs that are rigidly attached (or the prismatic fingers against their
# parent) and must not be treated as self-collisions.
ATTACHMENT_PAIRS = [
    ('Link7', 'tool0', 'Adjacent'),
    ('Link7', 'camera_mount_link', 'Adjacent'),
    ('Link7', 'camera_link', 'Never'),
    ('Link7', 'gripper_base_link', 'Never'),
    ('Link7', 'gripper_finger1_link', 'Never'),
    ('Link7', 'gripper_finger2_link', 'Never'),
    ('tool0', 'camera_mount_link', 'Never'),
    ('tool0', 'camera_link', 'Never'),
    ('tool0', 'gripper_base_link', 'Adjacent'),
    ('tool0', 'gripper_finger1_link', 'Never'),
    ('tool0', 'gripper_finger2_link', 'Never'),
    ('camera_mount_link', 'camera_link', 'Adjacent'),
    ('camera_mount_link', 'gripper_base_link', 'Never'),
    ('camera_mount_link', 'gripper_finger1_link', 'Never'),
    ('camera_mount_link', 'gripper_finger2_link', 'Never'),
    ('camera_link', 'gripper_base_link', 'Never'),
    ('camera_link', 'gripper_finger1_link', 'Never'),
    ('camera_link', 'gripper_finger2_link', 'Never'),
    ('gripper_base_link', 'gripper_finger1_link', 'Adjacent'),
    ('gripper_base_link', 'gripper_finger2_link', 'Adjacent'),
]


def _make_srdf(urdf_text):
    """rm_75 SRDF + collision pairs for the assembled camera/gripper."""
    root_urdf = ET.fromstring(urdf_text)
    links = {link.get('name') for link in root_urdf.findall('link')}
    srdf_path = os.path.join(
        get_package_share_directory('rm_75_config'),
        'config', 'rm_75_description.srdf')
    root = ET.parse(srdf_path).getroot()
    existing = {(d.get('link1'), d.get('link2'))
                for d in root.findall('disable_collisions')}
    for link1, link2, reason in ATTACHMENT_PAIRS:
        if link1 not in links or link2 not in links:
            continue
        if (link1, link2) in existing or (link2, link1) in existing:
            continue
        ET.SubElement(root, 'disable_collisions', {
            'link1': link1, 'link2': link2, 'reason': reason})
    text = ET.tostring(root, encoding='unicode')
    with open(SRDF_FILE, 'w') as stream:
        stream.write(text)
    return text


def generate_launch_description():
    from moveit_configs_utils import MoveItConfigsBuilder

    # The default path matches arm_bench.launch.py's moveit_urdf argument.
    # Set ARM_BENCH_MOVEIT_URDF if that path was changed.
    requested = os.environ.get('ARM_BENCH_MOVEIT_URDF', URDF_FILE)
    urdf_file = os.path.expanduser(requested)
    if not os.path.exists(urdf_file):
        raise RuntimeError(
            'assembled URDF not found: %s\n'
            'start "ros2 launch go2_description arm_bench.launch.py" first'
            % urdf_file)
    with open(urdf_file) as stream:
        urdf_text = stream.read()
    srdf_text = _make_srdf(urdf_text)

    moveit_config = (
        MoveItConfigsBuilder('rm_75_description', package_name='rm_75_config')
        .to_moveit_configs()
    )
    params = moveit_config.to_dict()
    params['robot_description'] = urdf_text
    params['robot_description_semantic'] = srdf_text

    rviz_config = os.path.join(get_package_share_directory('rm_75_config'),
                               'config', 'moveit.rviz')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True,
            'robot_description': urdf_text,
            'robot_description_semantic': srdf_text,
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        move_group,
        rviz,
    ])
