import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOINTS = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]


def _gz_robot_description(pkg_share):
    with open(os.path.join(pkg_share, 'urdf', 'go2_description.urdf')) as f:
        urdf = f.read()
    urdf = urdf.replace('package://go2_description/', 'file://' + pkg_share + '/')

    ros2_control = [
        '<ros2_control name="Go2GazeboSystem" type="system">',
        '<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>',
    ]
    for joint in JOINTS:
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
        '<parameters>'
        + os.path.join(pkg_share, 'config', 'go2_controllers.yaml')
        + '</parameters>'
        '<controller_manager_name>controller_manager</controller_manager_name>'
        '</plugin></gazebo>')

    feet = ''.join(
        f'<gazebo reference="{link}">'
        '<mu1>1.5</mu1><mu2>1.5</mu2><kp>100000</kp><kd>100</kd>'
        '</gazebo>'
        for link in ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'))

    pose_plugin = (
        '<gazebo><plugin filename="ignition-gazebo-pose-publisher-system" '
        'name="ignition::gazebo::systems::PosePublisher">'
        '<publish_link_pose>false</publish_link_pose>'
        '<publish_model_pose>true</publish_model_pose>'
        '</plugin></gazebo>')

    return urdf.replace(
        '</robot>',
        ''.join(ros2_control) + plugin + feet + pose_plugin + '</robot>')


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')
    robot_description = _gz_robot_description(pkg_share)

    use_gui = LaunchConfiguration('use_gui')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')
    walk = LaunchConfiguration('walk')

    gz_plugin_path = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    gz_env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    spawner_jgpc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Start the Gazebo GUI client'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Gazebo world to load'),
        DeclareLaunchArgument('spawn_z', default_value='0.45',
                              description='Spawn height of the Go2'),
        DeclareLaunchArgument('walk', default_value='false',
                              description='Start the open-loop trot gait node'),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '3', world],
            output='screen',
            additional_env=gz_env,
            condition=IfCondition(use_gui),
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', world],
            output='screen',
            additional_env=gz_env,
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'go2',
                       '-z', spawn_z],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                       '/model/go2/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose'],
            output='screen',
        ),
        spawner_jsb,
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_jsb,
                          on_exit=[spawner_jgpc]),
        ),
        Node(
            package='go2_description',
            executable='go2_trot.py',
            condition=IfCondition(walk),
            output='screen',
        ),
    ])
