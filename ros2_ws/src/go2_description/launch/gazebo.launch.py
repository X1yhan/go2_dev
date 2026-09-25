import os
import subprocess
import sys

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

JOINTS = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
ARM_DEFAULT_VARIANT = 'rm_75'
DEFAULT_CAMERA = 'd435i'
DEFAULT_GRIPPER = 'ag95'


def _realsense_share_default():
    try:
        return get_package_share_directory('realsense2_description')
    except Exception:
        return ''


def _gripper_share_default():
    repo_root = os.path.abspath(os.path.join(
        get_package_share_directory('go2_description'),
        '..', '..', '..', '..', '..'))
    return os.environ.get(
        'DH_GRIPPER_SHARE',
        os.path.join(repo_root, 'third_party', 'dh_gripper_ros',
                     'dh_robotics_ag95_gripper',
                     'dh_robotics_ag95_description'))

# Go2 built-in sensors (simulated).
# Camera: specs derived from the community front_camera_720 calibration
#         (fx=864.399, fy=863.738, 1280x720 -> HFOV 73.03 deg), see
#         docs/reference/front_camera_720.yaml
# LiDAR: Unitree L1 placeholder (360 deg x 90 deg, 30 m, 10 Hz); exact specs
#        still to be confirmed. The official URDF "radar" link sits inside the
#        head mesh (rays never escape), so the sim mount defaults to the head
#        top (x/y from the radar link, z=0.10) and is overridable via args.
CAMERA_TOPIC = '/go2/camera/image'
CAMERA_INFO_TOPIC = '/go2/camera/camera_info'
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
CAMERA_HFOV = 1.274656

LIDAR_TOPIC = '/go2/lidar'          # gz prefix: scan on <topic>, cloud on <topic>/points
LIDAR_POINTS_TOPIC = '/go2/lidar/points'
LIDAR_LINK = 'utlidar_lidar'
LIDAR_MOUNT_XYZ = '0.28945 0 0.10'
LIDAR_RATE = 10.0
LIDAR_RANGE = 30.0
LIDAR_HSAMPLES = 720
LIDAR_VSAMPLES = 16
LIDAR_VFOV = 1.570796


def _sensor_xml(lidar_xyz):
    half_v = LIDAR_VFOV / 2.0
    return (
        f'<link name="{LIDAR_LINK}"/>'
        f'<joint name="{LIDAR_LINK}_joint" type="fixed">'
        f'<parent link="base"/>'
        f'<child link="{LIDAR_LINK}"/>'
        f'<origin xyz="{lidar_xyz}" rpy="0 0 0"/>'
        f'</joint>'
        '<gazebo reference="front_camera">'
        '<sensor name="front_camera" type="camera">'
        '<camera>'
        f'<horizontal_fov>{CAMERA_HFOV}</horizontal_fov>'
        f'<image><width>{CAMERA_WIDTH}</width>'
        f'<height>{CAMERA_HEIGHT}</height></image>'
        '<clip><near>0.05</near><far>50.0</far></clip>'
        '</camera>'
        '<always_on>1</always_on>'
        f'<update_rate>{CAMERA_FPS}</update_rate>'
        '<visualize>true</visualize>'
        f'<topic>{CAMERA_TOPIC}</topic>'
        '</sensor>'
        '</gazebo>'
        f'<gazebo reference="{LIDAR_LINK}">'
        f'<sensor name="{LIDAR_LINK}" type="gpu_lidar">'
        f'<topic>{LIDAR_TOPIC}</topic>'
        '<always_on>1</always_on>'
        f'<update_rate>{LIDAR_RATE}</update_rate>'
        '<visualize>true</visualize>'
        '<lidar>'
        '<scan>'
        '<horizontal>'
        f'<samples>{LIDAR_HSAMPLES}</samples><resolution>1</resolution>'
        '<min_angle>-3.141593</min_angle><max_angle>3.141593</max_angle>'
        '</horizontal>'
        '<vertical>'
        f'<samples>{LIDAR_VSAMPLES}</samples><resolution>1</resolution>'
        f'<min_angle>{-half_v:.6f}</min_angle>'
        f'<max_angle>{half_v:.6f}</max_angle>'
        '</vertical>'
        '</scan>'
        f'<range><min>0.05</min><max>{LIDAR_RANGE}</max>'
        '<resolution>0.01</resolution></range>'
        '</lidar>'
        '</sensor>'
        '</gazebo>')


def _gz_robot_description(urdf, pkg_share, sensors=False, arm=False,
                          lidar_xyz=LIDAR_MOUNT_XYZ, gripper_joints=None):
    urdf = urdf.replace('package://go2_description/', 'file://' + pkg_share + '/')

    joints = JOINTS + (ARM_JOINTS if arm else []) + list(gripper_joints or [])
    controller_yaml = 'go2_arm_controllers.yaml' if arm else 'go2_controllers.yaml'

    ros2_control = [
        '<ros2_control name="Go2GazeboSystem" type="system">',
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
        '<parameters>'
        + os.path.join(pkg_share, 'config', controller_yaml)
        + '</parameters>'
        '<controller_manager_name>controller_manager</controller_manager_name>'
        '</plugin></gazebo>')

    feet = ''.join(
        f'<gazebo reference="{link}">'
        '<mu1>1.5</mu1><mu2>1.5</mu2><kp>100000</kp><kd>100</kd>'
        '</gazebo>'
        for link in ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'))

    odom_plugin = (
        '<gazebo><plugin filename="ignition-gazebo-odometry-publisher-system" '
        'name="ignition::gazebo::systems::OdometryPublisher">'
        '<odom_frame>world</odom_frame>'
        '<robot_base_frame>base</robot_base_frame>'
        '<dimensions>3</dimensions>'
        '<odom_publish_frequency>30</odom_publish_frequency>'
        '</plugin></gazebo>')

    extra = ''.join(ros2_control) + plugin + feet + odom_plugin
    if sensors:
        extra += _sensor_xml(lidar_xyz)

    return urdf.replace('</robot>', extra + '</robot>')


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')

    use_gui = LaunchConfiguration('use_gui')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')
    walk = LaunchConfiguration('walk')
    sensors = LaunchConfiguration('sensors')
    rviz = LaunchConfiguration('rviz')
    gz_world_name = LaunchConfiguration('gz_world_name')
    arm = LaunchConfiguration('arm')

    gz_plugin_path = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    gz_env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }

    def _truthy(context, name):
        return context.perform_substitution(
            LaunchConfiguration(name)).lower() in ('1', 'true', 'yes', 'on')

    def _robot_nodes(context):
        enabled = _truthy(context, 'sensors')
        arm_on = _truthy(context, 'arm')
        lidar_xyz = ' '.join(
            context.perform_substitution(LaunchConfiguration(axis))
            for axis in ('lidar_x', 'lidar_y', 'lidar_z'))
        with open(os.path.join(pkg_share, 'urdf',
                               'go2_description.urdf')) as handle:
            urdf = handle.read()
        gripper_joints = []
        if arm_on:
            def _arg(name):
                return context.perform_substitution(LaunchConfiguration(name))

            arm_root = _arg('arm_root')
            variant = _arg('arm_variant')
            builder = os.path.join(
                get_package_prefix('go2_description'), 'lib',
                'go2_description', 'build_arm_model.py')
            command = [
                sys.executable, builder,
                '--go2-urdf', os.path.join(pkg_share, 'urdf',
                                           'go2_description.urdf'),
                '--arm-urdf', os.path.join(arm_root, 'rm_description', 'urdf',
                                           variant + '.urdf'),
                '--go2-share', pkg_share,
                '--arm-share', os.path.join(arm_root, 'rm_description'),
                '--plate-xyz', _arg('plate_xyz'),
                '--arm-xyz', _arg('arm_xyz'),
                '--arm-rpy', _arg('arm_rpy'),
                '--tool-xyz', _arg('tool_xyz'),
                '--tool-rpy', _arg('tool_rpy'),
                '--camera', _arg('camera'),
                '--camera-xyz', _arg('camera_xyz'),
                '--camera-rpy', _arg('camera_rpy'),
                '--gripper', _arg('gripper'),
                '--gripper-xyz', _arg('gripper_xyz'),
                '--gripper-rpy', _arg('gripper_rpy'),
            ]
            realsense_share = _arg('realsense_share')
            if realsense_share:
                command += ['--realsense-share', realsense_share]
            gripper_share = _arg('gripper_share')
            if gripper_share:
                command += ['--gripper-share', gripper_share]
            if _arg('gripper') != 'none':
                command += ['--gripper-simple']
                gripper_joints = ['gripper_finger1_joint',
                                  'gripper_finger2_joint']
            result = subprocess.run(command, capture_output=True, text=True,
                                    check=True)
            urdf = result.stdout
        robot_description = _gz_robot_description(
            urdf, pkg_share, sensors=enabled, arm=arm_on,
            lidar_xyz=lidar_xyz, gripper_joints=gripper_joints)
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-world', gz_world_name,
                           '-topic', 'robot_description', '-name', 'go2',
                           '-z', spawn_z],
                output='screen',
            ),
        ]

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
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rm_group_controller',
                   '--controller-manager-timeout', '60'],
        condition=IfCondition(arm),
        output='screen',
    )
    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager-timeout', '60'],
        condition=IfCondition(PythonExpression([
            "'", arm, "'.lower() in ('1','true','yes','on') and '",
            LaunchConfiguration('gripper'), "' != 'none'"])),
        output='screen',
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{CAMERA_TOPIC}@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo'
            '[ignition.msgs.CameraInfo',
            f'{LIDAR_POINTS_TOPIC}@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked',
        ],
        condition=IfCondition(sensors),
        output='screen',
    )

    # gz message frame_ids are model-prefixed (go2/base/...), add the matching
    # static TF so RViz can transform them into the robot tree.
    sensor_tf = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.32715', '--y', '-0.00003', '--z', '0.04297',
                       '--frame-id', 'base',
                       '--child-frame-id', 'go2/base/front_camera'],
            condition=IfCondition(sensors),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', LaunchConfiguration('lidar_x'),
                       '--y', LaunchConfiguration('lidar_y'),
                       '--z', LaunchConfiguration('lidar_z'),
                       '--frame-id', 'base',
                       '--child-frame-id', f'go2/base/{LIDAR_LINK}'],
            condition=IfCondition(sensors),
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Start the Gazebo GUI client'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'go2_sim.sdf'),
            description='Gazebo world to load (ships with the Sensors system)'),
        DeclareLaunchArgument('gz_world_name', default_value='go2_sim',
                              description='World name inside the world SDF'),
        DeclareLaunchArgument('spawn_z', default_value='0.45',
                              description='Spawn height of the Go2'),
        DeclareLaunchArgument('walk', default_value='false',
                              description='Start the open-loop trot gait node'),
        DeclareLaunchArgument('auto_forward', default_value='true',
                              description='Trot forward before the first '
                                          '/cmd_vel (walk mode only)'),
        DeclareLaunchArgument('sensors', default_value='false',
                              description='Add the built-in front camera and '
                                          'LiDAR to the simulated Go2'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz2 with the sensor view '
                                          '(needs sensors:=true)'),
        DeclareLaunchArgument('lidar_x', default_value='0.28945',
                              description='LiDAR mount x in base frame'),
        DeclareLaunchArgument('lidar_y', default_value='0',
                              description='LiDAR mount y in base frame'),
        DeclareLaunchArgument('lidar_z', default_value='0.10',
                              description='LiDAR mount z in base frame'),
        DeclareLaunchArgument('arm', default_value='false',
                              description='Mount the RM75 arm on the back'),
        DeclareLaunchArgument(
            'arm_root',
            default_value=os.environ.get(
                'RM_ROBOT_ROOT',
                os.path.join(os.path.abspath(os.path.join(
                    pkg_share, '..', '..', '..', '..', '..')),
                    'third_party', 'ros2_rm_robot')),
            description='ros2_rm_robot repo path'),
        DeclareLaunchArgument('arm_variant', default_value=ARM_DEFAULT_VARIANT,
                              description='rm_75 | rm_75_6f | rm_75_6fb'),
        DeclareLaunchArgument('plate_xyz', default_value='0 0 0.065',
                              description='Payload plate position in base'),
        DeclareLaunchArgument('arm_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('arm_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tool_xyz', default_value='0 0 0',
                              description='gripper flange frame tool0'),
        DeclareLaunchArgument('tool_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('camera', default_value=DEFAULT_CAMERA,
                              description='none | d435i | d435 | d415 ...'),
        DeclareLaunchArgument('camera_xyz', default_value='0 0.06 -0.02',
                              description='camera bracket in Link7 frame'),
        DeclareLaunchArgument('camera_rpy', default_value='0 -1.5708 0'),
        DeclareLaunchArgument('realsense_share',
                              default_value=_realsense_share_default()),
        DeclareLaunchArgument('gripper', default_value=DEFAULT_GRIPPER,
                              description='none | ag95 | ag145'),
        DeclareLaunchArgument('gripper_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('gripper_rpy', default_value='0 -1.5708 0'),
        DeclareLaunchArgument('gripper_share',
                              default_value=_gripper_share_default()),
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
        OpaqueFunction(function=_robot_nodes),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                       '/model/go2/odometry@nav_msgs/msg/Odometry'
                       '[ignition.msgs.Odometry',
                       '/model/red_ball/odometry@nav_msgs/msg/Odometry'
                       '[ignition.msgs.Odometry',
                       '/model/grasp_ball/odometry@nav_msgs/msg/Odometry'
                       '[ignition.msgs.Odometry',
                       ['/world/', gz_world_name,
                        '/set_pose@ros_gz_interfaces/srv/SetEntityPose']],
            output='screen',
        ),
        sensor_bridge,
        *sensor_tf,
        spawner_jsb,
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_jsb,
                          on_exit=[spawner_jgpc]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_jgpc,
                          on_exit=[spawner_arm]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_arm,
                          on_exit=[spawner_gripper]),
        ),
        Node(
            package='go2_description',
            executable='go2_trot.py',
            parameters=[{'auto_forward': ParameterValue(
                LaunchConfiguration('auto_forward'), value_type=bool)}],
            condition=IfCondition(walk),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'sensors.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
