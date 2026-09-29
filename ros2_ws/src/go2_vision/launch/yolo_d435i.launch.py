import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'align_depth.enable': LaunchConfiguration('align_depth'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    detector = Node(
        package='go2_vision',
        executable='yolo_detector.py',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'image_topic': ['/camera/', LaunchConfiguration('camera_name'),
                            '/color/image_raw'],
            'depth_topic': ['/camera/', LaunchConfiguration('camera_name'),
                            '/aligned_depth_to_color/image_raw'],
            'use_depth': ParameterValue(
                LaunchConfiguration('use_depth'), value_type=bool),
            'confidence': LaunchConfiguration('confidence'),
            'device': ParameterValue(
                LaunchConfiguration('device'), value_type=str),
        }],
        output='screen',
    )

    view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/vision/image_annotated'],
        condition=IfCondition(LaunchConfiguration('view')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.expanduser('~/go2_dev/models/yolo11n.pt')),
        DeclareLaunchArgument('camera_name', default_value='d435i'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('align_depth', default_value='true'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('view', default_value='true'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
        DeclareLaunchArgument('device', default_value='0'),
        camera,
        detector,
        view,
    ])
