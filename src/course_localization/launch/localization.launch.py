"""Start localization/logging only, alongside the existing OAK/cuVSLAM stack."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(get_package_share_directory('course_localization')) / 'config/hardware.yaml'
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default_config)),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='course_localization', executable='localize', name='course_localization',
             output='screen', parameters=[config, {'config_file': config}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(default_config.parent / 'course.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
