"""Opt-in feedback controller; never starts a second flight interface or mission."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='false'),
        Node(package='course_localization', executable='follow_course',
             name='course_controller', output='screen', parameters=[{
                 'enable_control': ParameterValue(LaunchConfiguration('enable_control'), value_type=bool),
             }]),
    ])
