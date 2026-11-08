"""Localization only: no flight interface, mission, controller or arming."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('course_localization'))
    config=LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/hardware.yaml')),
        LogInfo(msg='WALK TEST ONLY: localization and compact logs; NO FLIGHT COMMANDS. ASCII view appears below at 2 Hz.'),
        Node(package='course_localization',executable='localize',name='course_localization',
             output='screen',parameters=[config,{'config_file':config,'compact_logs':True,
                                               'record_raw_topics':False,'ascii_live':True}]),
    ])
