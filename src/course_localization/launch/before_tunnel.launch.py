"""Separate opt-in course prefix: land before tunnel, <=1 ft/s commands."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('course_localization'))
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(share/'config/hardware.yaml')),
        LogInfo(msg='BEFORE TUNNEL: bounded velocity feedback, max 0.3048 m/s (1 ft/s), then touchdown. NOT MPC.'),
        Node(package='skybeta_flight_interface', executable='flight_interface', output='screen',
             parameters=[{'max_speed_mps': .3048, 'auto_offboard_on_arm': True}]),
        Node(package='course_localization', executable='localize', name='course_localization',
             output='screen', parameters=[config, {'config_file': config, 'compact_logs': True,
                                                  'record_raw_topics': False}]),
        Node(package='course_localization', executable='follow_course', name='course_controller',
             output='screen', parameters=[{
                 'enable_control': ParameterValue(LaunchConfiguration('enable_control'), value_type=bool),
                 'trajectory_path': str(share/'trajectory/before_tunnel.npz'),
                 'max_speed_mps': .3048, 'max_position_error_m': .10, 'land_at_end': True,
             }]),
    ])
