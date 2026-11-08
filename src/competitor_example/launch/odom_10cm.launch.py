"""EXPLICIT INITIAL FLIGHT: odometry-only UP35cm, FORWARD10cm, DOWN; stereo map recorded."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name -> (default, help). Envelope values are the flight interface's own
# defaults, repeated here so a competitor can tighten them from one command
# line without editing either package.
ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0',
     'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY: on a real vehicle Offboard '
     'follows the pilot arming, then switching to Position'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position. false: the vehicle '
     'stays in Position under the pilot'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    ('enable_motion', 'true', 'Initial 10cm test; false records only'),
    ('competitor', 'true', 'Start initial odometry test and stereo recorder'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    recorder = Node(package='course_localization', executable='stereo_map_only',
                    output='screen', condition=IfCondition(LaunchConfiguration('competitor')))
    mission = Node(package='course_localization', executable='odom_10cm_test',
                   output='screen', condition=IfCondition(LaunchConfiguration('competitor')),
                   parameters=[{'enable_motion': LaunchConfiguration('enable_motion')}])
    return LaunchDescription(declared + [
        LogInfo(msg='INITIAL TEST: UP 35cm -> FORWARD 10cm -> DOWN. ODOMETRY ONLY; NO PF/MPC.'),
        flight_interface, recorder, mission])
