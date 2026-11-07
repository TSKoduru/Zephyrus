"""Launch the flight interface plus the example competitor node.

This is what ``run_full_stack.sh`` starts in its ``competitor`` tmux window.
Fork it: keep the ``flight_interface`` node exactly as it is (it owns the
offboard heartbeat and the safety envelope) and swap ``square_mission`` for
your own.

    ros2 launch competitor_example competitor.launch.py
    ros2 launch competitor_example competitor.launch.py \
        max_altitude_m:=2.0 square_side_m:=3.0

Set ``competitor:=false`` to bring up only the flight interface, e.g. to fly
setpoints by hand from ``ros2 topic pub``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    ('hover_altitude_m', '1.5', 'example: hover height, metres'),
    ('hover_seconds', '5.0', 'example: seconds to hold the hover'),
    ('square_side_m', '5.0', 'example: side of the square, metres'),
    ('arrival_radius_m', '0.3',
     'example: how close counts as reaching a corner, metres'),
    ('competitor', 'true', 'also start the example node'),
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

    competitor = Node(
        package='competitor_example',
        executable='square_mission',
        name='competitor_example',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'hover_altitude_m': LaunchConfiguration('hover_altitude_m'),
            'hover_seconds': LaunchConfiguration('hover_seconds'),
            'square_side_m': LaunchConfiguration('square_side_m'),
            'arrival_radius_m': LaunchConfiguration('arrival_radius_m'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, competitor])
