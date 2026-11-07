"""Start the cuVSLAM -> PX4 VehicleOdometry bridge.

Run in a second container shell alongside `run_slam.sh`. The
Micro-XRCE-DDS-Agent (see `run_agent.sh`) shuttles the published
/fmu/in/vehicle_visual_odometry topic to the Pixhawk over serial.

`pose_frame` defaults to FRD because cuVSLAM's odom frame is initialised at
the rig's startup pose and has no heading reference -- see the module
docstring in px4_odom_bridge.py. Override with:

    ros2 launch oakd_vslam_bringup px4_bridge.launch.py pose_frame:=NED
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# (name, default, type, description)
ARGS = [
    ('input_topic', '/visual_slam/tracking/odometry', str,
     'cuVSLAM nav_msgs/Odometry input (ENU/FLU).'),
    ('output_topic', '/fmu/in/vehicle_visual_odometry', str,
     'PX4 VehicleOdometry output (NED/FRD).'),
    ('status_topic', '/visual_slam/status', str,
     'VisualSlamStatus topic driving reset_counter. Empty to disable.'),
    ('pose_frame', 'FRD', str,
     'FRD (default, arbitrary heading reference) or NED.'),
    ('publish_velocity', 'true', bool,
     'Publish body-FRD velocity alongside the pose.'),
    ('min_position_variance', '0.0', float,
     'Lower bound (m^2) on the published position variance; 0.0 = off. '
     'Stopgap for cuVSLAM identity-fallback covariance.'),
]


def generate_launch_description():
    bridge = Node(
        package='oakd_vslam_bringup',
        executable='px4_odom_bridge',
        name='px4_odom_bridge',
        output='screen',
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=vtype)
            for name, _, vtype, _ in ARGS
        }],
    )
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, _, desc in ARGS
    ]
    return LaunchDescription(declared + [bridge])
