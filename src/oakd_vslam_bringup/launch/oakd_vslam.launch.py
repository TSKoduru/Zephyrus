"""OAK-D-S2 stereo + IMU -> isaac_ros_visual_slam (cuVSLAM 3.2).

The OAK-D-S2 stereo pair is **OV9282 monochrome** (1280x800 native), not the colour
OV9782 an earlier version of this docstring claimed. The depthai v3 driver nevertheless
publishes those sockets as ``bgr8`` -- it reports supportedTypes=[MONO, COLOR] for them
and exposes no output-encoding parameter -- while cuVSLAM requires ``mono8``. The
conversion therefore has to happen host-side.

It is done here by two ``isaac_ros_image_proc`` ``ImageFormatConverterNode`` instances
loaded into the *same* composable container as cuVSLAM, so the bgr8 -> mono8 step runs on
the GPU (GXF tensorops) instead of in a Python callback. Measured on this rig at
1024x640, the added image_raw -> image_mono latency drops from p50 10.1 / p95 16.4 ms
(the Python ``bgr_to_mono`` nodes this replaces) to p50 5.2 / p95 17.3 ms -- and the GPU
path measured that while also carrying cuVSLAM and the second eye, which the Python
baseline did not. It also loses no frames (0 of 1743, against 2 of 1714 for Python).
That latency lands directly in the ``EKF2_EV_DELAY`` budget.

Note on correctness: the sensor is monochrome, so the driver's bgr8 frames carry three
identical channels. The converted mono8 is bit-exact with each of them (verified on
recorded frames), so the fact that NITROS labels the input format ``nitros_image_rgb8``
while the driver stamps ``bgr8`` cannot introduce a channel swap or a weighting error.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

# Must match left/right i_width and i_height in config/oakd_vslam.yaml. The converter
# allocates its GPU blocks from these, so a mismatch is a runtime failure, not a resize.
IMAGE_WIDTH = 1024
IMAGE_HEIGHT = 640


def generate_launch_description():
    depthai_share = get_package_share_directory('depthai_ros_driver_v3')
    bringup_share = get_package_share_directory('oakd_vslam_bringup')

    # ---- camera mount pose in the drone body frame (FLU) -------------------------
    # Translation in metres, rotation in radians. This static TF is the ONLY place the
    # camera lever arm and mount rotation are expressed.
    #
    # EKF2_EV_POS_X/Y/Z *do* exist on the flight controller (v1.14 through v1.16 alike;
    # they are in this airframe's own param dump). Leave them at 0. If they are set on
    # the FC while this TF is also non-zero the lever arm is applied twice. Pick one --
    # this one, because it is what host/run.sh and the launch arguments below drive.
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.0')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')
    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll', default_value='0.0',
        description='Camera mount roll in the body frame, radians.')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='0.0',
        description='Camera mount pitch, radians. A down-tilted VIO camera is the norm '
                    'on a drone; an unmodelled tilt couples vehicle rotation into '
                    'apparent translation.')
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw', default_value='0.0',
        description='Camera mount yaw in the body frame, radians.')

    # ---- cuVSLAM tuning ----------------------------------------------------------
    jitter_arg = DeclareLaunchArgument(
        'image_jitter_threshold_ms', default_value='40.0',
        description='cuVSLAM 3.2 only WARNS when the header-stamp gap between frames '
                    'exceeds this -- Track() still runs, nothing is discarded. The '
                    'driver stamps 30 fps at 32.2-33.6 ms, which tips over the 34.0 ms '
                    'default about once a minute. 40.0 silences that; it does not and '
                    'cannot recover frames.')
    localization_arg = DeclareLaunchArgument(
        'enable_localization_n_mapping', default_value='False',
        description='cuVSLAM defaults this to True, i.e. full SLAM with loop closure. '
                    'For flight we want odometry only: loop closure re-localises by '
                    'teleporting the pose, and EKF2 integrates that jump as real '
                    'motion. Set True only for offline mapping runs.')
    # cuVSLAM 3.2 always *advertises* the 13 /visual_slam/vis/* topics; these flags only
    # gate what it publishes on them. With all three False, 11 of the 13 carry zero
    # messages (landmarks, observations, loop closures, pose graph, localizer, gravity).
    # /visual_slam/vis/slam_odometry and /visual_slam/vis/velocity keep publishing at
    # 30 Hz regardless -- they are not gated in 3.2. Measured, not assumed.
    slam_vis_arg = DeclareLaunchArgument('enable_slam_visualization', default_value='False')
    landmarks_arg = DeclareLaunchArgument('enable_landmarks_view', default_value='False')
    observations_arg = DeclareLaunchArgument('enable_observations_view', default_value='False')
    path_max_size_arg = DeclareLaunchArgument('path_max_size', default_value='1024')

    # depthai's URDF is rooted at 'oak_parent_frame', not 'oak'. Attach it to base_link
    # so cuVSLAM (base_frame=base_link) can reach the camera optical frames through a
    # single connected TF tree.
    base_to_oak_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_oak',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'oak_parent_frame',
        ],
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_share, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'name': 'oak',
            'params_file': os.path.join(bringup_share, 'config', 'oakd_vslam.yaml'),
        }.items(),
    )

    def format_converter(side):
        """bgr8 -> mono8 on the GPU, in the same process as cuVSLAM.

        QoS is left at the node's 'DEFAULT' (RELIABLE, depth 10), which matches the
        depthai driver's image publisher and is compatible with cuVSLAM's BEST_EFFORT
        subscription. Set 'input_qos'/'output_qos' here only if that changes.

        NITROS type negotiation does NOT complete on either side, and that is expected,
        not a misconfiguration: the depthai driver is a plain ROS publisher upstream, and
        downstream cuVSLAM 3.2 uses a ManagedNitrosSubscriber, which does not take part
        in the negotiation protocol. The log therefore says "Could not negotiate" twice
        and both ends fall back to their compatible endpoints, i.e. a normal
        sensor_msgs/Image on /oak/{side}/image_mono. Raising
        'type_negotiation_duration_s' does not change this -- tried at 5 s, same result.
        The conversion itself still runs on the GPU, which is the win; the hand-off to
        cuVSLAM is an ordinary in-container ROS message.
        """
        return ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name=f'{side}_format_converter',
            namespace='',
            parameters=[{
                'encoding_desired': 'mono8',
                'image_width': IMAGE_WIDTH,
                'image_height': IMAGE_HEIGHT,
            }],
            remappings=[
                ('image_raw', f'/oak/{side}/image_raw'),
                ('image', f'/oak/{side}/image_mono'),
            ],
        )

    vslam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': False,
            # Valid in 3.2 (4.x renames this to tracking_mode). Keep False: /oak/imu/data
            # is a raw BNO086 stream with no online calibration, and pure stereo VO is
            # what the PX4 bridge and its covariances assume.
            'enable_imu_fusion': False,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'imu_frame': 'oak_imu_frame',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            # Declared explicitly rather than left at the 3.2 default of {} so the
            # base_link -> optical-frame extrinsics resolve against known frame names
            # instead of whatever frame_id the driver happens to stamp.
            'camera_optical_frames': [
                'oak_left_camera_optical_frame',
                'oak_right_camera_optical_frame',
            ],
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'image_jitter_threshold_ms': ParameterValue(
                LaunchConfiguration('image_jitter_threshold_ms'), value_type=float),
            'enable_localization_n_mapping': ParameterValue(
                LaunchConfiguration('enable_localization_n_mapping'), value_type=bool),
            'enable_slam_visualization': ParameterValue(
                LaunchConfiguration('enable_slam_visualization'), value_type=bool),
            'enable_landmarks_view': ParameterValue(
                LaunchConfiguration('enable_landmarks_view'), value_type=bool),
            'enable_observations_view': ParameterValue(
                LaunchConfiguration('enable_observations_view'), value_type=bool),
            'path_max_size': ParameterValue(
                LaunchConfiguration('path_max_size'), value_type=int),
        }],
        remappings=[
            ('visual_slam/image_0', '/oak/left/image_mono'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_mono'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )

    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            format_converter('left'),
            format_converter('right'),
            vslam,
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_x_arg, camera_y_arg, camera_z_arg,
        camera_roll_arg, camera_pitch_arg, camera_yaw_arg,
        jitter_arg, localization_arg,
        slam_vis_arg, landmarks_arg, observations_arg, path_max_size_arg,
        base_to_oak_tf,
        camera, container,
    ])
