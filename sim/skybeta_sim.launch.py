"""skybeta stack against Isaac Sim / Pegasus instead of the OAK-D.

Runs inside the ``skybeta-sim`` container (docker/Dockerfile.sim). It is
``oakd_vslam.launch.py`` with the depthai driver removed and Pegasus's camera
topics in its place, plus the two pieces ``host/run.sh stack`` normally starts in
their own tmux windows: ``MicroXRCEAgent udp4`` and the PX4 odometry bridge.

    ros2 launch /root/skybeta_ws/sim/skybeta_sim.launch.py

Data path::

    Isaac Sim (host)
      /drone1/left|right/color/image_raw    (rgb8, 1280x800 @ 30)
      /drone1/left|right/color/camera_info
      /drone1/sensors/imu
              |
              |  ImageFormatConverterNode  (rgb8 -> mono8, GPU, in-container)
              v
      /oak/left|right/image_mono  ------> cuVSLAM  ---> /visual_slam/tracking/odometry
                                                             |
                                                  px4_odom_bridge (ENU->NED, FRD)
                                                             v
                                            /fmu/in/vehicle_visual_odometry
                                                             |
                                        MicroXRCEAgent udp4 -p 8888  <-> PX4 SITL

Differences from the flight launch, and why:

* ``rectified_images: True``. Isaac renders an ideal pinhole projection (the
  Pegasus script writes zero OpenCV distortion coefficients), so the images that
  arrive really are rectified. On the drone the depthai driver publishes raw
  frames and the flight launch sets this False.
* No depthai driver, no ``oakd_vslam.yaml``.
* The camera optical frames are named after the Pegasus cameras (``left`` /
  ``right``), because ``ROS2Backend`` stamps every image and CameraInfo with
  ``frame_id = <camera name>``. On the drone they are depthai's
  ``oak_{left,right}_camera_optical_frame``.
* ``/oak/*`` mirrors of the camera_info and IMU topics exist so competitor code
  written against the hardware topic names runs unchanged. The colour images are
  NOT mirrored by default -- 1280x800 rgb8 at 30 Hz on two eyes is ~184 MB/s
  through a generic relay. Set ``mirror_raw_images:=true`` if you need them.

The ``camera_*`` and ``baseline`` arguments MUST match the ``SKYBETA_CAMERA_*``
and ``SKYBETA_BASELINE`` environment variables sim/pegasus_skybeta.py was started
with. They carry the same defaults; the static TFs published here are what
cuVSLAM uses as the stereo rig geometry (``visual_slam_impl.cpp`` reads the
extrinsics from TF, never from the CameraInfo ``P`` matrix), so a mismatch is a
silently wrong baseline, not an error.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare

# Must match LEFT_CAMERA_NAME / RIGHT_CAMERA_NAME in sim/pegasus_skybeta.py.
# These are not launch arguments on purpose: they are also the TF frame names and
# the values of cuVSLAM's camera_optical_frames parameter, and a list-of-strings
# parameter built out of substitutions is a good deal more fragile than a
# constant that two files are documented to keep in step.
LEFT_CAMERA_NAME = 'left'
RIGHT_CAMERA_NAME = 'right'

# Must match SKYBETA_IMAGE_WIDTH / _HEIGHT. ImageFormatConverterNode sizes its GPU
# block pool from these, so a mismatch is a runtime failure rather than a resize.
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 800

# FLU parent -> ROS optical child (optical +X right, +Y down, +Z forward).
# tf2_ros static_transform_publisher applies Rz(yaw) * Ry(pitch) * Rx(roll).
OPTICAL_ROLL = '-1.5707963267948966'
OPTICAL_PITCH = '0.0'
OPTICAL_YAW = '-1.5707963267948966'

CAMERA_RIG_FRAME = 'oak_parent_frame'


def generate_launch_description():
    # ---- which Pegasus vehicle to consume -----------------------------------
    # ROS2Backend publishes under "<namespace><vehicle_id>/". Note that its own
    # default namespace is already "drone<vehicle_id>", so pegasus_skybeta.py
    # passes namespace="drone" explicitly to get "/drone1/..." rather than
    # "/drone11/...". Keep these two in step with that script.
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='drone',
        description="ROS2Backend namespace, WITHOUT the vehicle id suffix.")
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id', default_value='1',
        description='ROS2Backend vehicle_id. Topics are /<namespace><vehicle_id>/...')

    # ---- camera rig pose, metres and radians, body FLU ----------------------
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.10')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')
    baseline_arg = DeclareLaunchArgument(
        'baseline', default_value='0.075',
        description='Stereo baseline in metres. OAK-D-S2 is 0.075.')

    # ---- cuVSLAM tuning, same knobs as the flight launch --------------------
    jitter_arg = DeclareLaunchArgument(
        'image_jitter_threshold_ms', default_value='40.0',
        description='cuVSLAM 3.2 warns above this and hands the same value to the '
                    'tracker as max_frame_delta_s. A rendered sim jitters more '
                    'than a camera does, not less: every frame Isaac cannot '
                    'render in 16.7 ms widens the gap.')
    localization_arg = DeclareLaunchArgument(
        'enable_localization_n_mapping', default_value='False',
        description='Odometry only, as in flight. cuVSLAM defaults this True.')
    slam_vis_arg = DeclareLaunchArgument('enable_slam_visualization', default_value='True')
    landmarks_arg = DeclareLaunchArgument('enable_landmarks_view', default_value='True')
    observations_arg = DeclareLaunchArgument('enable_observations_view', default_value='True')
    path_max_size_arg = DeclareLaunchArgument('path_max_size', default_value='1024')

    # ---- optional pieces ----------------------------------------------------
    # In SITL the uXRCE-DDS client is a UDP client on 8888, started unconditionally
    # by ROMFS/px4fmu_common/init.d-posix/rcS (`uxrce_dds_client start -t udp -p
    # $uxrce_dds_port`, default 8888, override with PX4_UXRCE_DDS_PORT). There is no
    # serial port and therefore no UXRCE_DDS_CFG / MAV_1_CONFIG contention to resolve.
    # This is container_scripts/run_agent.sh's XRCE_TRANSPORT=udp4 branch.
    run_agent_arg = DeclareLaunchArgument('run_agent', default_value='true')
    xrce_udp_port_arg = DeclareLaunchArgument('xrce_udp_port', default_value='8888')

    run_bridge_arg = DeclareLaunchArgument('run_bridge', default_value='true')
    pose_frame_arg = DeclareLaunchArgument(
        'pose_frame', default_value='FRD',
        description="As on the drone: cuVSLAM's odom frame is initialised at the "
                    'rig startup pose and has no heading reference, which is the '
                    'definition of POSE_FRAME_FRD.')

    mirror_oak_arg = DeclareLaunchArgument(
        'mirror_oak_topics', default_value='true',
        description='Republish CameraInfo and IMU under the hardware /oak/* names.')
    mirror_images_arg = DeclareLaunchArgument(
        'mirror_raw_images', default_value='false',
        description='Also republish the colour images as /oak/{left,right}/image_raw. '
                    'Off by default: ~184 MB/s through a generic relay.')

    # ---- substitutions ------------------------------------------------------
    prefix = [LaunchConfiguration('namespace'), LaunchConfiguration('vehicle_id')]

    def pegasus_topic(*parts):
        return ['/'] + prefix + ['/'] + list(parts)

    def camera_topic(camera, leaf):
        return pegasus_topic(camera, '/color/', leaf)

    half_baseline = PythonExpression(['0.5 * ', LaunchConfiguration('baseline')])
    neg_half_baseline = PythonExpression(['-0.5 * ', LaunchConfiguration('baseline')])

    # ---- TF: base_link -> rig -> optical frames -----------------------------
    # The chain mirrors _camera_config() in pegasus_skybeta.py, which rotates the
    # lateral offset by the same rig rotation before handing it to Isaac.
    base_to_rig_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_camera_rig',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', '0.0',
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', CAMERA_RIG_FRAME,
        ],
    )

    def optical_tf(camera_name, y_substitution):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'camera_rig_to_{camera_name}',
            arguments=[
                '--x', '0.0', '--y', y_substitution, '--z', '0.0',
                '--roll', OPTICAL_ROLL,
                '--pitch', OPTICAL_PITCH,
                '--yaw', OPTICAL_YAW,
                '--frame-id', CAMERA_RIG_FRAME,
                '--child-frame-id', camera_name,
            ],
        )

    # ---- rgb8 -> mono8 on the GPU, in cuVSLAM's own container ---------------
    def format_converter(side, camera_name):
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
                # Straight off Isaac's publisher: one hop fewer than going via
                # the /oak/* mirror, and it keeps the SLAM path working whether
                # or not mirror_raw_images is on.
                ('image_raw', camera_topic(camera_name, 'image_raw')),
                ('image', f'/oak/{side}/image_mono'),
            ],
        )

    vslam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False,
            # True here, False on the drone -- Isaac renders an undistorted
            # pinhole projection. cuVSLAM then applies CameraInfo.r inverse to the
            # extrinsics; Isaac publishes r = identity, so that is a no-op.
            'rectified_images': True,
            # Pegasus's IMU is PX4-convention NED/FRD, not a camera frame. Fusing
            # it would be wrong in a way that looks like a tuning problem.
            'enable_imu_fusion': False,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            # ROS2Backend stamps images and CameraInfo with frame_id = camera name.
            # Declared explicitly rather than inferred, as in the flight launch.
            'camera_optical_frames': [LEFT_CAMERA_NAME, RIGHT_CAMERA_NAME],
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
            ('visual_slam/camera_info_0', camera_topic(LEFT_CAMERA_NAME, 'camera_info')),
            ('visual_slam/image_1', '/oak/right/image_mono'),
            ('visual_slam/camera_info_1', camera_topic(RIGHT_CAMERA_NAME, 'camera_info')),
            ('visual_slam/imu', pegasus_topic('sensors/imu')),
        ],
    )

    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            format_converter('left', LEFT_CAMERA_NAME),
            format_converter('right', RIGHT_CAMERA_NAME),
            vslam,
        ],
        output='screen',
    )

    # ---- /oak/* mirrors for hardware topic-name parity ----------------------
    def relay(name, input_topic, output_topic, condition_arg):
        return Node(
            package='topic_tools', executable='relay', name=name,
            parameters=[{
                'input_topic': ParameterValue(input_topic, value_type=str),
                'output_topic': output_topic,
                'lazy': False,
            }],
            condition=IfCondition(LaunchConfiguration(condition_arg)),
            output='screen',
        )

    mirrors = [
        relay('relay_left_camera_info',
              camera_topic(LEFT_CAMERA_NAME, 'camera_info'),
              '/oak/left/camera_info', 'mirror_oak_topics'),
        relay('relay_right_camera_info',
              camera_topic(RIGHT_CAMERA_NAME, 'camera_info'),
              '/oak/right/camera_info', 'mirror_oak_topics'),
        # NOTE: this is PX4-convention NED/FRD data on a topic whose hardware
        # namesake carries a BNO086 stream in the OAK-D's frame. Same name, same
        # message type, different frame. Nothing in this launch consumes it.
        relay('relay_imu', pegasus_topic('sensors/imu'),
              '/oak/imu/data', 'mirror_oak_topics'),
        relay('relay_left_image', camera_topic(LEFT_CAMERA_NAME, 'image_raw'),
              '/oak/left/image_raw', 'mirror_raw_images'),
        relay('relay_right_image', camera_topic(RIGHT_CAMERA_NAME, 'image_raw'),
              '/oak/right/image_raw', 'mirror_raw_images'),
    ]

    # ---- the uXRCE-DDS agent ------------------------------------------------
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_udp_port')],
        name='micro_xrce_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    # ---- the PX4 odometry bridge -------------------------------------------
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('oakd_vslam_bringup'), 'launch', 'px4_bridge.launch.py',
        ])),
        launch_arguments={'pose_frame': LaunchConfiguration('pose_frame')}.items(),
        condition=IfCondition(LaunchConfiguration('run_bridge')),
    )

    return LaunchDescription([
        namespace_arg, vehicle_id_arg,
        camera_x_arg, camera_z_arg,
        camera_roll_arg, camera_pitch_arg, camera_yaw_arg, baseline_arg,
        jitter_arg, localization_arg,
        slam_vis_arg, landmarks_arg, observations_arg, path_max_size_arg,
        run_agent_arg, xrce_udp_port_arg, run_bridge_arg, pose_frame_arg,
        mirror_oak_arg, mirror_images_arg,
        base_to_rig_tf,
        optical_tf(LEFT_CAMERA_NAME, half_baseline),
        optical_tf(RIGHT_CAMERA_NAME, neg_half_baseline),
        container,
        *mirrors,
        agent,
        bridge,
    ])
