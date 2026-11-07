"""Read-only course localization and extensive logging beside the existing flight stack."""
import json
import os
import sys
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, String

from .filter_session import FilterSession
from .inputs import Inputs
from .raw_bag import RawBag
from .recording import RunRecorder


DEFAULTS = {
    'left_image_topic': '/oak/left/image_mono',
    'right_image_topic': '/oak/right/image_mono',
    'left_info_topic': '/oak/left/camera_info',
    'right_info_topic': '/oak/right/camera_info',
    'odometry_topic': '/visual_slam/tracking/odometry',
    'imu_topic': '/oak/imu/data',
    'range_topic': '/course/downward_range',
    'px4_range_topic': '/fmu/out/distance_sensor',
    'body_frame': 'base_link', 'map_frame': 'course_map',
    'initial_mean4': [-2.4146, 0.8144, 0.116, 0.872664626],
    'initial_std4': [0.25, 0.25, 0.08, 0.2617993878],
    'particle_seed': 42, 'update_hz': 5.0, 'stereo_slop_s': 0.003,
    'range_position_body_m': [0.0, 0.0, 0.0],
    'range_fov_deg': 1.0, 'range_variance_m2': 0.0005,
    'range_max_age_s': 0.10, 'px4_time_offset_s': 0.0,
    'log_root': '~/team_ws/logs/course', 'record_raw_topics': True,
    'config_file': '',
}


def cloud_message(points, header):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    message = PointCloud2(header=header, height=1, width=len(points), is_bigendian=False,
                          point_step=12, row_step=12 * len(points), is_dense=True)
    message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
    message.data = points.tobytes()
    return message


def depth_points(frame):
    y, x = np.nonzero(np.isfinite(frame.depth_m) & (frame.depth_m > 0))
    camera = (np.c_[x, y, np.ones(len(x))] @ np.linalg.inv(frame.K).T) * frame.depth_m[y, x, None]
    return camera @ frame.T_body_camera[:3, :3].T + frame.T_body_camera[:3, 3]


class CourseLocalization(Node):
    def __init__(self):
        super().__init__('course_localization')
        self.config = {name: self.declare_parameter(name, value).value for name, value in DEFAULTS.items()}
        if not 0 < self.config['update_hz'] <= 30 or self.config['range_max_age_s'] <= 0:
            raise ValueError('Positive range age and update_hz in (0,30] required')
        share = Path(get_package_share_directory('course_localization'))
        map_directory = share / 'map'
        files = {f'map/{p.name}': p for p in map_directory.glob('*.obj')}
        files.update({f'reference/{p.name}': p for p in (share / 'trajectory').glob('*.npz')})
        files.update({f'config/{p.name}': p for p in (share / 'config').glob('*') if p.is_file()})
        files.update({f'source/{p.relative_to(Path(__file__).parent)}': p
                      for p in Path(__file__).parent.rglob('*.py')})
        if self.config['config_file']:
            files['parameters.yaml'] = Path(self.config['config_file'])
        self.recorder = RunRecorder(
            Path(self.config['log_root']).expanduser(), metadata={'parameters': self.config,
                'purpose': 'Measured hardware localization, not simulated ground truth',
                'python_version': sys.version, 'numpy_version': np.__version__,
                'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
                'range_mount_note': 'Configured body offset must match the physical mount',
                'vo_map_alignment': 'Initial configured pose only; never corrected by PF',
                'core_tune': {'particles': 800, 'motion_noise_scale': 4.0, 'sigma_hit': 0.18,
                              'keyframe_translation': 0.3, 'effective_rays': 6.0}},
            config_paths=files, map_identity={'source': 'photo-reconstructed nominal structural map'},
        )
        self.bag = None
        try:
            self.session = FilterSession(self.config, map_directory, Path.home() / '.cache/course_localization')
            self.inputs = Inputs(self, self.session, self.config)
            topics = [value for key, value in self.config.items() if key.endswith('_topic') and value]
            topics += ['/tf', '/tf_static', '/visual_slam/status', '/course/particle_pose',
                       '/course/particles', '/course/status', '/course/stereo_depth',
                       '/course/observed_points_vo', '/course/observed_points_pf',
                       '/course/nominal_map', '/course/controller_status', '/skybeta/state',
                       '/fmu/out/vehicle_attitude',
                       '/skybeta/setpoint/pose', '/skybeta/setpoint/twist']
            self.bag = RawBag(self.recorder.path, topics, self.config['record_raw_topics'])
        except Exception:
            self.recorder.close()
            raise
        self.nominal_pub = self.create_publisher(
            PointCloud2, '/course/nominal_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nominal_pub.publish(cloud_message(
            self.session.pf.map.points, Header(frame_id=self.config['map_frame'])))
        self.pose_pub = self.create_publisher(Odometry, '/course/particle_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/course/particles', 2)
        self.depth_pub = self.create_publisher(Image, '/course/stereo_depth', 2)
        self.vo_cloud = self.create_publisher(PointCloud2, '/course/observed_points_vo', 2)
        self.pf_cloud = self.create_publisher(PointCloud2, '/course/observed_points_pf', 2)
        self.status_pub = self.create_publisher(String, '/course/status', 10)
        self.bridge = CvBridge()
        self.last_measurement = {}
        self.frame_count = 0
        self.last_frame_time = None
        self.last_error = 'waiting_for_calibrated_stereo_and_odometry'
        self.create_timer(0.02, self.process)
        self.create_timer(1.0, self.status)
        self.get_logger().info(f'Logging to {self.recorder.path}; localization only, no flight commands')

    def process(self):
        started = time.monotonic()
        try:
            observation = self.inputs.next_observation()
            if observation is None:
                return
            frame, position, quaternion, measurement, covered, raw, wait = observation
            state, arrays, diagnostics, transforms = self.session.step(
                frame, position, quaternion, measurement, covered
            )
            points = depth_points(frame)
            arrays.update(raw)
            arrays['processing_ros_time'] = self.get_clock().now().nanoseconds * 1e-9
            diagnostics.update(stereo=self.inputs.estimator.last_diagnostics,
                               synchronization_wait_s=wait, dropped_pairs=self.inputs.dropped_pairs,
                               processing_seconds=time.monotonic() - started)
            accepted = self.recorder.record_frame(arrays, diagnostics, points_body=points, **transforms)
            if not accepted:
                self.get_logger().error(f'Frame logging rejected: {self.recorder.health}', throttle_duration_sec=2)
            self.publish(frame, state, transforms, points)
            self.last_measurement = dict(diagnostics, range_valid=measurement.valid)
            self.frame_count += 1
            self.last_frame_time = time.monotonic()
            self.last_error = ''
            self.status()
        except Exception as exc:
            self.last_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().warning(self.last_error, throttle_duration_sec=2)
            self.recorder.record_frame({}, {'event': 'processing_error', 'error': self.last_error,
                                           'ros_time': self.get_clock().now().nanoseconds * 1e-9})

    def publish(self, frame, state, transforms, points):
        stamp = rclpy.time.Time(nanoseconds=int(round(frame.timestamp * 1e9))).to_msg()
        header = Header(stamp=stamp, frame_id=self.config['map_frame'])
        output = Odometry(header=header, child_frame_id=self.config['body_frame'])
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = map(float, state['pose'][:3])
        q = Rotation.from_matrix(transforms['T_map_body_pf'][:3, :3]).as_quat()
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = map(float, q)
        covariance = np.zeros((6, 6))
        covariance[np.ix_([0, 1, 2, 5], [0, 1, 2, 5])] = state['covariance']
        covariance[3, 3] = covariance[4, 4] = np.deg2rad(5)**2
        output.pose.covariance = covariance.ravel().tolist()
        self.pose_pub.publish(output)
        poses = PoseArray(header=header)
        for x, y, z, yaw in self.session.pf.particles:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
            pose.orientation.z, pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            poses.poses.append(pose)
        self.particles_pub.publish(poses)
        depth = self.bridge.cv2_to_imgmsg(frame.depth_m, encoding='32FC1')
        depth.header = Header(stamp=stamp, frame_id='course_right_rectified_optical')
        self.depth_pub.publish(depth)
        for publisher, key in ((self.vo_cloud, 'T_map_body_vo'), (self.pf_cloud, 'T_map_body_pf')):
            transform = transforms[key]
            world = points @ transform[:3, :3].T + transform[:3, 3]
            publisher.publish(cloud_message(world, header))

    def status(self):
        health = self.recorder.health
        bag = self.bag.health()
        message = {'frames': self.frame_count, 'last_error': self.last_error,
                   'seconds_since_frame': None if self.last_frame_time is None else time.monotonic() - self.last_frame_time,
                   'log_directory': str(self.recorder.path), 'recorder': health, 'raw_bag': bag,
                   'last_measurement': self.last_measurement,
                   'pending_stereo': len(self.inputs.pending), 'dropped_stereo_pairs': self.inputs.dropped_pairs}
        self.status_pub.publish(String(data=json.dumps(message)))
        if health['failed'] or (bag['enabled'] and not bag['running']):
            self.get_logger().error(f'Logging failure: {message}', throttle_duration_sec=5)

    def close(self):
        if self.bag is not None:
            self.bag.close()
        self.recorder.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CourseLocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
