"""Timestamped ROS observations and calibrated stereo, separate from filter logic."""
from collections import deque
import time

import cv2
import message_filters
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .core.range_sensor import RangeMeasurement
from .core.stereo_depth import StereoDepthEstimator
from .sensor_geometry import StereoGeometry, interpolate_pose, pose_matrix


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Inputs:
    def __init__(self, node, session, config):
        self.node, self.session, self.config = node, session, config
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.info = {}
        self.odom = deque(maxlen=300)
        self.ranges = deque(maxlen=200)
        self.imu_log = deque(maxlen=4000)
        self.pending = deque(maxlen=10)
        self.geometry = self.estimator = self.body_camera = None
        self.calibration_signature = None
        self.last_queued = -float('inf')
        self.dropped_pairs = 0
        self.subscriptions = []
        for eye in ('left', 'right'):
            self.subscriptions.append(node.create_subscription(
                CameraInfo, config[f'{eye}_info_topic'],
                lambda msg, eye=eye: self.info.__setitem__(eye, msg), qos_profile_sensor_data,
            ))
        for message_type, topic, callback in (
            (Odometry, config['odometry_topic'], self.on_odom),
            (Imu, config['imu_topic'], self.on_imu),
            (Range, config['range_topic'], self.on_range),
        ):
            self.subscriptions.append(node.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data
            ))
        if config['px4_range_topic']:
            from px4_msgs.msg import DistanceSensor
            self.subscriptions.append(node.create_subscription(
                DistanceSensor, config['px4_range_topic'], self.on_px4_range, qos_profile_sensor_data
            ))
        self.left = message_filters.Subscriber(
            node, Image, config['left_image_topic'], qos_profile=qos_profile_sensor_data
        )
        self.right = message_filters.Subscriber(
            node, Image, config['right_image_topic'], qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left, self.right], queue_size=10, slop=config['stereo_slop_s']
        )
        self.sync.registerCallback(self.on_pair)

    def transform(self, target, source):
        msg = self.tf.lookup_transform(target, source, Time()).transform
        return pose_matrix([msg.translation.x, msg.translation.y, msg.translation.z],
                           [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w])

    def on_odom(self, msg):
        if msg.child_frame_id != self.config['body_frame']:
            self.node.get_logger().error('Odometry child frame must be the configured body frame',
                                         throttle_duration_sec=5)
            return
        pose = msg.pose.pose
        row = (stamp_seconds(msg.header.stamp),
               np.array([pose.position.x, pose.position.y, pose.position.z]),
               np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        if not all(np.isfinite(v).all() for v in row) or np.linalg.norm(row[2]) < 0.5:
            return
        if self.odom and row[0] <= self.odom[-1][0]:
            return
        self.odom.append(row)

    def on_imu(self, msg):
        try:
            rotation = self.transform(self.config['body_frame'], msg.header.frame_id)[:3, :3]
            gyro = rotation @ [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            accel = rotation @ [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            stamp = stamp_seconds(msg.header.stamp)
            self.session.inertial.add_batch([stamp], [gyro], [accel])
            self.imu_log.append(np.r_[stamp, gyro, accel])
            # Keep a long bracket around the bounded pending exposure queue.
            aid = self.session.inertial
            if len(aid.times) > 6000:
                del aid.times[:-4000]
                del aid.quaternions[:-4000]
                aid.gaps = [(a, b) for a, b in aid.gaps if b >= aid.times[0]]
        except Exception as exc:
            self.node.get_logger().warning(f'IMU not usable: {exc}', throttle_duration_sec=5)

    def on_range(self, msg):
        # Standard Range uses its exposure timestamp; TF verifies the beam points down.
        try:
            mount = self.transform(self.config['body_frame'], msg.header.frame_id)
            if mount[2, 0] > -0.9995:  # REP-117 Range rays point along sensor +X.
                raise ValueError('Range frame +X is not downward')
            if np.linalg.norm(mount[:3, 3] - self.config['range_position_body_m']) > 0.02:
                raise ValueError('Range TF position differs from configured mount by >2cm')
            self.add_range(stamp_seconds(msg.header.stamp), msg.range, msg.min_range,
                           msg.max_range, 'sensor_msgs/Range')
        except Exception as exc:
            self.node.get_logger().warning(f'Range not usable: {exc}', throttle_duration_sec=5)

    def on_px4_range(self, msg):
        if msg.orientation != 25:  # MAV_SENSOR_ROTATION_PITCH_270: downward.
            return
        stamp = msg.timestamp * 1e-6 + self.config['px4_time_offset_s']
        quality = getattr(msg, 'signal_quality', -1)
        if quality == 0:
            return
        self.add_range(stamp, msg.current_distance, msg.min_distance,
                       msg.max_distance, 'px4/DistanceSensor')

    def add_range(self, stamp, value, minimum, maximum, source):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        # Refuse an unaligned boot clock rather than inventing an offset from arrival time.
        valid = (abs(now - stamp) <= 2.0 and np.isfinite(value) and minimum <= value <= maximum)
        status = source if valid else f'{source}:invalid_value_or_clock'
        self.ranges.append(RangeMeasurement(
            stamp, float(value), bool(valid), status, self.config['range_variance_m2'],
            np.deg2rad(self.config['range_fov_deg']), float(minimum), float(maximum),
        ))

    def on_pair(self, left, right):
        stamp = stamp_seconds(right.header.stamp)
        if stamp - self.last_queued < 1 / self.config['update_hz'] - 0.001:
            return
        if len(self.pending) == self.pending.maxlen:
            self.dropped_pairs += 1
        self.pending.append((left, right, time.monotonic()))
        self.last_queued = stamp

    def configure_stereo(self, left, right):
        if set(self.info) != {'left', 'right'}:
            raise ValueError('Waiting for both CameraInfo messages')
        li, ri = self.info['left'], self.info['right']
        if (li.header.frame_id, ri.header.frame_id) != (left.header.frame_id, right.header.frame_id):
            raise ValueError('Image and CameraInfo optical frames differ')
        if (li.width, li.height) != (left.width, left.height) or (ri.width, ri.height) != (right.width, right.height):
            raise ValueError('CameraInfo dimensions do not match images')
        if li.distortion_model not in ('plumb_bob', 'rational_polynomial') or ri.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('Unsupported camera distortion model')
        relative = self.transform(right.header.frame_id, left.header.frame_id)
        self.geometry = StereoGeometry(
            np.asarray(li.k).reshape(3, 3), li.d, np.asarray(ri.k).reshape(3, 3), ri.d,
            relative[:3, :3], relative[:3, 3], (left.width, left.height),
        )
        raw_mount = self.transform(self.config['body_frame'], right.header.frame_id)
        self.body_camera = self.geometry.body_from_rectified_right(raw_mount)
        self.calibration_signature = self.signature()
        self.estimator = StereoDepthEstimator(
            self.geometry.K_right, baseline_m=self.geometry.baseline_m,
            texture_noise_multiplier=1.25,
        )

    def signature(self):
        return tuple((m.width, m.height, m.header.frame_id, tuple(m.k), tuple(m.d))
                     for m in (self.info.get('left'), self.info.get('right')) if m is not None)

    def next_observation(self):
        if not self.pending or len(self.odom) < 2:
            return None
        left, right, arrival = self.pending[0]
        stamp = stamp_seconds(right.header.stamp)
        wait = time.monotonic() - arrival
        latest_imu = self.session.inertial.last_time
        covered = latest_imu is not None and latest_imu >= stamp
        if (self.odom[-1][0] < stamp or not covered) and wait < 0.2:
            return None
        self.pending.popleft()
        stamps, positions, quaternions = zip(*self.odom)
        position, quaternion = interpolate_pose(stamps, positions, quaternions, stamp, max_gap_s=0.1)
        if self.geometry is None or self.signature() != self.calibration_signature:
            self.configure_stereo(left, right)
        lraw = self.bridge.imgmsg_to_cv2(left, desired_encoding='mono8')
        rraw = self.bridge.imgmsg_to_cv2(right, desired_encoding='mono8')
        lrect, rrect = self.geometry.rectify(lraw, rraw)
        frame = self.estimator.compute(lrect, rrect, stamp, self.body_camera)
        candidates = [r for r in self.ranges if r.valid and abs(r.timestamp - stamp) <= self.config['range_max_age_s']]
        measurement = min(candidates, key=lambda r: abs(r.timestamp - stamp)) if candidates else RangeMeasurement(
            stamp, float('nan'), False, 'missing_or_stale_range', self.config['range_variance_m2']
        )
        raw = {'left_image': lraw, 'right_image': rraw, 'left_rectified': lrect,
               'right_rectified': rrect, 'left_timestamp': stamp_seconds(left.header.stamp),
               'right_timestamp': stamp, 'imu_samples': np.asarray(self.imu_log),
               'odom_samples': np.asarray([np.r_[t, p, q] for t, p, q in self.odom]),
               'range_samples': np.asarray([[r.timestamp, r.range_m, r.valid] for r in self.ranges]),
               'left_K': np.asarray(self.info['left'].k).reshape(3, 3),
               'right_K': np.asarray(self.info['right'].k).reshape(3, 3),
               'left_frame_id': left.header.frame_id, 'right_frame_id': right.header.frame_id,
               'left_P': np.asarray(self.info['left'].p), 'right_P': np.asarray(self.info['right'].p),
               'left_D': np.asarray(self.info['left'].d), 'right_D': np.asarray(self.info['right'].d),
               'R_left_rectification': self.geometry.R_left_rectification,
               'R_right_rectification': self.geometry.R_right_rectification}
        return frame, position, quaternion, measurement, covered, raw, wait
