"""Read-only measured stereo mapping in cuVSLAM odometry; no particle filter."""
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, String

from .inputs import Inputs
from .compact_recording import CompactRecorder
from .sensor_geometry import pose_matrix

DEFAULTS = {
    'left_image_topic': '/oak/left/image_mono', 'right_image_topic': '/oak/right/image_mono',
    'left_info_topic': '/oak/left/camera_info', 'right_info_topic': '/oak/right/camera_info',
    'odometry_topic': '/visual_slam/tracking/odometry', 'imu_topic': '/oak/imu/data',
    'range_topic': '/course/downward_range', 'px4_range_topic': '',
    'body_frame': 'base_link', 'odom_frame': 'odom', 'update_hz': 5., 'stereo_slop_s': .003,
    'range_position_body_m': [0., 0., 0.], 'range_fov_deg': 1., 'range_variance_m2': .0005,
    'range_max_age_s': .1, 'px4_time_offset_s': 0.,
    'log_root': '~/team_ws/logs/odom_10cm',
}


class ImuCoverage:
    """Inputs compatibility only: retain timestamp coverage, never estimate attitude."""
    def __init__(self):
        self.times, self.quaternions, self.gaps = [], [], []
        self.last_time = None

    def add_batch(self, times, gyro, accel):
        for stamp, angular, linear in zip(times, gyro, accel):
            if not np.isfinite(np.r_[stamp, angular, linear]).all():
                continue
            if self.last_time is None or stamp > self.last_time:
                self.times.append(float(stamp)); self.last_time = float(stamp)


class OdometryInputs(Inputs):
    def on_odom(self, message):
        if message.header.frame_id != self.config['odom_frame']:
            self.node.get_logger().error('Unexpected odometry frame; refusing map integration', throttle_duration_sec=5)
            return
        super().on_odom(message)


class StereoMapLogger(Node):
    def __init__(self):
        super().__init__('odom_stereo_map')
        self.config = {k:self.declare_parameter(k,v).value for k,v in DEFAULTS.items()}
        if not 0 < self.config['update_hz'] <= 30:
            raise ValueError('update_hz must be in (0,30]')
        self.recorder = CompactRecorder(Path(self.config['log_root']).expanduser(),
            metadata={'parameters':self.config, 'purpose':'Stereo observations accumulated using cuVSLAM only',
                      'particle_filter_enabled':False, 'cloud_frame':self.config['odom_frame'],
                      'alignment':'Identity from odometry to output frame; no prior map',
                      'storage':'Bounded text, JPEG/depth PNG previews and PLY maps; no NPZ or rosbag'},
            config_paths={name:Path(__file__).with_name(name) for name in
                          ('stereo_map_node.py','compact_recording.py','odom_10cm_test.py')},
            map_identity={'source':'No nominal map', 'frame':self.config['odom_frame']})
        try:
            self.inputs = OdometryInputs(self, SimpleNamespace(inertial=ImuCoverage()), self.config)
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import PoseStamped, TwistStamped
            from rosidl_runtime_py.convert import message_to_ordereddict
            self.event_subscriptions = []
            for topic, kind in [('/odom_test/mission_status', String), ('/skybeta/state', String),
                                ('/odom_test/map_status', String),
                                ('/skybeta/setpoint/pose', PoseStamped),
                                ('/skybeta/setpoint/twist', TwistStamped),
                                (self.config['odometry_topic'], Odometry)]:
                def record(msg, topic=topic):
                    value = json.loads(msg.data) if isinstance(msg, String) else message_to_ordereddict(msg)
                    self.recorder.event(topic, value)
                from rclpy.qos import qos_profile_sensor_data
                qos = qos_profile_sensor_data if kind is Odometry else 10
                self.event_subscriptions.append(self.create_subscription(kind, topic, record, qos))
        except Exception:
            self.recorder.close(); raise
        self.depth_pub = self.create_publisher(Image, '/odom_test/stereo_depth', 2)
        self.cloud_pub = self.create_publisher(PointCloud2, '/odom_test/observed_points', 2)
        self.status_pub = self.create_publisher(String, '/odom_test/map_status', 10)
        self.frames, self.last_time, self.last_error = 0, None, 'waiting_for_stereo_and_odometry'
        self.create_timer(.02, self.process); self.create_timer(1., self.status)
        self.get_logger().info(f'Odometry-only stereo map logs: {self.recorder.path}; no flight commands')

    def process(self):
        try:
            observation = self.inputs.next_observation()
            if observation is None:
                return
            frame, position, quaternion, measurement, covered, raw, wait = observation
            transform = pose_matrix(position, quaternion)
            y,x = np.nonzero(np.isfinite(frame.depth_m) & (frame.depth_m > 0))
            camera = (np.c_[x,y,np.ones(len(x))] @ np.linalg.inv(frame.K).T) * frame.depth_m[y,x,None]
            points = camera @ frame.T_body_camera[:3,:3].T + frame.T_body_camera[:3,3]
            arrays = dict({k:v for k,v in raw.items() if k not in
                          ('left_image','right_image','imu_samples','odom_samples','range_samples')},
                          timestamp=frame.timestamp, depth_m=frame.depth_m, K=frame.K,
                          T_body_camera=frame.T_body_camera, odom_position=position,
                          odom_quaternion_xyzw=quaternion, T_odom_body=transform,
                          range_m=measurement.range_m, range_valid=measurement.valid)
            diagnostics = dict(stereo=self.inputs.estimator.last_diagnostics, imu_covered=covered,
                               synchronization_wait_s=wait, frame=self.config['odom_frame'],
                               particle_filter_enabled=False)
            if not self.recorder.record_frame(arrays,diagnostics,points_body=points,T_map_body_vo=transform):
                raise RuntimeError(f'Logging rejected: {self.recorder.health}')
            stamp = rclpy.time.Time(nanoseconds=int(round(frame.timestamp*1e9))).to_msg()
            depth = self.inputs.bridge.cv2_to_imgmsg(frame.depth_m, encoding='32FC1')
            depth.header = Header(stamp=stamp, frame_id='odom_test_right_rectified_optical')
            self.depth_pub.publish(depth)
            world = np.asarray(points @ transform[:3,:3].T + transform[:3,3],dtype='<f4')
            cloud = PointCloud2(header=Header(stamp=stamp,frame_id=self.config['odom_frame']),
                                height=1,width=len(world),point_step=12,row_step=12*len(world),
                                is_bigendian=False,is_dense=True)
            cloud.fields = [PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1)
                            for i,n in enumerate(('x','y','z'))]
            cloud.data = world.tobytes(); self.cloud_pub.publish(cloud)
            self.frames += 1; self.last_time = time.monotonic(); self.last_error = ''
        except Exception as exc:
            self.last_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().warning(self.last_error, throttle_duration_sec=2)
            self.recorder.record_frame({}, {'event':'processing_error','error':self.last_error})

    def status(self):
        self.status_pub.publish(String(data=json.dumps(dict(ready=(self.frames > 0 and self.last_time is not None
            and time.monotonic()-self.last_time < 2 and not self.last_error
            and not self.recorder.health.get('error')
            and not self.recorder.events.error and not self.recorder.transitions.error),
            frames=self.frames,last_error=self.last_error,
            seconds_since_frame=None if self.last_time is None else time.monotonic()-self.last_time,
            log_directory=str(self.recorder.path),recorder=self.recorder.health,storage='compact; no rosbag or NPZ',
            particle_filter_enabled=False,cloud_frame=self.config['odom_frame']))))

    def close(self):
        self.recorder.close()


def main(args=None):
    rclpy.init(args=args); node=None
    try:
        node=StereoMapLogger();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:node.close()
            finally:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
