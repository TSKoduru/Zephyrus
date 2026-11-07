"""Opt-in bounded course feedback through the existing flight-interface contract.

No arming/mode commands and no PX4 topics. A pilot owns flight initiation.
Zero twist primes the existing interface; faults stop publication and request HOLD.
"""
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .controller import TimedPath, CourseFollower
from .sensor_geometry import interpolate_pose


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class CourseController(Node):
    def __init__(self, **kwargs):
        super().__init__('course_controller', **kwargs)
        defaults = dict(enable_control=False, trajectory_path='', map_frame='course_map',
                        odom_frame='odom', body_frame='base_link', max_pose_age_s=.4,
                        max_xy_std_m=.25, max_z_std_m=.25, max_yaw_std_rad=.35,
                        max_odom_gap_s=.1, max_speed_mps=.3, max_acceleration_mps2=.8,
                        max_yaw_rate_rps=.6, max_position_error_m=.25, publish_rate_hz=20.,
                        require_range=True, require_imu=True, px4_clock_offset_s=0.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.settings.items():
            if isinstance(value, float) and (not np.isfinite(value) or (key != 'px4_clock_offset_s' and value <= 0)):
                raise ValueError(f'{key} must be positive and finite')
        path = self.settings['trajectory_path'] or str(Path(get_package_share_directory('course_localization')) / 'trajectory' / 'course.npz')
        self.path = TimedPath.load(path, max_speed=self.settings['max_speed_mps'])
        self.follower = CourseFollower(self.path, max_speed=self.settings['max_speed_mps'],
            max_acceleration=self.settings['max_acceleration_mps2'],
            max_yaw_rate=self.settings['max_yaw_rate_rps'],
            max_position_error=self.settings['max_position_error_m'],
            max_measurement_age=self.settings['max_pose_age_s'])
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.status_pub = self.create_publisher(String, '/course/controller_status', 10)
        self.hold_client = self.create_client(Trigger, '/skybeta/hold')
        self.odom = deque(maxlen=600)
        self.attitudes = deque(maxlen=600)
        self.pf = None
        self.health, self.flight = {}, {}
        self.health_received = self.flight_received = self.pf_received = 0.
        self.active = self.locked = self.ever_offboard = False
        self.fault = ''
        self.hold_needed = False
        self.hold_future = None
        self.last_hold_attempt = 0.
        self.last_tick = time.monotonic()
        self.create_subscription(Odometry, '/course/particle_pose', self.pose_cb, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/course/status', self.health_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.flight_cb, 10)
        self.create_timer(1. / self.settings['publish_rate_hz'], self.tick)
        self.get_logger().info('Course controller ready; enable_control defaults false. Stop all other setpoint publishers before enabling.')

    def pose_cb(self, msg):
        if self.pf is not None and stamp_seconds(msg) <= stamp_seconds(self.pf):
            self.fail('PF timestamps reset or arrived out of order')
        self.pf, self.pf_received = msg, time.monotonic()

    def odom_cb(self, msg):
        if msg.header.frame_id != self.settings['odom_frame'] or msg.child_frame_id != self.settings['body_frame']:
            self.odom.clear()
            self.fail('odometry frame mismatch')
            return
        t = stamp_seconds(msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.) > .01:
            self.odom.clear()
            self.fail('invalid odometry pose')
            return
        if self.odom and t <= self.odom[-1][0]:
            self.odom.clear()
            self.fail('odometry timestamps reset or arrived out of order')
        self.odom.append((t, values[:3], values[3:]))

    def health_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('status must be an object')
            self.health, self.health_received = data, time.monotonic()
        except (ValueError, TypeError):
            self.health = {}

    def attitude_cb(self, msg):
        stamp = msg.timestamp_sample * 1e-6 + self.settings['px4_clock_offset_s']
        q = np.asarray(msg.q, dtype=float)
        if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > .01:
            self.attitudes.clear()
            self.fail('invalid PX4 attitude')
            return
        rotation = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
        enu_from_ned = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        enu_flu = enu_from_ned @ rotation @ np.diag([1., -1., -1.])
        if self.attitudes and stamp <= self.attitudes[-1][0]:
            self.attitudes.clear()
            self.fail('PX4 attitude timestamps reset or out of order')
        self.attitudes.append((stamp, Rotation.from_matrix(enu_flu).as_quat()))

    def flight_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('armed'), bool) or not isinstance(data.get('offboard_active'), bool):
                raise ValueError('flight state must include boolean armed/offboard_active')
        except (ValueError, TypeError):
            self.flight = {}
            return
        if self.flight.get('armed') is True and data['armed'] is False:
            self.active = self.locked = self.ever_offboard = False
            self.fault = ''
            self.hold_needed = False
            self.follower.reset(self.get_clock().now().nanoseconds * 1e-9)
        if self.ever_offboard and not data['offboard_active']:
            self.fail('Offboard lost; disarm required before restart')
        self.flight, self.flight_received = data, time.monotonic()

    def fail(self, reason):
        if self.active or self.ever_offboard:
            if not self.locked:
                self.get_logger().error(reason)
                self.hold_needed = True
            self.locked, self.active = True, False
            self.fault = self.fault or reason

    def request_hold(self, mono):
        if self.hold_future is not None:
            if not self.hold_future.done():
                return
            try:
                response = self.hold_future.result()
                self.hold_needed = not response.success
            except Exception as exc:
                self.get_logger().error(f'HOLD service failed: {exc}')
            self.hold_future = None
        if self.hold_needed and mono-self.last_hold_attempt >= 1.:
            self.last_hold_attempt = mono
            if self.hold_client.service_is_ready():
                self.hold_future = self.hold_client.call_async(Trigger.Request())
            else:
                self.get_logger().error('HOLD service unavailable; twist publication stopped, interface timeout remains fallback')

    def measurement(self, now, mono):
        cfg = self.settings
        if self.pf is None or mono-self.pf_received > cfg['max_pose_age_s']:
            raise ValueError('PF missing or receive timeout')
        msg = self.pf
        stamp = stamp_seconds(msg)
        if not 0 <= now-stamp <= cfg['max_pose_age_s']:
            raise ValueError('PF source stamp stale or clock mismatch')
        if msg.header.frame_id != cfg['map_frame'] or msg.child_frame_id != cfg['body_frame']:
            raise ValueError('PF frame mismatch')
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
        c4 = covariance[np.ix_([0, 1, 2, 5], [0, 1, 2, 5])]
        if not np.isfinite(covariance).all() or not np.allclose(c4, c4.T) or np.linalg.eigvalsh(c4).min() < -1e-9:
            raise ValueError('invalid PF covariance')
        if np.linalg.eigvalsh(c4[:2, :2]).max() > cfg['max_xy_std_m']**2 or c4[2, 2] > cfg['max_z_std_m']**2 or c4[3, 3] > cfg['max_yaw_std_rad']**2:
            raise ValueError('PF covariance exceeds configured limit')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        if not np.isfinite([p.x, p.y, p.z, *quat]).all() or abs(np.linalg.norm(quat)-1.) > .01:
            raise ValueError('invalid PF pose')
        pose = np.array([p.x, p.y, p.z, Rotation.from_quat(quat).as_euler('xyz')[2]])
        samples = list(self.odom)
        _, odom_q = interpolate_pose([s[0] for s in samples], [s[1] for s in samples],
                                    [s[2] for s in samples], stamp, cfg['max_odom_gap_s'])
        attitudes = list(self.attitudes)
        _, control_q = interpolate_pose([s[0] for s in attitudes], np.zeros((len(attitudes), 3)),
                                       [s[1] for s in attitudes], stamp, cfg['max_odom_gap_s'])
        return pose, Rotation.from_quat(odom_q).as_euler('xyz')[2], stamp, c4, Rotation.from_quat(control_q).as_euler('xyz')[2]

    def tick(self):
        mono = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        dt, self.last_tick = mono-self.last_tick, mono
        gates, output = [], {}
        enabled = self.get_parameter('enable_control').value is True
        if not enabled:
            gates.append('control disabled')
        if self.count_publishers('/skybeta/setpoint/pose') or self.count_publishers('/skybeta/setpoint/twist') > 1:
            gates.append('competing setpoint publisher')
        if mono-self.flight_received > .6 or not self.flight.get('px4_link') or not self.flight.get('envelope_ok'):
            gates.append('flight interface missing/stale/unhealthy')
        flight_stamp = self.flight.get('stamp')
        if not isinstance(flight_stamp, (int, float)) or not -.05 <= now-flight_stamp <= .6:
            gates.append('flight state source clock stale/mismatch')
        age = self.health.get('seconds_since_frame')
        if mono-self.health_received > 1.5 or self.health.get('last_error', 'missing') or not isinstance(age, (int, float)) or not 0 <= age <= self.settings['max_pose_age_s']:
            gates.append('localization health stale/error')
        if self.flight.get('frozen'):
            gates.append('flight interface frozen')
        if self.hold_future is not None:
            gates.append('HOLD service request pending')
        measurement_health = self.health.get('last_measurement', {})
        if self.settings['require_range'] and not measurement_health.get('range_valid'):
            gates.append('valid downward range required')
        if self.settings['require_imu'] and not measurement_health.get('imu_valid'):
            gates.append('valid IMU required')
        try:
            pose, odom_yaw, stamp, covariance, control_yaw = self.measurement(now, mono)
            output.update(pose=pose.tolist(), pf_stamp=stamp, odom_interpolated_stamp=stamp,
                          control_frame_yaw=control_yaw, vo_yaw=odom_yaw,
                          vo_px4_heading_offset=float(np.arctan2(np.sin(control_yaw-odom_yaw), np.cos(control_yaw-odom_yaw))),
                          control_frame='px4_local_enu', px4_clock_offset_s=self.settings['px4_clock_offset_s'])
        except (ValueError, TypeError, IndexError) as exc:
            gates.append(str(exc))
        if gates:
            self.fail('; '.join(gates))
        if not gates and not self.locked:
            if self.flight.get('offboard_active') and self.flight.get('armed'):
                if not self.active:
                    self.follower.reset(now)
                    self.active = self.ever_offboard = True
                output.update(self.follower.update(now, dt, pose, control_yaw,
                    measurement_time=stamp, odom_time=stamp, covariance=covariance))
                if output['stop_requested']:
                    self.fail(output.get('fault') or output.get('status', 'controller stopped'))
                else:
                    self.publish_twist(output['velocity_enu'], output['yaw_rate'])
            else:
                # No pose prime: interface HOLD must not retain a startup pose.
                self.publish_twist([0., 0., 0.], 0.)
                output['status'] = 'ready_waiting_for_pilot_offboard'
        self.request_hold(mono)
        output.update(ros_time_s=now, monotonic_time_s=mono, clock='ROS source stamps; monotonic watchdogs',
                      enabled=enabled, active=self.active, latched=self.locked, fault=self.fault,
                      gates=gates, hold_pending=self.hold_needed, parameters=self.settings,
                      frame_alignment='PF map yaw to PX4 ENU/FLU heading at PF exposure timestamp')
        self.status_pub.publish(String(data=json.dumps(output, allow_nan=False)))

    def publish_twist(self, velocity, yaw_rate):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_enu'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, velocity)
        msg.twist.angular.z = float(yaw_rate)
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CourseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fail('controller shutdown')
        node.request_hold(time.monotonic())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
