#!/usr/bin/env python3
"""Read-only check: hold the drone level and still over a flat floor, props safe.

Inside the ROS container: python3 tools/course_selftest.py --height 0.80
Height is measured vertically from the downward sensor, NOT from the drone body.
PX4 range is the default; use --range-source range for a ROS Range publisher.
"""
import argparse
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from nav_msgs.msg import Odometry


def pixels(msg):
    types = {'mono8': ('u1', 1), '32FC1': ('f4', 4), '16UC1': ('u2', 2)}
    if msg.encoding not in types or msg.height <= 0 or msg.width <= 0:
        raise ValueError('unsupported encoding or empty dimensions')
    dtype, size = types[msg.encoding]
    if msg.step < msg.width * size or len(msg.data) != msg.step * msg.height:
        raise ValueError('invalid step or payload length')
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    return np.ndarray((msg.height, msg.width), dtype, buffer=bytes(msg.data), strides=(msg.step, size))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--height', type=float)
    parser.add_argument('--tolerance', type=float, default=.05)
    parser.add_argument('--body-frame', default='base_link')
    parser.add_argument('--range-source', choices=['range', 'px4'], default='px4')
    parser.add_argument('--range-topic')
    args, ros_args = parser.parse_known_args()
    if args.seconds <= 0 or args.tolerance <= 0 or (args.height is not None and args.height <= 0):
        parser.error('duration, tolerance and reference height must be positive')
    range_type = Range
    if args.range_source == 'px4':
        from px4_msgs.msg import DistanceSensor
        range_type = DistanceSensor
    range_topic = args.range_topic or ('/course/downward_range' if args.range_source == 'range' else '/fmu/out/distance_sensor')
    topics = {eye: (Image, f'/oak/{eye}/image_mono') for eye in ('left', 'right')}
    topics.update({eye + '_info': (CameraInfo, f'/oak/{eye}/camera_info') for eye in ('left', 'right')})
    topics.update(imu=(Imu, '/oak/imu/data'), odom=(Odometry, '/visual_slam/tracking/odometry'), depth=(Image, '/course/stereo_depth'), range=(range_type, range_topic))
    rclpy.init(args=ros_args)
    node = rclpy.create_node('course_readonly_selftest')
    latest, stamps, received, ranges, gravity = {}, {k: [] for k in topics}, {}, [], []
    failures = []

    def capture(key, msg):
        latest[key], received[key] = msg, time.monotonic()
        stamp = msg.timestamp * 1e-6 if key == 'range' and args.range_source == 'px4' else msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamps[key].append(stamp)
        if key == 'range':
            value = msg.current_distance if args.range_source == 'px4' else msg.range
            lower, upper = (msg.min_distance, msg.max_distance) if args.range_source == 'px4' else (msg.min_range, msg.max_range)
            downward = args.range_source != 'px4' or (msg.orientation == 25 and msg.signal_quality != 0)
            if downward and np.isfinite(value) and lower <= value <= upper:
                ranges.append(float(value))
        if key == 'imu':
            a = msg.linear_acceleration
            gravity.append(np.linalg.norm([a.x, a.y, a.z]))

    def check(label, ok, detail):
        print(f'{"PASS" if ok else "FAIL"} {label}: {detail}')
        if not ok:
            failures.append(label)

    subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: capture(key, msg), qos_profile_sensor_data) for key, (kind, topic) in topics.items()]
    print(f'Keep stationary and level for {args.seconds:g}s; reference height={args.height}.')
    until = time.monotonic() + args.seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    now = node.get_clock().now().nanoseconds * 1e-9
    for key in topics:
        ts = np.asarray(stamps[key])
        age = now-ts[-1] if len(ts) else float('inf')
        check(key + ' stream', len(ts) >= 2 and np.all(np.diff(ts) > 0) and time.monotonic()-received.get(key, 0) < 1., f'{len(ts)} messages; receive freshness/order')
        check(key + ' clock', bool(len(ts)) and -.1 <= age <= 1., f'source age={age:.3f}s (stale/clock mismatch if outside [-.1,1])')
    for eye in ('left', 'right', 'depth'):
        try:
            msg = latest[eye]
            data = pixels(msg)
            if eye == 'depth':
                fraction = float(np.mean(np.isfinite(data) & (data > 0)))
                check('depth pixels', msg.encoding in ('32FC1', '16UC1') and fraction > .05, f'valid positive fraction={fraction:.1%}')
            else:
                info = latest[eye + '_info']
                check(eye + ' pixels/calibration', msg.encoding == 'mono8' and np.std(data) > 1 and (msg.width, msg.height) == (info.width, info.height) and np.isfinite(info.k).all() and info.k[0] > 0 and info.k[4] > 0, f'{msg.width}x{msg.height}, std={np.std(data):.2f}')
        except (KeyError, ValueError, TypeError) as exc:
            check(eye + ' payload', False, str(exc))
    if stamps['left'] and stamps['right']:
        skew = np.min(np.abs(np.asarray(stamps['left'])[:, None]-np.asarray(stamps['right'])[None, :]), axis=1)
        check('stereo pairing', np.mean(skew < .01) > .9, f'{np.mean(skew < .01):.1%} left frames matched within 10ms')
    if 'odom' in latest:
        odom = latest['odom']
        q = odom.pose.pose.orientation
        p = odom.pose.pose.position
        check('odometry pose/frame', np.isfinite([p.x, p.y, p.z]).all() and abs(np.linalg.norm([q.x, q.y, q.z, q.w])-1) < .01 and odom.child_frame_id == args.body_frame and bool(odom.header.frame_id), f'{odom.header.frame_id} -> {odom.child_frame_id}')
    check('stationary IMU', len(gravity) >= 2 and abs(np.median(gravity)-9.81) < 1. and np.std(gravity) < .5, 'acceleration norm near gravity, low variation')
    check('range validity', len(ranges) >= 2, f'{len(ranges)} finite in-bounds samples')
    if args.height is None:
        print('NOT TESTED range accuracy: supply --height with a physically measured sensor-to-floor distance.')
    else:
        measured = float(np.median(ranges)) if ranges else float('nan')
        check('range accuracy (level sensor over flat floor)', abs(measured-args.height) <= args.tolerance, f'median={measured:.3f}m reference={args.height:.3f}m tolerance={args.tolerance:.3f}m')
    node.destroy_node()
    rclpy.shutdown()
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
