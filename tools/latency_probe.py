#!/usr/bin/env python3
"""Measure the visual-odometry chain latency, to set EKF2_EV_DELAY.

Run inside the skybeta container while the full stack is up:

    python3 /workspace/tools/latency_probe.py --seconds 30

Two numbers are collected, both against the node's ROS clock:

  driver leg   /oak/left/image_raw:  header.stamp -> probe arrival.
               Camera exposure/readout -> depthai -> ROS publish -> local
               DDS hop.  This is a lower bound on everything downstream.

  full chain   /fmu/in/vehicle_visual_odometry: timestamp_sample -> probe
               arrival.  px4_odom_bridge copies the cuVSLAM odometry header
               stamp into timestamp_sample, and cuVSLAM copies the image
               stamp into that, so timestamp_sample IS the original image
               stamp.  The delta therefore covers image -> depthai ->
               ImageFormatConverterNode (GPU bgr8->mono8) -> cuVSLAM ->
               bridge -> local DDS.

EKF2_EV_DELAY wants the age of a measurement at the moment EKF2 fuses it.
This probe stops at the bridge's DDS publish; it does not include the
uXRCE-DDS serialisation and the 921600-baud serial hop to the FC (small,
but real).  Treat the printed number as a starting point, and cross-check
it on the vehicle with the authoritative PX4 method: set SDLOG_PROFILE bit
7 ("EKF2 replay"), fly/hand-move, then run the replay and inspect the
innovation timing (PX4 docs, "EKF2 tuning / vision delay").  Never ship a
guessed 50.

Self-check without hardware:

    python3 tools/latency_probe.py --selftest
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Image


DEFAULT_IMAGE_TOPIC = '/oak/left/image_raw'
DEFAULT_ODOM_TOPIC = '/fmu/in/vehicle_visual_odometry'


def best_effort_qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def percentile(values, pct: float) -> float:
    """Linear-interpolated percentile of an unsorted sequence."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * (pct / 100.0)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[int(position)]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def summarize(values):
    """-> dict of n / p50 / p95 / max / min / mean, all in the input unit."""
    if not values:
        return {'n': 0, 'p50': float('nan'), 'p95': float('nan'),
                'max': float('nan'), 'min': float('nan'),
                'mean': float('nan')}
    return {
        'n': len(values),
        'p50': percentile(values, 50.0),
        'p95': percentile(values, 95.0),
        'max': max(values),
        'min': min(values),
        'mean': sum(values) / len(values),
    }


class LatencyProbe(Node):

    def __init__(self, image_topic=DEFAULT_IMAGE_TOPIC,
                 odom_topic=DEFAULT_ODOM_TOPIC, subscribe=True):
        super().__init__('latency_probe')
        self.image_topic = image_topic
        self.odom_topic = odom_topic
        self.image_latencies_ms = []
        self.odom_latencies_ms = []
        if subscribe:
            self.create_subscription(
                Image, image_topic, self.image_cb, best_effort_qos())
            self.create_subscription(
                VehicleOdometry, odom_topic, self.odom_cb, best_effort_qos())

    def now_ns(self) -> int:
        """Overridable so --selftest can drive a deterministic clock."""
        return self.get_clock().now().nanoseconds

    def image_cb(self, msg):
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self.image_latencies_ms.append((self.now_ns() - stamp_ns) / 1e6)

    def odom_cb(self, msg):
        # timestamp_sample is microseconds in the ROS-time domain -- the
        # bridge fills it from the cuVSLAM odometry header stamp.
        sample_ns = int(msg.timestamp_sample) * 1000
        self.odom_latencies_ms.append((self.now_ns() - sample_ns) / 1e6)


def format_report(probe, elapsed_s) -> str:
    image = summarize(probe.image_latencies_ms)
    odom = summarize(probe.odom_latencies_ms)
    lines = []
    lines.append('')
    lines.append('=' * 72)
    lines.append(f'latency probe: {elapsed_s:.1f} s')
    lines.append('=' * 72)
    for label, topic, stats, leg in (
        ('driver leg', probe.image_topic, image,
         'image header.stamp -> probe arrival'),
        ('full chain', probe.odom_topic, odom,
         'image header.stamp -> bridge publish'),
    ):
        lines.append(f'{label}  {topic}')
        lines.append(f'    {leg}')
        if stats['n'] == 0:
            lines.append('    NO MESSAGES RECEIVED -- is the stack running, '
                         'and ROS_DOMAIN_ID the same?')
            continue
        rate = stats['n'] / elapsed_s if elapsed_s > 0 else float('nan')
        lines.append(
            f"    n={stats['n']}  ({rate:.1f} Hz)  "
            f"p50={stats['p50']:.1f} ms  p95={stats['p95']:.1f} ms  "
            f"max={stats['max']:.1f} ms  min={stats['min']:.1f} ms"
        )
    lines.append('')
    if odom['n'] == 0:
        lines.append('EKF2_EV_DELAY: cannot recommend a value -- no '
                     'VehicleOdometry received.')
    else:
        recommended = int(round(odom['p50']))
        clamped = max(0, min(300, recommended))
        note = ''
        if clamped != recommended:
            note = f'; clamped from {recommended} into the 0-300 range'
        lines.append(
            f'Recommended EKF2_EV_DELAY = {clamped} ms   '
            f'(p50 of the full chain, rounded{note})'
        )
        if odom['n'] > 0 and image['n'] > 0:
            lines.append(
                f"    of which {image['p50']:.1f} ms is the driver leg; "
                f"{odom['p50'] - image['p50']:.1f} ms is "
                'GPU mono8 conversion + cuVSLAM + bridge.'
            )
    lines.append('')
    lines.append("This stops at the bridge's DDS publish. It excludes the")
    lines.append('uXRCE-DDS serial hop to the FC, so the true fusion age is')
    lines.append('slightly larger. Cross-check on the vehicle with the')
    lines.append('authoritative method -- SDLOG_PROFILE bit 7 (EKF2 replay),')
    lines.append('then replay the log and inspect the innovation timing.')
    lines.append('=' * 72)
    return '\n'.join(lines)


def run_probe(seconds, image_topic, odom_topic) -> int:
    rclpy.init()
    probe = LatencyProbe(image_topic=image_topic, odom_topic=odom_topic)
    probe.get_logger().info(
        f'sampling {image_topic} and {odom_topic} for {seconds:.0f} s ...')
    start = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - start) < seconds:
            rclpy.spin_once(probe, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - start
    print(format_report(probe, elapsed))
    got_odom = bool(probe.odom_latencies_ms)
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if got_odom else 1


def _synthetic_image(stamp_ns):
    msg = Image()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = 'oak_left_camera_optical_frame'
    return msg


def _synthetic_odometry(stamp_ns):
    msg = VehicleOdometry()
    msg.timestamp_sample = stamp_ns // 1000
    msg.timestamp = stamp_ns // 1000
    return msg


def selftest() -> int:
    """Feed known latencies through the real callbacks and check the math."""
    rclpy.init()
    probe = LatencyProbe(subscribe=False)
    base_ns = 1_700_000_000 * 1_000_000_000  # a plausible wall-clock epoch
    fake_now = {'ns': base_ns}
    probe.now_ns = lambda: fake_now['ns']

    # Driver leg: 10, 12, ... 30 ms.  Full chain: driver leg + 55 ms.
    image_truth_ms = [10.0 + 2.0 * i for i in range(11)]
    for i, latency_ms in enumerate(image_truth_ms):
        stamp_ns = base_ns + i * 33_333_333
        fake_now['ns'] = stamp_ns + int(latency_ms * 1e6)
        probe.image_cb(_synthetic_image(stamp_ns))
    odom_truth_ms = [ms + 55.0 for ms in image_truth_ms]
    for i, latency_ms in enumerate(odom_truth_ms):
        stamp_ns = base_ns + i * 33_333_333
        fake_now['ns'] = stamp_ns + int(latency_ms * 1e6)
        probe.odom_cb(_synthetic_odometry(stamp_ns))

    print(format_report(probe, elapsed_s=11 * 0.0333333))

    failures = []
    image = summarize(probe.image_latencies_ms)
    odom = summarize(probe.odom_latencies_ms)
    # timestamp_sample is microseconds, so the odometry leg carries up to
    # 1 us (0.001 ms) of quantisation; compare at 0.1 ms resolution.
    checks = [
        ('image n', image['n'], 11),
        ('image p50', round(image['p50'], 1), 20.0),
        ('image max', round(image['max'], 1), 30.0),
        ('odom n', odom['n'], 11),
        ('odom p50', round(odom['p50'], 1), 75.0),
        ('odom max', round(odom['max'], 1), 85.0),
        ('percentile p95 of 0..100', percentile(list(range(101)), 95.0), 95.0),
        ('empty summary n', summarize([])['n'], 0),
    ]
    for name, got, want in checks:
        close = isinstance(got, float) and abs(got - want) < 1e-6
        ok = got == want or close
        print(f"  {'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")
        if not ok:
            failures.append(name)
    recommended = int(round(odom['p50']))
    ok = recommended == 75
    print(f"  {'ok  ' if ok else 'FAIL'} recommended EKF2_EV_DELAY: "
          f'{recommended} (want 75)')
    if not ok:
        failures.append('recommended')

    probe.destroy_node()
    rclpy.shutdown()
    if failures:
        print(f'\nSELFTEST FAILED: {failures}')
        return 1
    print('\nSELFTEST PASSED')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--seconds', type=float, default=30.0,
                        help='sampling window (default: 30)')
    parser.add_argument('--image-topic', default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument('--odom-topic', default=DEFAULT_ODOM_TOPIC)
    parser.add_argument('--selftest', action='store_true',
                        help='run synthetic messages through the callbacks '
                             'and verify the statistics; no hardware needed')
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    return run_probe(args.seconds, args.image_topic, args.odom_topic)


if __name__ == '__main__':
    sys.exit(main())
