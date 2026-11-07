#!/usr/bin/env python3
"""Launch-free integration check for the flight interface.

Starts the real ``flight_interface`` node as a subprocess, stands in for PX4
with a fake ``vehicle_status`` / ``vehicle_local_position`` publisher and for
cuVSLAM with a fake odometry publisher, then asserts the two properties that
cannot be tested without a live DDS graph:

  1. ``OffboardControlMode`` really arrives at >= 9 Hz on the wire (PX4 drops
     out of Offboard below ~2 Hz and refuses to enter it without the stream).
  2. A pose setpoint above the ceiling comes back **clamped**, not dropped,
     in the published ``TrajectorySetpoint``; its horizontal position (30 m
     out, there is no geofence) passes through untouched.

Not named ``test_*.py`` on purpose: it needs a DDS peer and several seconds,
so it must not be swept up by the unit-test run.  Run it inside the container:

    source /opt/ros/humble/setup.bash
    source /root/skybeta_ws/install/setup.bash
    python3 src/skybeta_flight_interface/test/integration_offboard.py

Exits 0 on success, 1 on failure, printing every measurement.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
)

from skybeta_flight_interface.topics import resolve_endpoints

MEASURE_S = 4.0
SETTLE_S = 2.0
MIN_RATE_HZ = 9.0

# 30 m out on a 3-4-5 bearing at 50 m up: over the ceiling, and far enough
# out that the old 10 m geofence would have caught it.
FAR_POSE_ENU = (18.0, 24.0, 50.0)
# Only the height is clamped: ENU (18, 24, 3) -> NED (24, 18, -3).
EXPECTED_NED = (24.0, 18.0, -3.0)
TOLERANCE_M = 0.01


class Harness(Node):
    """Fake PX4 + fake cuVSLAM + a tap on everything the interface emits."""

    def __init__(self, topics):
        super().__init__('integration_harness')
        px4_out_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        px4_in_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # --- stand in for PX4 -----------------------------------------
        self.status_pub = self.create_publisher(
            VehicleStatus, topics['vehicle_status'], px4_out_qos)
        self.local_pub = self.create_publisher(
            VehicleLocalPosition, topics['vehicle_local_position'],
            px4_out_qos)
        # --- stand in for cuVSLAM -------------------------------------
        self.odom_pub = self.create_publisher(
            Odometry, '/visual_slam/tracking/odometry', 10)
        # --- the competitor -------------------------------------------
        self.pose_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)

        # --- taps ------------------------------------------------------
        self.control_stamps = []
        self.setpoints = []
        self.states = []
        self.create_subscription(
            OffboardControlMode, topics['offboard_control_mode'],
            lambda _m: self.control_stamps.append(time.monotonic()),
            px4_in_qos)
        self.create_subscription(
            TrajectorySetpoint, topics['trajectory_setpoint'],
            self.setpoints.append, px4_in_qos)
        self.create_subscription(
            String, '/skybeta/state',
            lambda m: self.states.append(json.loads(m.data)), 10)

        self.create_timer(0.05, self.publish_fakes)

    def publish_fakes(self):
        now_us = self.get_clock().now().nanoseconds // 1000
        status = VehicleStatus()
        status.timestamp = now_us
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.status_pub.publish(status)

        local = VehicleLocalPosition()
        local.timestamp = now_us
        local.xy_valid = True
        local.z_valid = True
        self.local_pub.publish(local)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def send_far_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(v) for v in FAR_POSE_ENU)
        pose.pose.orientation.w = 0.0     # yaw not commanded
        self.pose_pub.publish(pose)


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    failures = []
    rclpy.init()
    topics = resolve_endpoints()
    print('resolved /fmu topic names:')
    for key, value in sorted(topics.items()):
        print(f'  {key:24s} {value}')

    harness = Harness(topics)
    # A file, not a PIPE: the node logs continuously and nothing reads the
    # pipe until the end, so a PIPE deadlocks the moment its 64 KB buffer
    # fills.
    log = tempfile.NamedTemporaryFile(
        prefix='flight_interface-', suffix='.log', delete=False)
    node = subprocess.Popen(
        ['ros2', 'run', 'skybeta_flight_interface', 'flight_interface'],
        stdout=log, stderr=subprocess.STDOUT)
    try:
        print(f'\nwaiting {SETTLE_S:.0f}s for discovery...')
        spin_for(harness, SETTLE_S)

        # --- 1. heartbeat rate ----------------------------------------
        harness.control_stamps.clear()
        start = time.monotonic()
        spin_for(harness, MEASURE_S)
        elapsed = time.monotonic() - start
        count = len(harness.control_stamps)
        rate = count / elapsed if elapsed > 0 else 0.0
        print(f'\nOffboardControlMode: {count} messages in {elapsed:.2f}s '
              f'= {rate:.2f} Hz  (need >= {MIN_RATE_HZ})')
        if rate < MIN_RATE_HZ:
            failures.append(
                f'heartbeat rate {rate:.2f} Hz < {MIN_RATE_HZ} Hz')
        gaps = [b - a for a, b in zip(harness.control_stamps,
                                      harness.control_stamps[1:])]
        if gaps:
            print(f'  gaps: min {min(gaps) * 1e3:.1f} ms  '
                  f'max {max(gaps) * 1e3:.1f} ms')
            if max(gaps) > 0.5:
                failures.append(
                    f'heartbeat gap {max(gaps) * 1e3:.0f} ms > 500 ms '
                    '(PX4 would drop Offboard)')

        # --- 2. ceiling clamp on the wire, no horizontal clamp ---------
        harness.setpoints.clear()
        for _ in range(5):
            harness.send_far_pose()
            spin_for(harness, 0.2)
        spin_for(harness, 1.0)
        if not harness.setpoints:
            failures.append('no TrajectorySetpoint received at all')
        else:
            position = [float(v) for v in harness.setpoints[-1].position]
            print(f'\npose setpoint ENU {FAR_POSE_ENU} '
                  f'(30 m out, 50 m up)\n  -> TrajectorySetpoint NED '
                  f'[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
                  f'  expected {list(EXPECTED_NED)}')
            errors = [abs(got - want)
                      for got, want in zip(position, EXPECTED_NED)]
            if max(errors) > TOLERANCE_M:
                failures.append(
                    f'clamped setpoint {position} != {list(EXPECTED_NED)}')
            else:
                print('  clamped, not rejected -- correct')

        # --- 3. /skybeta/state ----------------------------------------
        if not harness.states:
            failures.append('no /skybeta/state received')
        else:
            state = harness.states[-1]
            print(f'\n/skybeta/state: armed={state["armed"]} '
                  f'nav_state={state["nav_state_name"]} '
                  f'offboard_active={state["offboard_active"]} '
                  f'envelope_ok={state["envelope_ok"]} '
                  f'clamped={state["clamped"]}')
            if not state['offboard_active']:
                failures.append('state did not report offboard_active')
            if state['clamped'] != ['max_altitude']:
                failures.append(
                    f'state reported clamp {state["clamped"]}, '
                    "expected ['max_altitude'] only")
    finally:
        node.terminate()
        try:
            node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait()
        log.close()
        harness.destroy_node()
        rclpy.shutdown()
        with open(log.name, 'r', encoding='utf-8', errors='replace') as handle:
            output = handle.read()
        os.unlink(log.name)
        print('\n--- flight_interface output ---')
        print(output.strip())

    print()
    if failures:
        for failure in failures:
            print(f'FAIL: {failure}')
        return 1
    print('PASS: heartbeat rate, ceiling clamp and /skybeta/state all good')
    return 0


if __name__ == '__main__':
    sys.exit(main())
