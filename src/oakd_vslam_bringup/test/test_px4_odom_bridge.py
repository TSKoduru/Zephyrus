"""Unit tests for the cuVSLAM -> PX4 VehicleOdometry bridge.

Run inside the skybeta container:

    source /opt/ros/humble/setup.bash
    source /root/skybeta_ws/install/local_setup.bash   # px4_msgs
    cd src/oakd_vslam_bringup && python3 -m pytest -q test/
"""
import math

import pytest

import rclpy
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

from oakd_vslam_bringup.px4_odom_bridge import (
    AIRCRAFT_BASELINK_Q,
    NED_ENU_Q,
    PX4OdomBridge,
    ResetCounter,
    VO_STATE_FAILED,
    VO_STATE_SUCCESS,
    VO_STATE_UNKNOWN,
    is_identity_covariance,
    odometry_to_vehicle_odometry,
    quat_mul,
    resolve_pose_frame,
)

SQRT_HALF = math.sqrt(0.5)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def make_odometry(position=(0.0, 0.0, 0.0),
                  orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                  linear=(0.0, 0.0, 0.0),
                  angular=(0.0, 0.0, 0.0),
                  pose_cov=None,
                  twist_cov=None,
                  stamp=(12, 345_678_000)):
    msg = Odometry()
    msg.header.stamp.sec = stamp[0]
    msg.header.stamp.nanosec = stamp[1]
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(position[0])
    msg.pose.pose.position.y = float(position[1])
    msg.pose.pose.position.z = float(position[2])
    w, x, y, z = orientation_wxyz
    msg.pose.pose.orientation.w = float(w)
    msg.pose.pose.orientation.x = float(x)
    msg.pose.pose.orientation.y = float(y)
    msg.pose.pose.orientation.z = float(z)
    msg.twist.twist.linear.x = float(linear[0])
    msg.twist.twist.linear.y = float(linear[1])
    msg.twist.twist.linear.z = float(linear[2])
    msg.twist.twist.angular.x = float(angular[0])
    msg.twist.twist.angular.y = float(angular[1])
    msg.twist.twist.angular.z = float(angular[2])
    msg.pose.covariance = [float(v) for v in (pose_cov or [0.0] * 36)]
    msg.twist.covariance = [float(v) for v in (twist_cov or [0.0] * 36)]
    return msg


def yaw_from_wxyz(q):
    """Yaw (rad) of a wxyz quaternion; invariant under q -> -q."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def enu_yaw_quat(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class RecordingLogger:
    """Stand-in for node.get_logger() that counts calls per level."""

    def __init__(self):
        self.records = {'debug': [], 'info': [], 'warn': [],
                        'warning': [], 'error': []}

    def _add(self, level):
        return lambda msg, *a, **k: self.records[level].append(str(msg))

    def __getattr__(self, name):
        if name in ('debug', 'info', 'warn', 'warning', 'error'):
            return self._add(name)
        raise AttributeError(name)


# --------------------------------------------------------------------------
# pose / frame conversion
# --------------------------------------------------------------------------
def test_position_enu_to_ned_and_default_pose_frame():
    msg = make_odometry(position=(1.0, 2.0, 3.0))
    out = odometry_to_vehicle_odometry(msg)
    assert list(out.position) == pytest.approx([2.0, 1.0, -3.0])
    # Default must be FRD: cuVSLAM's odom frame has an arbitrary heading
    # reference, which is the definition of POSE_FRAME_FRD.
    assert out.pose_frame == VehicleOdometry.POSE_FRAME_FRD == 2


def test_pose_frame_ned_when_requested():
    msg = make_odometry(position=(1.0, 2.0, 3.0))
    out = odometry_to_vehicle_odometry(
        msg, pose_frame=resolve_pose_frame('NED'))
    assert out.pose_frame == VehicleOdometry.POSE_FRAME_NED == 1
    # position math is unaffected by the declaration
    assert list(out.position) == pytest.approx([2.0, 1.0, -3.0])


def test_resolve_pose_frame_accepts_spellings_and_rejects_junk():
    assert resolve_pose_frame('FRD') == 2
    assert resolve_pose_frame('ned') == 1
    assert resolve_pose_frame(' POSE_FRAME_NED ') == 1
    assert resolve_pose_frame(2) == 2
    with pytest.raises(ValueError):
        resolve_pose_frame('ENU')


def test_identity_orientation_matches_composed_quaternion():
    msg = make_odometry(position=(1.0, 2.0, 3.0))
    out = odometry_to_vehicle_odometry(msg)
    expected = quat_mul(quat_mul(NED_ENU_Q, (1.0, 0.0, 0.0, 0.0)),
                        AIRCRAFT_BASELINK_Q)
    assert list(out.q) == pytest.approx(list(expected), abs=1e-6)
    norm = math.sqrt(sum(float(v) ** 2 for v in out.q))
    assert norm == pytest.approx(1.0, abs=1e-6)
    # An ENU-identity attitude (body +x pointing East) is heading 90 deg
    # in NED.
    assert math.degrees(yaw_from_wxyz(out.q)) == pytest.approx(90.0, abs=1e-4)


@pytest.mark.parametrize('yaw_enu_deg', [0.0, 45.0, 90.0, 180.0, -90.0])
def test_yaw_enu_to_ned(yaw_enu_deg):
    """NED yaw is 90 deg - ENU yaw: +90 in ENU is -90 in NED."""
    yaw_enu = math.radians(yaw_enu_deg)
    msg = make_odometry(orientation_wxyz=enu_yaw_quat(yaw_enu))
    out = odometry_to_vehicle_odometry(msg)

    expected = quat_mul(quat_mul(NED_ENU_Q, enu_yaw_quat(yaw_enu)),
                        AIRCRAFT_BASELINK_Q)
    assert list(out.q) == pytest.approx(list(expected), abs=1e-6)
    norm = math.sqrt(sum(float(v) ** 2 for v in out.q))
    assert norm == pytest.approx(1.0, abs=1e-6)

    got = yaw_from_wxyz([float(v) for v in out.q])
    want = wrap_pi(math.radians(90.0) - yaw_enu)
    assert wrap_pi(got - want) == pytest.approx(0.0, abs=1e-5)


def test_ninety_degree_enu_yaw_is_minus_ninety_in_ned():
    """A +90 deg yaw about ENU z rotates the NED attitude by -90 deg."""
    base = odometry_to_vehicle_odometry(
        make_odometry(orientation_wxyz=enu_yaw_quat(0.0)))
    turned = odometry_to_vehicle_odometry(
        make_odometry(orientation_wxyz=enu_yaw_quat(math.radians(90.0))))
    base_yaw = yaw_from_wxyz([float(v) for v in base.q])
    turned_yaw = yaw_from_wxyz([float(v) for v in turned.q])
    delta = wrap_pi(turned_yaw - base_yaw)
    assert math.degrees(delta) == pytest.approx(-90.0, abs=1e-4)
    # ...and the absolute NED yaw lands on 0 (body pointing North).
    assert math.degrees(turned_yaw) == pytest.approx(0.0, abs=1e-4)


# --------------------------------------------------------------------------
# velocity
# --------------------------------------------------------------------------
def test_twist_flu_to_body_frd():
    msg = make_odometry(linear=(1.0, 2.0, 3.0), angular=(4.0, 5.0, 6.0))
    out = odometry_to_vehicle_odometry(msg)
    assert list(out.velocity) == pytest.approx([1.0, -2.0, -3.0])
    assert list(out.angular_velocity) == pytest.approx([4.0, -5.0, -6.0])
    assert out.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD == 3


def test_velocity_disabled_publishes_nan_and_unknown_frame():
    msg = make_odometry(linear=(1.0, 2.0, 3.0))
    out = odometry_to_vehicle_odometry(msg, publish_velocity=False)
    assert out.velocity_frame == VehicleOdometry.VELOCITY_FRAME_UNKNOWN
    assert all(math.isnan(float(v)) for v in out.velocity)
    assert all(math.isnan(float(v)) for v in out.angular_velocity)
    assert all(math.isnan(float(v)) for v in out.velocity_variance)


# --------------------------------------------------------------------------
# covariance (bug #4)
# --------------------------------------------------------------------------
def test_covariance_permutation():
    """Only position takes the ENU->NED x/y swap; body quantities do not."""
    cov = [float(i) for i in range(36)]
    msg = make_odometry(pose_cov=cov, twist_cov=cov)
    out = odometry_to_vehicle_odometry(msg)
    pc = tc = cov
    assert list(out.position_variance) == pytest.approx(
        [pc[7], pc[0], pc[14]])
    assert list(out.orientation_variance) == pytest.approx(
        [pc[21], pc[28], pc[35]])
    assert list(out.velocity_variance) == pytest.approx(
        [tc[0], tc[7], tc[14]])
    # Explicit values, so a re-permutation cannot pass by accident.
    assert list(out.position_variance) == pytest.approx([7.0, 0.0, 14.0])
    assert list(out.orientation_variance) == pytest.approx(
        [21.0, 28.0, 35.0])
    assert list(out.velocity_variance) == pytest.approx([0.0, 7.0, 14.0])


def test_is_identity_covariance():
    identity = [0.0] * 36
    for i in (0, 7, 14, 21, 28, 35):
        identity[i] = 1.0
    assert is_identity_covariance(identity)
    identity[14] = 0.02
    assert not is_identity_covariance(identity)
    assert not is_identity_covariance([float(i) for i in range(36)])


def test_min_position_variance_clamps_from_below():
    cov = [0.0] * 36
    cov[0], cov[7], cov[14] = 0.001, 0.25, 0.004
    msg = make_odometry(pose_cov=cov)
    out = odometry_to_vehicle_odometry(msg, min_position_variance=0.01)
    # position_variance is [pc[7], pc[0], pc[14]] = [0.25, 0.001, 0.004]
    assert list(out.position_variance) == pytest.approx([0.25, 0.01, 0.01])
    off = odometry_to_vehicle_odometry(msg, min_position_variance=0.0)
    assert list(off.position_variance) == pytest.approx(
        [0.25, 0.001, 0.004], rel=1e-6)


# --------------------------------------------------------------------------
# misc scalar fields
# --------------------------------------------------------------------------
def test_timestamps_and_reset_counter_passthrough():
    msg = make_odometry(stamp=(12, 345_678_000))
    out = odometry_to_vehicle_odometry(msg, reset_counter=7)
    assert out.timestamp_sample == 12_345_678
    assert out.timestamp == 12_345_678  # no clock supplied
    assert out.reset_counter == 7
    assert out.quality == 0
    out2 = odometry_to_vehicle_odometry(msg, timestamp_us=99)
    assert out2.timestamp == 99
    assert out2.timestamp_sample == 12_345_678
    # uint8 wrap
    assert odometry_to_vehicle_odometry(
        msg, reset_counter=257).reset_counter == 1


# --------------------------------------------------------------------------
# reset counter state machine
# --------------------------------------------------------------------------
def test_reset_counter_class_transitions():
    counter = ResetCounter()
    assert counter.update(VO_STATE_SUCCESS) is False   # first sample
    assert counter.value == 0
    assert counter.update(VO_STATE_SUCCESS) is False   # tracking -> tracking
    assert counter.value == 0
    assert counter.update(VO_STATE_FAILED) is False    # lost
    assert counter.value == 0
    assert counter.update(VO_STATE_FAILED) is False    # still lost
    assert counter.value == 0
    assert counter.update(VO_STATE_SUCCESS) is True    # re-acquired
    assert counter.value == 1
    assert counter.update(VO_STATE_SUCCESS) is False
    assert counter.value == 1
    assert counter.update(VO_STATE_UNKNOWN) is False
    assert counter.update(VO_STATE_SUCCESS) is True    # unknown counts too
    assert counter.value == 2


def test_reset_counter_wraps_at_256():
    counter = ResetCounter()
    counter.update(VO_STATE_SUCCESS)
    for _ in range(256):
        counter.update(VO_STATE_FAILED)
        counter.update(VO_STATE_SUCCESS)
    assert counter.value == 0


# --------------------------------------------------------------------------
# node-level (constructed, never spun)
# --------------------------------------------------------------------------
@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = PX4OdomBridge()
    n._test_logger = RecordingLogger()
    n.get_logger = lambda: n._test_logger
    yield n
    n.destroy_node()


def test_node_defaults_to_frd(node):
    assert node.pose_frame == VehicleOdometry.POSE_FRAME_FRD
    assert node.min_position_variance == 0.0
    assert node.status_sub is not None


def test_node_pose_frame_override(ros):
    n = PX4OdomBridge(parameter_overrides=[
        Parameter('pose_frame', Parameter.Type.STRING, 'NED')])
    try:
        assert n.pose_frame == VehicleOdometry.POSE_FRAME_NED
    finally:
        n.destroy_node()


def test_node_status_callback_increments_once(node):
    status = VisualSlamStatus()
    for state in (VO_STATE_SUCCESS, VO_STATE_SUCCESS, VO_STATE_FAILED,
                  VO_STATE_FAILED, VO_STATE_SUCCESS, VO_STATE_SUCCESS):
        status.vo_state = state
        node.status_cb(status)
    assert node.reset_counter.value == 1
    assert len(node._test_logger.records['info']) == 1

    # ...and the published message carries it.
    published = []
    node.pub.publish = published.append
    node.cb(make_odometry())
    assert published[0].reset_counter == 1


def test_node_warns_once_on_sustained_identity_covariance(node):
    """Identity covariance only warns after it persists past the grace window.

    cuVSLAM's PoseCache returns the 1.0 identity fallback for the first frame
    or two after the tracker starts, before it holds two poses to differentiate.
    Measured on the bench: 0 of 1313 steady-state messages were identity, but
    the run still logged the warning at startup. Warning on first sight is a
    false alarm on every single run, so the node requires a full second of
    consecutive identity before it complains.
    """
    identity = [0.0] * 36
    for i in (0, 7, 14, 21, 28, 35):
        identity[i] = 1.0
    real = [0.01] * 36
    node.pub.publish = lambda _msg: None
    grace = node.identity_covariance_grace

    # A real covariance keeps it quiet.
    node.cb(make_odometry(pose_cov=real))
    assert node.identity_covariance_warned is False
    assert node._test_logger.records['warn'] == []

    # The startup transient — a few identity frames, then real ones — must not
    # warn, and must reset the streak.
    for _ in range(grace - 1):
        node.cb(make_odometry(pose_cov=identity))
    assert node.identity_covariance_warned is False
    assert node._test_logger.records['warn'] == []
    node.cb(make_odometry(pose_cov=real))
    assert node.identity_covariance_streak == 0

    # Sustained identity past the grace window warns exactly once.
    for _ in range(grace * 3):
        node.cb(make_odometry(pose_cov=identity))
    assert node.identity_covariance_warned is True
    warnings = node._test_logger.records['warn']
    assert len(warnings) == 1
    assert 'EKF2_EV_NOISE_MD' in warnings[0]


def test_node_cb_publishes_converted_message(node):
    published = []
    node.pub.publish = published.append
    node.cb(make_odometry(position=(1.0, 2.0, 3.0),
                          linear=(1.0, 2.0, 3.0)))
    assert len(published) == 1
    out = published[0]
    assert out.pose_frame == VehicleOdometry.POSE_FRAME_FRD
    assert list(out.position) == pytest.approx([2.0, 1.0, -3.0])
    assert list(out.velocity) == pytest.approx([1.0, -2.0, -3.0])
    # timestamp comes from the node clock, sample from the header
    assert out.timestamp_sample == 12_345_678
    assert out.timestamp != out.timestamp_sample
