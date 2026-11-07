"""Node-level tests. The node is constructed but never spun.

Timers are driven by calling ``tick()`` / ``publish_state()`` directly and
publishers are stubbed, so these run without a DDS peer.  The live end-to-end
check is ``test/integration_offboard.py``.
"""
import json
import math

import pytest

import rclpy
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from px4_msgs.msg import VehicleCommand, VehicleLandDetected, VehicleLocalPosition, VehicleStatus

from skybeta_flight_interface.flight_interface import FlightInterface

NAN = float('nan')


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def make_pose(x, y, z, yaw=None):
    msg = PoseStamped()
    msg.header.frame_id = 'odom'
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    if yaw is None:
        # All-zero orientation == "yaw not commanded".
        msg.pose.orientation.w = 0.0
    else:
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return msg


def make_twist(vx, vy, vz, yaw_rate=0.0):
    msg = TwistStamped()
    msg.header.frame_id = 'odom'
    msg.twist.linear.x = float(vx)
    msg.twist.linear.y = float(vy)
    msg.twist.linear.z = float(vz)
    msg.twist.angular.z = float(yaw_rate)
    return msg


def make_odometry(x, y, z):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation.w = 1.0
    return msg


def make_status(nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                arming_state=VehicleStatus.ARMING_STATE_ARMED):
    msg = VehicleStatus()
    msg.nav_state = nav_state
    msg.arming_state = arming_state
    return msg


def make_local_position(x, y, z, valid=True):
    msg = VehicleLocalPosition()
    msg.x, msg.y, msg.z = float(x), float(y), float(z)
    msg.xy_valid = bool(valid)
    msg.z_valid = bool(valid)
    return msg


class Recorder:
    """Collects everything published, in order."""

    def __init__(self):
        self.messages = []

    def __call__(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = FlightInterface()
    n.offboard = Recorder()
    n.setpoints = Recorder()
    n.commands = Recorder()
    n.states = Recorder()
    n.offboard_pub.publish = n.offboard
    n.setpoint_pub.publish = n.setpoints
    n.command_pub.publish = n.commands
    n.state_pub.publish = n.states
    yield n
    n.destroy_node()


# --------------------------------------------------------------------------
# defaults
# --------------------------------------------------------------------------
def test_defaults(node):
    assert node.setpoint_rate_hz == 10.0
    assert node.allow_mode_request is False
    assert node.anchor_setpoints is False
    assert not node.has_parameter('geofence_radius_m')
    assert node.envelope.max_altitude_m == 3.0
    assert node.mode == 'hold'


def test_no_arm_publisher_exists(node):
    """There must be no path to arming in this node at all."""
    names = [name for name in dir(node) if 'arm' in name.lower()]
    assert names == ['armed']            # the read-only status property
    # rclpy adds six parameter services to every node; ours are the only two
    # under /skybeta, and neither of them arms.
    services = sorted(srv.srv_name for srv in node.services
                      if srv.srv_name.startswith('/skybeta/'))
    assert services == ['/skybeta/hold', '/skybeta/request_offboard']


# --------------------------------------------------------------------------
# heartbeat
# --------------------------------------------------------------------------
def test_tick_publishes_control_mode_and_setpoint_in_that_order(node):
    node.tick()
    assert len(node.offboard.messages) == 1
    assert len(node.setpoints.messages) == 1
    control = node.offboard.last
    assert control.timestamp == node.setpoints.last.timestamp
    assert control.acceleration is False
    assert control.attitude is False
    assert control.body_rate is False


def test_heartbeat_runs_before_any_setpoint_or_px4_data(node):
    """With nothing known, hold == zero velocity, which is a legal stream."""
    for _ in range(5):
        node.tick()
    assert len(node.offboard.messages) == 5
    control = node.offboard.last
    assert control.velocity is True
    assert control.position is False
    assert list(node.setpoints.last.velocity) == pytest.approx([0.0, 0.0, 0.0])
    assert all(math.isnan(v) for v in node.setpoints.last.position)


def test_hold_uses_px4_local_position_when_available(node):
    node.local_position_cb(make_local_position(3.0, 4.0, -5.0))
    node.tick()
    assert node.offboard.last.position is True
    assert node.offboard.last.velocity is False
    assert list(node.setpoints.last.position) == pytest.approx(
        [3.0, 4.0, -5.0])


def test_invalid_local_position_is_not_used_as_a_hold_point(node):
    node.local_position_cb(make_local_position(3.0, 4.0, -5.0, valid=False))
    node.tick()
    assert node.offboard.last.velocity is True
    assert list(node.setpoints.last.velocity) == pytest.approx([0.0, 0.0, 0.0])


# --------------------------------------------------------------------------
# pose setpoints
# --------------------------------------------------------------------------
def test_pose_setpoint_is_converted_to_ned(node):
    node.pose_setpoint_cb(make_pose(1.0, 2.0, 3.0))
    node.tick()
    assert node.mode == 'position'
    assert node.offboard.last.position is True
    assert node.offboard.last.velocity is False
    # ENU (1, 2, 3) -> NED (2, 1, -3)
    assert list(node.setpoints.last.position) == pytest.approx([2.0, 1.0, -3.0])
    assert math.isnan(node.setpoints.last.yaw)


def test_pose_setpoint_yaw_is_ninety_minus_enu_yaw(node):
    node.pose_setpoint_cb(make_pose(0.0, 0.0, 1.5, yaw=math.radians(90.0)))
    node.tick()
    # +90 deg ENU points North, which is NED yaw 0.
    assert node.setpoints.last.yaw == pytest.approx(0.0, abs=1e-6)

    node.pose_setpoint_cb(make_pose(0.0, 0.0, 1.5, yaw=0.0))
    node.tick()
    assert math.degrees(node.setpoints.last.yaw) == pytest.approx(
        90.0, abs=1e-4)


def test_far_pose_passes_through_there_is_no_geofence(node):
    node.odometry_cb(make_odometry(0.0, 0.0, 0.0))
    node.pose_setpoint_cb(make_pose(18.0, 24.0, 1.5))   # 30 m out
    node.tick()
    assert node.mode == 'position'
    assert node.last_clamp_reasons == ()
    # ENU (18, 24, 1.5) -> NED (24, 18, -1.5), untouched
    assert list(node.setpoints.last.position) == pytest.approx(
        [24.0, 18.0, -1.5])


def test_pose_above_the_ceiling_is_clamped(node):
    node.odometry_cb(make_odometry(0.0, 0.0, 0.0))
    node.pose_setpoint_cb(make_pose(0.0, 0.0, 50.0))
    node.tick()
    assert 'max_altitude' in node.last_clamp_reasons
    assert node.setpoints.last.position[2] == pytest.approx(-3.0)


def test_altitude_band_is_relative_to_the_first_odometry_sample(node):
    node.odometry_cb(make_odometry(100.0, 100.0, 5.0))
    assert node.odom_origin_enu == (100.0, 100.0, 5.0)
    node.odometry_cb(make_odometry(101.0, 100.0, 6.0))       # origin sticks
    assert node.odom_origin_enu == (100.0, 100.0, 5.0)
    node.pose_setpoint_cb(make_pose(105.0, 100.0, 6.5))      # 1.5 m up
    assert node.last_clamp_reasons == ()
    node.pose_setpoint_cb(make_pose(105.0, 100.0, 9.0))      # 4 m up
    assert node.last_clamp_reasons == ('max_altitude',)


def test_non_finite_pose_is_ignored(node):
    node.pose_setpoint_cb(make_pose(1.0, 2.0, 3.0))
    node.pose_setpoint_cb(make_pose(float('nan'), 0.0, 1.0))
    node.tick()
    assert list(node.setpoints.last.position) == pytest.approx([2.0, 1.0, -3.0])


def test_pose_setpoint_does_not_expire(node):
    """A position setpoint is already a hold, so staleness is not unsafe."""
    node.pose_setpoint_cb(make_pose(1.0, 2.0, 3.0))
    node.last_setpoint_time = node.now_s() - 3600.0
    node.tick()
    assert node.mode == 'position'
    assert list(node.setpoints.last.position) == pytest.approx([2.0, 1.0, -3.0])


# --------------------------------------------------------------------------
# twist setpoints
# --------------------------------------------------------------------------
def test_twist_setpoint_uses_the_world_permutation(node):
    node.twist_setpoint_cb(make_twist(1.0, 0.5, -0.25, yaw_rate=0.4))
    node.tick()
    assert node.mode == 'velocity'
    assert node.offboard.last.velocity is True
    assert node.offboard.last.position is False
    # TrajectorySetpoint.velocity is NED *world*: (vy, vx, -vz).
    assert list(node.setpoints.last.velocity) == pytest.approx(
        [0.5, 1.0, 0.25])
    # yaw rate about ENU up -> about NED down: negate.
    assert node.setpoints.last.yawspeed == pytest.approx(-0.4)
    assert all(math.isnan(v) for v in node.setpoints.last.position)


def test_twist_over_the_speed_cap_is_scaled(node):
    node.twist_setpoint_cb(make_twist(3.0, 4.0, 0.0))    # 5 m/s
    node.tick()
    assert node.last_clamp_reasons == ('max_speed',)
    speed = math.sqrt(sum(v * v for v in node.setpoints.last.velocity))
    assert speed == pytest.approx(1.5)


def test_stale_twist_expires_into_a_hold(node):
    node.pose_setpoint_cb(make_pose(1.0, 2.0, 3.0))      # establishes a hold
    node.twist_setpoint_cb(make_twist(1.0, 0.0, 0.0))
    node.tick()
    assert node.mode == 'velocity'
    node.last_setpoint_time = node.now_s() - 5.0
    node.tick()
    assert node.mode == 'hold'
    assert node.offboard.last.position is True
    # Falls back to the last position it considered safe, not to zero.
    assert list(node.setpoints.last.position) == pytest.approx([2.0, 1.0, -3.0])


def test_expired_twist_without_a_prior_hold_commands_zero_velocity(node):
    node.twist_setpoint_cb(make_twist(1.0, 0.0, 0.0))
    node.last_setpoint_time = node.now_s() - 5.0
    node.tick()
    assert node.mode == 'hold'
    assert list(node.setpoints.last.velocity) == pytest.approx([0.0, 0.0, 0.0])


def test_non_finite_twist_is_ignored(node):
    node.twist_setpoint_cb(make_twist(1.0, 0.0, 0.0))
    node.twist_setpoint_cb(make_twist(float('inf'), 0.0, 0.0))
    node.tick()
    assert list(node.setpoints.last.velocity) == pytest.approx([0.0, 1.0, 0.0])


# --------------------------------------------------------------------------
# services
# --------------------------------------------------------------------------
def test_hold_freezes_the_last_safe_position(node):
    node.pose_setpoint_cb(make_pose(1.0, 2.0, 3.0))
    node.twist_setpoint_cb(make_twist(2.0, 0.0, 0.0))
    response = node.hold_cb(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert node.mode == 'hold'
    assert node.frozen is True
    node.tick()
    assert node.offboard.last.position is True
    assert list(node.setpoints.last.position) == pytest.approx([2.0, 1.0, -3.0])


def test_px4_link_goes_false_when_status_stops(node):
    assert node.px4_link is False                  # never heard from PX4
    node.status_cb(make_status())
    assert node.px4_link is True
    node.last_status_time -= 10.0                  # silent for 10 s
    assert node.px4_link is False
    node.publish_state()
    assert json.loads(node.states.last.data)['px4_link'] is False


def auto_node():
    n = FlightInterface(parameter_overrides=[
        Parameter('auto_offboard_on_arm', Parameter.Type.BOOL, True)])
    n.commands = Recorder()
    n.offboard_pub.publish = Recorder()
    n.setpoint_pub.publish = Recorder()
    n.command_pub.publish = n.commands
    return n


def make_land_detected(landed):
    msg = VehicleLandDetected()
    msg.landed = landed
    return msg


def arm_then_position(n, order='arm_first', setpoint=True, landed=True):
    """The pilot's sequence. 'arm_first' is the go signal, 'position_first' is not."""
    position = VehicleStatus.NAVIGATION_STATE_POSCTL
    altitude = VehicleStatus.NAVIGATION_STATE_ALTCTL
    armed = VehicleStatus.ARMING_STATE_ARMED
    first = altitude if order == 'arm_first' else position
    n.status_cb(make_status(first, VehicleStatus.ARMING_STATE_DISARMED))
    n.local_position_cb(make_local_position(0.0, 0.0, 0.0))
    if landed is not None:
        n.land_detected_cb(make_land_detected(landed))
    n.odometry_cb(make_odometry(0.0, 0.0, 0.0))
    n.tick()
    if setpoint:
        n.pose_setpoint_cb(make_pose(0.0, 0.0, 1.0))
    n.status_cb(make_status(first, armed))
    n.tick()
    n.status_cb(make_status(position, armed))
    n.tick()


def test_arming_then_position_sends_do_set_mode(ros):
    n = auto_node()
    try:
        arm_then_position(n)
        assert len(n.commands.messages) == 1
        command = n.commands.messages[0]
        assert command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        assert command.param2 == 6.0
    finally:
        n.destroy_node()


def test_wrong_order_in_the_air_or_without_team_code_stays_out(ros):
    for kw in ({'order': 'position_first'}, {'setpoint': False},
               {'landed': False}, {'landed': None}):
        n = auto_node()
        try:
            arm_then_position(n, **kw)
            assert n.commands.messages == []
        finally:
            n.destroy_node()


def test_auto_offboard_is_off_unless_asked_for(node):
    arm_then_position(node)
    assert node.commands.messages == []


def test_no_arm_command_is_ever_sent(ros):
    n = auto_node()
    try:
        arm_then_position(n)
        for _ in range(30):
            n.tick()
        assert all(c.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                   for c in n.commands.messages)
    finally:
        n.destroy_node()


def test_request_offboard_is_refused_by_default(node):
    response = node.request_offboard_cb(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert 'allow_mode_request' in response.message
    assert node.commands.messages == []      # nothing reached PX4


def test_request_offboard_sends_do_set_mode_when_enabled(ros):
    n = FlightInterface(parameter_overrides=[
        Parameter('allow_mode_request', Parameter.Type.BOOL, True)])
    try:
        commands = Recorder()
        n.command_pub.publish = commands
        response = n.request_offboard_cb(Trigger.Request(), Trigger.Response())
        assert response.success is True
        command = commands.last
        assert command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE == 176
        assert command.param1 == pytest.approx(1.0)
        assert command.param2 == pytest.approx(6.0)
        assert command.from_external is True
        assert command.target_system == 1
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------
# /skybeta/state
# --------------------------------------------------------------------------
def test_state_reports_link_down_before_any_px4_message(node):
    node.publish_state()
    state = json.loads(node.states.last.data)
    assert state['px4_link'] is False
    assert state['armed'] is False
    assert state['offboard_active'] is False
    assert state['nav_state'] == -1
    assert state['odom_enu'] is None


def test_state_reports_offboard_and_armed(node):
    node.status_cb(make_status())
    node.local_position_cb(make_local_position(2.0, 1.0, -3.0))
    node.odometry_cb(make_odometry(1.0, 2.0, 3.0))   # origin
    node.odometry_cb(make_odometry(1.0, 2.0, 4.5))   # 1.5 m above it
    node.publish_state()
    state = json.loads(node.states.last.data)
    assert state['px4_link'] is True
    assert state['armed'] is True
    assert state['offboard_active'] is True
    assert state['nav_state_name'] == 'OFFBOARD'
    assert state['odom_enu'] == pytest.approx([1.0, 2.0, 4.5])
    # PX4 NED (2, 1, -3) reported back in ENU as (1, 2, 3).
    assert state['local_enu'] == pytest.approx([1.0, 2.0, 3.0])
    assert state['envelope_ok'] is True
    assert 'geofence_ok' not in state
    assert state['setpoint_mode'] == 'hold'


def test_state_reports_offboard_false_in_another_mode(node):
    node.status_cb(make_status(
        nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL))
    node.publish_state()
    state = json.loads(node.states.last.data)
    assert state['offboard_active'] is False
    assert state['nav_state_name'] == 'POSCTL'


def test_state_envelope_ok_stays_true_far_out(node):
    node.odometry_cb(make_odometry(0.0, 0.0, 1.0))     # origin
    node.odometry_cb(make_odometry(30.0, 0.0, 2.0))    # 30 m out: no geofence
    node.publish_state()
    state = json.loads(node.states.last.data)
    assert state['envelope_ok'] is True
    assert 'geofence_radius_m' not in state['envelope']


def test_state_envelope_ok_is_true_sitting_on_the_ground(node):
    """A vehicle below the altitude floor has not breached anything -- the
    floor is a setpoint clamp, not a vehicle-state violation."""
    node.odometry_cb(make_odometry(0.0, 0.0, 0.0))     # origin, on the ground
    node.publish_state()
    assert json.loads(node.states.last.data)['envelope_ok'] is True


def test_state_envelope_ok_goes_false_above_the_ceiling(node):
    node.odometry_cb(make_odometry(0.0, 0.0, 0.0))
    node.odometry_cb(make_odometry(0.0, 0.0, 9.0))
    node.publish_state()
    assert json.loads(node.states.last.data)['envelope_ok'] is False


def test_state_reports_the_clamp_reasons(node):
    node.odometry_cb(make_odometry(0.0, 0.0, 0.0))
    node.pose_setpoint_cb(make_pose(100.0, 0.0, 100.0))
    node.publish_state()
    state = json.loads(node.states.last.data)
    assert state['clamped'] == ['max_altitude']
    assert state['setpoint_mode'] == 'position'


def test_state_is_valid_json_every_time(node):
    node.status_cb(make_status())
    for _ in range(3):
        node.publish_state()
    for message in node.states.messages:
        json.loads(message.data)


# --------------------------------------------------------------------------
# anchoring (off by default)
# --------------------------------------------------------------------------
def test_anchoring_offsets_by_both_origins(ros):
    n = FlightInterface(parameter_overrides=[
        Parameter('anchor_setpoints', Parameter.Type.BOOL, True)])
    try:
        setpoints = Recorder()
        n.setpoint_pub.publish = setpoints
        n.offboard_pub.publish = Recorder()
        n.local_position_cb(make_local_position(10.0, 20.0, -5.0))
        n.odometry_cb(make_odometry(3.0, 5.0, 0.0))
        assert n.px4_origin_ned == (10.0, 20.0, -5.0)
        n.pose_setpoint_cb(make_pose(4.0, 7.0, 1.5))
        n.tick()
        # delta ENU (1, 2, 1.5) -> NED (2, 1, -1.5), + origin (10, 20, -5)
        assert list(setpoints.last.position) == pytest.approx(
            [12.0, 21.0, -6.5])
    finally:
        n.destroy_node()


def test_topics_resolved_against_the_installed_px4_msgs(node):
    assert node.topics['offboard_control_mode'] == (
        '/fmu/in/offboard_control_mode')
    assert node.topics['trajectory_setpoint'] == '/fmu/in/trajectory_setpoint'
    # vehicle_status carries a _v<N> suffix on px4_msgs release/1.16 and none
    # on release/1.14 -- either spelling is correct for its own image.
    assert node.topics['vehicle_status'] in (
        '/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1')
