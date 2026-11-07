"""skybeta flight interface -- the only thing that talks to PX4 offboard.

Competitors publish ENU setpoints on ``/skybeta/setpoint/pose`` and
``/skybeta/setpoint/twist`` and read ``/skybeta/state``.  This node owns
everything between those topics and PX4:

  * the **10 Hz OffboardControlMode heartbeat**.  PX4 refuses to *enter*
    Offboard without a stream already running and drops *out* of it if the
    stream falls below ~2 Hz (``COM_OF_LOSS_T``, then ``COM_OBL_RC_ACT``), so
    the heartbeat runs from node start, before any competitor setpoint
    exists, and never stops.
  * the ENU/FLU -> NED/FRD conversion (``frames.py``).
  * the safety envelope (``envelope.py``): altitude band and speed cap,
    applied as a **clamp**, never a rejection.  There is no horizontal
    geofence.
  * ``/fmu`` topic-name resolution against the installed px4_msgs, so the
    same source works on the 1.14 and 1.16 message generations
    (``topics.py``).

What it deliberately does NOT do
--------------------------------
**It never arms.**  There is no arm service at all: the safety pilot on RC arms
the vehicle.  With ``auto_offboard_on_arm`` set, the pilot's switch to Position
after arming, on the ground, is the go signal: it is followed by one request for
Offboard (``auto_offboard.py`` has the rules).  The pilot takes the vehicle back with the mode switch or the
sticks (``COM_RC_OVERRIDE`` = 3), and nothing here requests Offboard again until
it has been disarmed.  The optional ``/skybeta/request_offboard`` service sends
the same ``VEHICLE_CMD_DO_SET_MODE(1, 6)`` on demand, only when the
``allow_mode_request`` parameter is explicitly set true, and it is **false by
default and intended for SITL**.

Setpoint semantics
------------------
A *pose* setpoint is a position hold at a point: it persists until replaced,
because that is what it means.  A *twist* setpoint is a rate command, which
is only safe while someone is actively refreshing it -- so it **expires**
after ``setpoint_timeout_s`` and the node falls back to holding the last
position it considered safe.  A competitor node that dies mid-square leaves
the vehicle hovering, not drifting.

Until a competitor setpoint arrives the node still has to stream something.
It holds PX4's own local position if EKF2 has one, and otherwise commands
zero velocity -- both are "do not move", and both are a legal stream for
entering Offboard.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from skybeta_flight_interface import frames
from skybeta_flight_interface.auto_offboard import AutoOffboard, Inputs
from skybeta_flight_interface.envelope import SafetyEnvelope
from skybeta_flight_interface.topics import px4_msgs_msg_dir, resolve_endpoints

NAN = float('nan')
#: vehicle_status arrives at about 5 Hz, so this is several missed messages.
STATUS_TIMEOUT_S = 2.0

#: PX4 custom main mode for Offboard, the ``param2`` of DO_SET_MODE(1, 6).
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
#: ``param1`` = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
PX4_MODE_FLAG_CUSTOM_ENABLED = 1.0

#: nav_state -> name, for the human reading /skybeta/state. Only the handful
#: a competition flight actually passes through; anything else prints its
#: number, which is enough to look up in VehicleStatus.msg.
NAV_STATE_NAMES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
    4: 'AUTO_LOITER', 5: 'AUTO_RTL', 10: 'ACRO', 12: 'DESCEND',
    13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND', 19: 'AUTO_FOLLOW_TARGET', 20: 'AUTO_PRECLAND',
    21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
}


def _finite_xyz(values):
    return all(math.isfinite(float(v)) for v in values)


class FlightInterface(Node):

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy.Node (parameter_overrides, context, ...) so
        # tests can build the node without spinning it.
        super().__init__('flight_interface', **kwargs)

        # -- parameters ------------------------------------------------
        self.declare_parameter('max_altitude_m', 3.0)
        self.declare_parameter('min_altitude_m', 0.3)
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('state_rate_hz', 5.0)
        self.declare_parameter('setpoint_timeout_s', 1.0)
        self.declare_parameter('allow_mode_request', False)
        self.declare_parameter('auto_offboard_on_arm', False)
        self.declare_parameter('odometry_topic',
                               '/visual_slam/tracking/odometry')
        self.declare_parameter('pose_setpoint_topic', '/skybeta/setpoint/pose')
        self.declare_parameter('twist_setpoint_topic',
                               '/skybeta/setpoint/twist')
        self.declare_parameter('state_topic', '/skybeta/state')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('warn_period_s', 2.0)
        # Anchoring: off by default, so the conversion is exactly the
        # documented ENU->NED rotation. See frames.setpoint_position_enu_to_ned
        # and docs/competitor_guide.md before turning it on.
        self.declare_parameter('anchor_setpoints', False)
        self.declare_parameter('odom_to_ned_yaw_offset_rad', 0.0)

        self.envelope = SafetyEnvelope(
            max_altitude_m=self.get_parameter('max_altitude_m').value,
            min_altitude_m=self.get_parameter('min_altitude_m').value,
            max_speed_mps=self.get_parameter('max_speed_mps').value,
        )
        self.setpoint_rate_hz = float(
            self.get_parameter('setpoint_rate_hz').value)
        self.setpoint_timeout_s = float(
            self.get_parameter('setpoint_timeout_s').value)
        self.allow_mode_request = bool(
            self.get_parameter('allow_mode_request').value)
        self.auto_offboard = AutoOffboard(
            self.get_parameter('auto_offboard_on_arm').value,
            self.setpoint_timeout_s)
        self.target_system = int(self.get_parameter('target_system').value)
        self.warn_period_s = float(self.get_parameter('warn_period_s').value)
        self.anchor_setpoints = bool(
            self.get_parameter('anchor_setpoints').value)
        self.yaw_offset_rad = float(
            self.get_parameter('odom_to_ned_yaw_offset_rad').value)

        # -- state -----------------------------------------------------
        self.mode = 'hold'              # 'hold' | 'position' | 'velocity'
        self.hold_position_ned = None   # last safe NED position setpoint
        self.hold_yaw_ned = NAN
        self.target_position_ned = None
        self.target_yaw_ned = NAN
        self.target_velocity_ned = None
        self.target_yawspeed_ned = NAN
        self.last_setpoint_time = None
        self.last_clamp_reasons = ()
        self.frozen = False             # set by the hold service

        self.odom_origin_enu = None     # captured at first odometry
        self.odom_position_enu = None
        self.px4_origin_ned = None      # captured with the odom origin
        self.local_position = None
        self.landed = None              # None until PX4 reports it
        self.vehicle_status = None
        self.last_status_time = None
        self._last_warn_time = {}
        self._expiry_logged = False

        # -- QoS -------------------------------------------------------
        # /fmu/in/*: exactly what px4_odom_bridge uses and what PX4's
        # uxrce_dds_client subscribers expect.
        px4_pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /fmu/out/*: uxrce_dds_client publishes BEST_EFFORT with
        # TRANSIENT_LOCAL durability (rmw_qos_profile_sensor_data plus
        # durability), per the PX4 v1.16 ROS 2 user guide.
        px4_sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.topics = resolve_endpoints()
        msg_dir = px4_msgs_msg_dir()

        # -- PX4 interface ---------------------------------------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, self.topics['offboard_control_mode'],
            px4_pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self.topics['trajectory_setpoint'],
            px4_pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, self.topics['vehicle_command'], px4_pub_qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, self.topics['vehicle_status'],
            self.status_cb, px4_sub_qos)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, self.topics['vehicle_local_position'],
            self.local_position_cb, px4_sub_qos)
        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected, self.topics['vehicle_land_detected'],
            self.land_detected_cb, px4_sub_qos)

        # -- competitor interface --------------------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_setpoint_topic').value,
            self.pose_setpoint_cb, 10)
        self.twist_sub = self.create_subscription(
            TwistStamped, self.get_parameter('twist_setpoint_topic').value,
            self.twist_setpoint_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self.odometry_cb, 10)
        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)

        self.hold_srv = self.create_service(
            Trigger, '/skybeta/hold', self.hold_cb)
        self.offboard_srv = self.create_service(
            Trigger, '/skybeta/request_offboard', self.request_offboard_cb)

        # -- timers ----------------------------------------------------
        self.setpoint_timer = self.create_timer(
            1.0 / self.setpoint_rate_hz, self.tick)
        state_rate = float(self.get_parameter('state_rate_hz').value)
        self.state_timer = self.create_timer(
            1.0 / state_rate, self.publish_state)

        topic_map = '\n    '.join(
            f'{key:24s} {value}' for key, value in sorted(self.topics.items()))
        self.get_logger().info(
            f'skybeta flight interface up @ {self.setpoint_rate_hz:.1f} Hz\n'
            f'  envelope: {self.envelope.describe()}\n'
            f'  px4_msgs msg dir: {msg_dir or "NOT FOUND"}\n'
            f'  resolved topics:\n    {topic_map}\n'
            f'  mode requests: '
            f'{"ENABLED (sim only!)" if self.allow_mode_request else "disabled"}'
            f'\n  Offboard on the switch to Position after arming: '
            f'{"ON" if self.auto_offboard.enabled else "off"}'
        )
        self.get_logger().info(
            'This node never arms: the safety pilot does, on RC.')

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def warn_throttled(self, key, message):
        """One WARN per ``warn_period_s`` per key -- a clamped setpoint at
        10 Hz would otherwise bury every other line in the log."""
        now = self.now_s()
        last = self._last_warn_time.get(key)
        if last is not None and (now - last) < self.warn_period_s:
            return
        self._last_warn_time[key] = now
        self.get_logger().warn(message)

    @property
    def px4_link(self):
        """vehicle_status is still arriving, not merely that it once did."""
        return (self.last_status_time is not None
                and self.now_s() - self.last_status_time <= STATUS_TIMEOUT_S)

    @property
    def offboard_active(self):
        status = self.vehicle_status
        return bool(status is not None
                    and status.nav_state
                    == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    @property
    def armed(self):
        status = self.vehicle_status
        return bool(status is not None
                    and status.arming_state
                    == VehicleStatus.ARMING_STATE_ARMED)

    def _anchors(self):
        """(odom_origin_enu, px4_origin_ned) or (None, None) when disabled."""
        if not self.anchor_setpoints:
            return None, None
        return self.odom_origin_enu, self.px4_origin_ned

    # ------------------------------------------------------------------
    # PX4 -> us
    # ------------------------------------------------------------------
    def status_cb(self, msg):
        previous = self.vehicle_status
        self.vehicle_status = msg
        self.last_status_time = self.now_s()
        if previous is None:
            self.get_logger().info(
                f'PX4 link up: nav_state={msg.nav_state} '
                f'({NAV_STATE_NAMES.get(msg.nav_state, msg.nav_state)}), '
                f'armed={self.armed}')
            return
        if previous.nav_state != msg.nav_state:
            self.get_logger().info(
                f'nav_state '
                f'{NAV_STATE_NAMES.get(previous.nav_state, previous.nav_state)}'
                f' -> '
                f'{NAV_STATE_NAMES.get(msg.nav_state, msg.nav_state)}')

    def local_position_cb(self, msg):
        self.local_position = msg

    def land_detected_cb(self, msg):
        self.landed = bool(msg.landed)

    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        position = (float(p.x), float(p.y), float(p.z))
        if not _finite_xyz(position):
            self.warn_throttled('odom_nan',
                                'odometry position is not finite - ignoring')
            return
        self.odom_position_enu = position
        if self.odom_origin_enu is None:
            self.odom_origin_enu = position
            # Capture PX4's local position at the same instant, so an
            # anchored setpoint has a matched pair of origins.
            local = self.local_position
            if local is not None and local.xy_valid and local.z_valid:
                self.px4_origin_ned = (float(local.x), float(local.y),
                                       float(local.z))
            self.get_logger().info(
                f'envelope origin captured at ENU '
                f'({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}); '
                f'{self.envelope.describe()}')
            if self.anchor_setpoints and self.px4_origin_ned is None:
                self.get_logger().warn(
                    'anchor_setpoints is true but EKF2 had no valid local '
                    'position when odometry started - setpoints will be '
                    'anchored at the NED origin instead.')

    # ------------------------------------------------------------------
    # competitor -> us
    # ------------------------------------------------------------------
    def pose_setpoint_cb(self, msg):
        p = msg.pose.position
        requested = (float(p.x), float(p.y), float(p.z))
        if not _finite_xyz(requested):
            self.warn_throttled(
                'pose_nan',
                'pose setpoint contains NaN/inf - ignored, still holding')
            return

        origin = self.odom_origin_enu or (0.0, 0.0, 0.0)
        if self.odom_origin_enu is None:
            self.warn_throttled(
                'no_origin',
                'no odometry yet, so the altitude band is measured from '
                '(0,0,0) - setpoints are still clamped, just against an '
                'unverified origin.')
        clamp = self.envelope.clamp_position(requested, origin)
        if clamp.clamped:
            self.warn_throttled(
                'pose_clamp',
                f'pose setpoint clamped ({", ".join(clamp.reasons)}): '
                f'({requested[0]:.2f}, {requested[1]:.2f}, {requested[2]:.2f})'
                f' -> ({clamp.value[0]:.2f}, {clamp.value[1]:.2f}, '
                f'{clamp.value[2]:.2f}) ENU')
        self.last_clamp_reasons = clamp.reasons

        odom_origin, px4_origin = self._anchors()
        self.target_position_ned = frames.setpoint_position_enu_to_ned(
            clamp.value, odom_origin, px4_origin, self.yaw_offset_rad)

        q = msg.pose.orientation
        quat = (float(q.x), float(q.y), float(q.z), float(q.w))
        if any(v != 0.0 for v in quat):
            yaw_ned = frames.yaw_ned_from_quat_xyzw(quat)
            self.target_yaw_ned = frames.wrap_pi(yaw_ned + self.yaw_offset_rad)
        else:
            # All-zero quaternion is not a rotation; treat it as "yaw is not
            # being commanded" rather than silently flying to yaw 0.
            self.target_yaw_ned = NAN

        # A position setpoint IS a hold, so it also becomes the safe fallback.
        self.hold_position_ned = self.target_position_ned
        self.hold_yaw_ned = self.target_yaw_ned
        self.target_velocity_ned = None
        self.mode = 'position'
        self.frozen = False
        self.last_setpoint_time = self.now_s()
        self._expiry_logged = False

    def twist_setpoint_cb(self, msg):
        linear = (float(msg.twist.linear.x), float(msg.twist.linear.y),
                  float(msg.twist.linear.z))
        if not _finite_xyz(linear):
            self.warn_throttled(
                'twist_nan',
                'twist setpoint contains NaN/inf - ignored, still holding')
            return
        clamp = self.envelope.clamp_velocity(linear)
        if clamp.clamped:
            speed = math.sqrt(sum(v * v for v in linear))
            self.warn_throttled(
                'twist_clamp',
                f'twist setpoint clamped (max_speed): {speed:.2f} -> '
                f'{self.envelope.max_speed_mps:.2f} m/s')
        self.last_clamp_reasons = clamp.reasons

        # TrajectorySetpoint.velocity is in the NED local *world* frame, so
        # this is the world permutation, not the FLU->FRD body sign flip.
        self.target_velocity_ned = frames.enu_to_ned(clamp.value)
        yawspeed = float(msg.twist.angular.z)
        self.target_yawspeed_ned = (
            frames.yaw_rate_enu_to_ned(yawspeed)
            if math.isfinite(yawspeed) else NAN)
        self.target_position_ned = None
        self.mode = 'velocity'
        self.frozen = False
        self.last_setpoint_time = self.now_s()
        self._expiry_logged = False

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def hold_cb(self, request, response):
        """Freeze at the last position considered safe."""
        del request
        current = self._current_local_ned()
        if current is not None:
            self.hold_position_ned = current
        self.frozen = True
        self.mode = 'hold'
        self.target_velocity_ned = None
        self.target_position_ned = None
        where = self.hold_position_ned or self._current_local_ned()
        response.success = True
        if where is None:
            response.message = (
                'holding: no position known yet, commanding zero velocity')
        else:
            response.message = (
                f'holding at NED ({where[0]:.2f}, {where[1]:.2f}, '
                f'{where[2]:.2f})')
        self.get_logger().warn(f'HOLD requested - {response.message}')
        return response

    def request_offboard_cb(self, request, response):
        """Ask PX4 for Offboard on demand. Off by default; for SITL."""
        del request
        if not self.allow_mode_request:
            response.success = False
            response.message = (
                'refused: allow_mode_request is false. On a real vehicle '
                'Offboard follows the pilot arming, then switching to '
                'Position (auto_offboard_on_arm); this service exists for '
                'SITL only.')
            self.get_logger().warn(response.message)
            return response
        self._send_offboard_command()
        response.success = True
        response.message = (
            'sent VEHICLE_CMD_DO_SET_MODE(1, 6). This does NOT arm; PX4 will '
            'reject it unless the offboard stream is already running.')
        self.get_logger().warn(f'offboard mode requested - {response.message}')
        return response

    def _send_offboard_command(self):
        command = VehicleCommand()
        command.timestamp = self.now_us()
        command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command.param1 = PX4_MODE_FLAG_CUSTOM_ENABLED
        command.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD
        command.target_system = self.target_system
        command.target_component = 1
        command.source_system = self.target_system
        command.source_component = 1
        command.from_external = True
        self.command_pub.publish(command)

    def _step_auto_offboard(self):
        status = self.vehicle_status
        age = None
        if self.last_setpoint_time is not None:
            age = self.now_s() - self.last_setpoint_time
        request, message = self.auto_offboard.step(Inputs(
            now_s=self.now_s(),
            link=self.px4_link,
            armed=self.armed,
            in_position=bool(
                status is not None and status.nav_state
                == VehicleStatus.NAVIGATION_STATE_POSCTL),
            landed=self.landed,
            offboard=self.offboard_active,
            position_valid=self._current_local_ned() is not None,
            setpoint_age_s=age,
            frozen=self.frozen,
        ))
        if message:
            self.get_logger().warn(f'auto offboard: {message}')
        if request:
            self._send_offboard_command()

    # ------------------------------------------------------------------
    # the 10 Hz loop
    # ------------------------------------------------------------------
    def _current_local_ned(self):
        local = self.local_position
        if local is None:
            return None
        if not (local.xy_valid and local.z_valid):
            return None
        position = (float(local.x), float(local.y), float(local.z))
        return position if _finite_xyz(position) else None

    def _expire_stale_velocity(self):
        """A rate command that stopped being refreshed is not safe to keep."""
        if self.mode != 'velocity' or self.last_setpoint_time is None:
            return
        age = self.now_s() - self.last_setpoint_time
        if age < self.setpoint_timeout_s:
            return
        current = self._current_local_ned()
        if current is not None:
            self.hold_position_ned = current
        self.mode = 'hold'
        self.target_velocity_ned = None
        if not self._expiry_logged:
            self._expiry_logged = True
            self.get_logger().warn(
                f'twist setpoint stale ({age:.2f}s > '
                f'{self.setpoint_timeout_s:.2f}s) - holding position. A pose '
                f'setpoint does not expire; a twist setpoint must be '
                f'refreshed.')

    def build_setpoint(self):
        """The TrajectorySetpoint for this tick, plus its control flags."""
        self._expire_stale_velocity()
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.now_us()
        setpoint.position = [NAN, NAN, NAN]
        setpoint.velocity = [NAN, NAN, NAN]
        setpoint.acceleration = [NAN, NAN, NAN]
        setpoint.yaw = NAN
        setpoint.yawspeed = NAN

        if self.mode == 'velocity' and self.target_velocity_ned is not None:
            setpoint.velocity = [float(v) for v in self.target_velocity_ned]
            setpoint.yawspeed = float(self.target_yawspeed_ned)
            return setpoint, {'velocity': True}

        if self.mode == 'position' and self.target_position_ned is not None:
            setpoint.position = [float(v) for v in self.target_position_ned]
            setpoint.yaw = float(self.target_yaw_ned)
            return setpoint, {'position': True}

        # 'hold' -- the last safe position, else PX4's own local position,
        # else a zero-velocity command so the stream is still legal.
        where = self.hold_position_ned or self._current_local_ned()
        if where is not None:
            setpoint.position = [float(v) for v in where]
            setpoint.yaw = float(self.hold_yaw_ned)
            return setpoint, {'position': True}
        setpoint.velocity = [0.0, 0.0, 0.0]
        return setpoint, {'velocity': True}

    def tick(self):
        setpoint, flags = self.build_setpoint()

        control = OffboardControlMode()
        control.timestamp = setpoint.timestamp
        control.position = bool(flags.get('position', False))
        control.velocity = bool(flags.get('velocity', False))
        control.acceleration = False
        control.attitude = False
        control.body_rate = False
        # The heartbeat goes first: PX4 reads the control mode to decide which
        # fields of the setpoint it is allowed to look at.
        self.offboard_pub.publish(control)
        self.setpoint_pub.publish(setpoint)
        self._step_auto_offboard()

    # ------------------------------------------------------------------
    def publish_state(self):
        local_enu = None
        local_ned = self._current_local_ned()
        if local_ned is not None:
            local_enu = list(frames.ned_to_enu(local_ned))

        envelope_ok = True
        if self.odom_position_enu is not None:
            envelope_ok = self.envelope.vehicle_inside(
                self.odom_position_enu, self.odom_origin_enu or (0.0, 0.0, 0.0))

        status = self.vehicle_status
        nav_state = int(status.nav_state) if status is not None else -1
        state = {
            'stamp': round(self.now_s(), 3),
            'px4_link': self.px4_link,
            'armed': self.armed,
            'landed': self.landed,
            'nav_state': nav_state,
            'nav_state_name': NAV_STATE_NAMES.get(nav_state, str(nav_state)),
            'offboard_active': self.offboard_active,
            'odom_enu': (list(self.odom_position_enu)
                         if self.odom_position_enu else None),
            'origin_enu': (list(self.odom_origin_enu)
                           if self.odom_origin_enu else None),
            'local_enu': local_enu,
            'envelope_ok': envelope_ok,
            'setpoint_mode': self.mode,
            'clamped': list(self.last_clamp_reasons),
            'frozen': self.frozen,
            'envelope': {
                'min_altitude_m': self.envelope.min_altitude_m,
                'max_altitude_m': self.envelope.max_altitude_m,
                'max_speed_mps': self.envelope.max_speed_mps,
            },
        }
        message = String()
        message.data = json.dumps(state)
        self.state_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FlightInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
