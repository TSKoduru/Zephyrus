"""THE FILE TO COPY. A worked example of a skybeta competitor node.

Mission: wait until the flight interface has entered Offboard (it does that once,
when the safety pilot arms, then switches to Position), hover at 1.5 m for 5 s, fly a 5 m square,
then hold.  Everything it does is done through two topics:

    subscribe  /skybeta/state                     what PX4 is doing
    subscribe  /visual_slam/tracking/odometry     where we are (ENU, odom)
    publish    /skybeta/setpoint/pose             where we want to be (ENU, odom)

That is the whole contract.  This node never talks to /fmu/*, never arms,
never changes mode, and never worries about NED: ``skybeta_flight_interface``
owns all of that, and it clamps whatever this file asks for to the safety
envelope before PX4 ever sees it.

Frames
------
Everything here is **ENU** in the ``odom`` frame that cuVSLAM publishes:

    +x East      +y North      +z Up

...where "East" and "North" mean *whatever direction the vehicle's x and y
axes pointed when the SLAM node started*, because cuVSLAM has no compass.
The origin is the rig's startup pose, so at power-on the vehicle is at
(0, 0, 0) and z is height above that point.  Do not assume magnetic north.

Yaw is optional.  Leaving the setpoint's orientation quaternion all-zero
means "do not command yaw"; this example does exactly that and lets PX4 hold
whatever heading it has.  Set a real quaternion (or use ``yaw_enu`` below) to
command one.

Safety
------
The safety pilot arms, then switches to Position; the flight interface then enters
Offboard, and this node takes off from the ground.  The pilot can take the
vehicle back at any moment with the mode switch or the sticks.  This node must
therefore **wait** until ``/skybeta/state`` reports ``offboard_active`` before
it starts the mission, and it stops advancing if Offboard is lost.  It does not
attempt to get Offboard back -- that is the pilot's decision.

Tuning knobs are ROS parameters; nothing is hard-coded in the state machine.
"""
import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SquareMission(Node):
    """Hover, fly a square, hold. A four-state machine on a 10 Hz timer."""

    def __init__(self, **kwargs):
        super().__init__('competitor_example', **kwargs)

        # --- parameters ------------------------------------------------
        self.declare_parameter('hover_altitude_m', 1.5)
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('square_side_m', 5.0)
        self.declare_parameter('arrival_radius_m', 0.3)
        self.declare_parameter('publish_rate_hz', 10.0)
        # Give up on a corner rather than hang forever if it cannot be
        # reached (wind, a clamp, or a competitor bug).
        self.declare_parameter('corner_timeout_s', 60.0)

        self.hover_altitude = float(
            self.get_parameter('hover_altitude_m').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        self.side = float(self.get_parameter('square_side_m').value)
        self.arrival_radius = float(
            self.get_parameter('arrival_radius_m').value)
        self.corner_timeout = float(
            self.get_parameter('corner_timeout_s').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # --- the contract ----------------------------------------------
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10)
        self.create_subscription(
            String, '/skybeta/state', self.state_cb, 10)

        # --- mission state ---------------------------------------------
        self.state = 'WAIT_FOR_OFFBOARD'
        self.position = None          # current ENU position from cuVSLAM
        self.start = None             # ENU position when the mission began
        self.corners = []             # ENU waypoints, filled once we know start
        self.corner_index = 0
        self.hover_started_at = None
        self.corner_started_at = None
        self.offboard_active = False
        self.was_offboard = False
        self.log_once = set()

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'competitor_example ready: hover {self.hover_altitude:.2f} m for '
            f'{self.hover_seconds:.0f} s, then a {self.side:.1f} m square '
            f'(arrive within {self.arrival_radius:.2f} m).\n'
            'Waiting for the safety pilot to arm, then switch to Position...')

    # ------------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------------
    def odometry_cb(self, msg):
        """Where we are, in the ENU ``odom`` frame."""
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def state_cb(self, msg):
        """What PX4 is doing, as reported by skybeta_flight_interface."""
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))
        if state.get('clamped'):
            self.info_once(
                f'clamped:{",".join(state["clamped"])}',
                f'the flight interface is CLAMPING our setpoints '
                f'({", ".join(state["clamped"])}) -- we are asking for '
                f'something outside the safety envelope '
                f'{state.get("envelope")}')

    def info_once(self, key, message):
        if key in self.log_once:
            return
        self.log_once.add(key)
        self.get_logger().info(message)

    # ------------------------------------------------------------------
    # outputs
    # ------------------------------------------------------------------
    def send(self, x, y, z, yaw_enu=None):
        """Publish one ENU setpoint in the ``odom`` frame.

        The flight interface clamps this to the safety envelope and converts
        it to PX4's NED ``TrajectorySetpoint``; we never do either ourselves.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        if yaw_enu is None:
            # All-zero quaternion == "do not command yaw".
            msg.pose.orientation.w = 0.0
        else:
            msg.pose.orientation.z = math.sin(float(yaw_enu) / 2.0)
            msg.pose.orientation.w = math.cos(float(yaw_enu) / 2.0)
        self.setpoint_pub.publish(msg)

    # ------------------------------------------------------------------
    # geometry
    # ------------------------------------------------------------------
    def distance_to(self, target):
        if self.position is None:
            return float('inf')
        return math.dist(self.position, target)

    def arrived_at(self, target):
        return self.distance_to(target) <= self.arrival_radius

    def build_square(self):
        """Four corners, counter-clockwise from the hover point.

        There is no horizontal limit in the flight interface, so the square
        is flown at whatever size you ask for: make sure it fits the room.
        Its height is still clamped to the altitude band, and a clamped
        corner is one the vehicle will simply never "arrive" at.
        """
        x0, y0, _ = self.start
        z = self.start[2] + self.hover_altitude
        side = self.side
        return [
            (x0 + side, y0, z),
            (x0 + side, y0 + side, z),
            (x0, y0 + side, z),
            (x0, y0, z),
        ]

    # ------------------------------------------------------------------
    # the 10 Hz state machine
    # ------------------------------------------------------------------
    def tick(self):
        if self.position is None:
            self.info_once(
                'no_odom',
                'no /visual_slam/tracking/odometry yet -- is cuVSLAM up and '
                'tracking? Nothing will be commanded until it is.')
            return

        # The pilot can take the vehicle back at any time. If they do, stop
        # advancing the mission and keep commanding the current waypoint
        # (which PX4 ignores while it is not in Offboard anyway).
        if self.was_offboard and not self.offboard_active:
            self.get_logger().warn(
                'Offboard lost -- the pilot has taken control. Mission '
                'paused. Offboard does not return until the vehicle '
                'is disarmed and armed again.')
            self.was_offboard = False
        if self.offboard_active and not self.was_offboard:
            self.was_offboard = True

        handler = getattr(self, f'_state_{self.state.lower()}')
        handler()

    # -- states --------------------------------------------------------
    def _state_wait_for_offboard(self):
        if not self.offboard_active:
            self.info_once(
                'waiting',
                'holding at the current position until the safety pilot '
                'arms and switches to Position.')
            # Command where we already are, so the moment Offboard starts the
            # vehicle holds instead of jumping to a stale target. This stream is
            # also what lets the flight interface enter Offboard at all.
            self.send(*self.position)
            return
        self.start = self.position
        self.corners = self.build_square()
        self.hover_started_at = self.now_s()
        self.state = 'HOVER'
        self.get_logger().info(
            f'Offboard active. Mission start at ENU '
            f'({self.start[0]:.2f}, {self.start[1]:.2f}, {self.start[2]:.2f}); '
            f'climbing to {self.hover_altitude:.2f} m.')

    def _state_hover(self):
        target = (self.start[0], self.start[1],
                  self.start[2] + self.hover_altitude)
        self.send(*target)
        # The hover clock only starts once we are actually at altitude, so
        # "5 seconds" means 5 seconds of hovering, not 5 seconds of climbing.
        if not self.arrived_at(target):
            self.hover_started_at = self.now_s()
            return
        if self.now_s() - self.hover_started_at < self.hover_seconds:
            return
        self.corner_index = 0
        self.corner_started_at = self.now_s()
        self.state = 'SQUARE'
        self.get_logger().info(
            f'hover complete -- flying the {self.side:.1f} m square, corner '
            f'1/{len(self.corners)}')

    def _state_square(self):
        target = self.corners[self.corner_index]
        self.send(*target)
        if self.arrived_at(target):
            self.corner_index += 1
            self.corner_started_at = self.now_s()
            if self.corner_index >= len(self.corners):
                self.state = 'HOLD'
                self.get_logger().info(
                    'square complete -- holding. The safety pilot can take '
                    'over and land.')
                return
            self.get_logger().info(
                f'corner reached -- next: {self.corner_index + 1}/'
                f'{len(self.corners)} at ENU '
                f'({self.corners[self.corner_index][0]:.2f}, '
                f'{self.corners[self.corner_index][1]:.2f}, '
                f'{self.corners[self.corner_index][2]:.2f})')
            return
        if self.now_s() - self.corner_started_at > self.corner_timeout:
            self.get_logger().error(
                f'corner {self.corner_index + 1} not reached within '
                f'{self.corner_timeout:.0f} s (still '
                f'{self.distance_to(target):.2f} m away) -- holding. Check '
                f'/skybeta/state for a clamp, and cuVSLAM for tracking loss.')
            self.state = 'HOLD'

    def _state_hold(self):
        target = self.corners[-1] if self.corners else self.position
        self.send(*target)

    # ------------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = SquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
