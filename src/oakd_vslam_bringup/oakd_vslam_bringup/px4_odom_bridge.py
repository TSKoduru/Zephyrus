"""Bridge cuVSLAM nav_msgs/Odometry -> PX4 VehicleOdometry via uXRCE-DDS.

ROS convention (REP-103):  ENU world, FLU body.
PX4 convention:            NED world, FRD body.

This node republishes /visual_slam/tracking/odometry (ENU/FLU) on
/fmu/in/vehicle_visual_odometry (NED/FRD) with the QoS profile PX4's
uxrce_dds_client subscribers expect (BEST_EFFORT, VOLATILE, KEEP_LAST).

The frame math mirrors px4_ros_com's frame_transforms:
    q_ned_frd = NED_ENU_Q * q_enu_flu * AIRCRAFT_BASELINK_Q
where both constant quaternions are self-inverse 180 deg rotations.

pose_frame defaults to FRD, not NED
-----------------------------------
cuVSLAM initialises its `odom` frame at the rig's startup pose.  With
`enable_imu_fusion` off this is pure stereo VO: no gravity vector and no
magnetometer/GPS heading ever reach the tracker, so `odom` is a local frame
with an *arbitrary heading reference* -- the literal definition of
POSE_FRAME_FRD in px4_msgs/msg/VehicleOdometry.msg ("FRD world-fixed frame,
arbitrary heading reference").  Declaring POSE_FRAME_NED instead tells EKF2
the measurements are north-referenced when they are not, which leaves a
constant yaw error between the vision positions and the mag/GPS attitude:
the position controller then pushes at a fixed angle to the true error and
the vehicle orbits instead of converging (the "toilet bowl").  With FRD,
EKF2 estimates the frame rotation itself.

Override only if the vision frame really is yaw-aligned to north:

    ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args -p pose_frame:=NED

reset_counter
-------------
/visual_slam/status carries `vo_state` (0 = unknown, 1 = success/tracking,
2 = failed).  Each non-tracking -> tracking transition means cuVSLAM
re-initialised and its pose jumped; incrementing reset_counter tells EKF2 to
expect the discontinuity instead of integrating it as real motion.

Covariance
----------
Only `position_variance` takes the ENU->NED x/y permutation.
`orientation_variance` and `velocity_variance` describe *body*-frame
quantities whose FLU->FRD map is (x, -y, -z) -- sign flips only, which
square out of a variance, so their diagonals pass through in order.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:  # pragma: no cover - allows import without cuVSLAM
    VisualSlamStatus = None


# from_euler(pi, 0, pi/2) in wxyz
NED_ENU_Q = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
# from_euler(pi, 0, 0) in wxyz
AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)

# isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus declares no
# constants, only a comment block on `uint8 vo_state`:
#   0 - Unknown state / 1 - Success / 2 - Failed
# (visual_slam_impl.cpp: `vo_state = vo_status == CUVSLAM_SUCCESS ? 1 : 2`)
VO_STATE_UNKNOWN = 0
VO_STATE_SUCCESS = 1
VO_STATE_FAILED = 2

# Row-major 6x6 diagonal.
COV_DIAG = (0, 7, 14, 21, 28, 35)

POSE_FRAMES = {
    'FRD': VehicleOdometry.POSE_FRAME_FRD,
    'NED': VehicleOdometry.POSE_FRAME_NED,
    'UNKNOWN': VehicleOdometry.POSE_FRAME_UNKNOWN,
}


def resolve_pose_frame(name) -> int:
    """Map a 'FRD'/'NED' string (or a raw int) to a POSE_FRAME_* value."""
    if isinstance(name, int) and not isinstance(name, bool):
        return int(name)
    key = str(name).strip().upper()
    if key.startswith('POSE_FRAME_'):
        key = key[len('POSE_FRAME_'):]
    if key not in POSE_FRAMES:
        raise ValueError(
            f'pose_frame must be one of {sorted(POSE_FRAMES)}, '
            f'got {name!r}'
        )
    return POSE_FRAMES[key]


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def stamp_to_us(stamp) -> int:
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000


def is_identity_covariance(cov) -> bool:
    """True when every diagonal entry is exactly 1.0.

    cuVSLAM writes a 6x6 identity whenever `pose_cache.GetCovariance()`
    fails (visual_slam_impl.cpp), so this is its "no estimate" sentinel and
    not a real uncertainty.
    """
    return all(float(cov[i]) == 1.0 for i in COV_DIAG)


def odometry_to_vehicle_odometry(
    msg,
    pose_frame=VehicleOdometry.POSE_FRAME_FRD,
    reset_counter=0,
    publish_velocity=True,
    min_position_variance=0.0,
    timestamp_us=None,
) -> VehicleOdometry:
    """Convert one ENU/FLU nav_msgs/Odometry into a PX4 VehicleOdometry.

    Pure function: no node, no clock, no I/O.  `timestamp_us` is the publish
    time in the ROS-time domain; when omitted it falls back to the sample
    time so the result is fully determined by the input.
    """
    out = VehicleOdometry()
    out.timestamp_sample = stamp_to_us(msg.header.stamp)
    out.timestamp = (
        out.timestamp_sample if timestamp_us is None else int(timestamp_us)
    )

    out.pose_frame = int(pose_frame)

    p = msg.pose.pose.position
    # ENU -> NED: (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)
    out.position = [float(p.y), float(p.x), float(-p.z)]

    # ROS quaternion is xyzw; convert to wxyz for internal math.
    q_ros = msg.pose.pose.orientation
    q_enu_flu = (q_ros.w, q_ros.x, q_ros.y, q_ros.z)
    q_tmp = quat_mul(NED_ENU_Q, q_enu_flu)
    q_ned_frd = quat_mul(q_tmp, AIRCRAFT_BASELINK_Q)
    out.q = [float(q_ned_frd[0]), float(q_ned_frd[1]),
             float(q_ned_frd[2]), float(q_ned_frd[3])]

    if publish_velocity:
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        # FLU -> FRD body: (x, -y, -z)
        out.velocity = [float(v.x), float(-v.y), float(-v.z)]
        out.angular_velocity = [float(w.x), float(-w.y), float(-w.z)]
    else:
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [float('nan')] * 3
        out.angular_velocity = [float('nan')] * 3

    # Covariance: ROS ships a full 6x6 row-major (36 floats); PX4 wants 3
    # diagonal variances per category.
    #
    # position is a *world*-frame quantity, so ENU->NED permutes x/y:
    #   var_x_ned = var_y_enu, var_y_ned = var_x_enu.
    # orientation and velocity are *body*-frame quantities (cuVSLAM's twist
    # is body-frame -- pose_cache.cpp differentiates
    # `poses.front().inverse() * poses.back()`), and FLU->FRD is (x, -y, -z):
    # sign flips only, which square out of a variance.  No permutation.
    pc = msg.pose.covariance
    tc = msg.twist.covariance
    pos_var = [float(pc[7]), float(pc[0]), float(pc[14])]
    if min_position_variance > 0.0:
        floor = float(min_position_variance)
        pos_var = [max(value, floor) for value in pos_var]
    out.position_variance = pos_var
    out.orientation_variance = [float(pc[21]), float(pc[28]), float(pc[35])]
    if publish_velocity:
        out.velocity_variance = [float(tc[0]), float(tc[7]), float(tc[14])]
    else:
        out.velocity_variance = [float('nan')] * 3

    out.reset_counter = int(reset_counter) & 0xFF
    out.quality = 0  # 0 = unknown per px4_msgs; EKF2 tolerates this
    return out


class ResetCounter:
    """uint8 counter bumped on every non-tracking -> tracking transition."""

    def __init__(self):
        self.value = 0
        self.previous_state = None

    def update(self, vo_state) -> bool:
        """Feed one VisualSlamStatus.vo_state; True if the counter moved."""
        state = int(vo_state)
        previous = self.previous_state
        self.previous_state = state
        if previous is None:
            # First sample: no transition is observable yet.
            return False
        if state == VO_STATE_SUCCESS and previous != VO_STATE_SUCCESS:
            self.value = (self.value + 1) & 0xFF
            return True
        return False


class PX4OdomBridge(Node):

    def __init__(self, **kwargs):
        # **kwargs is forwarded to rclpy Node (parameter_overrides, context,
        # ...) so tests can construct the node without spinning.
        super().__init__('px4_odom_bridge', **kwargs)
        self.declare_parameter('input_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter(
            'output_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('status_topic', '/visual_slam/status')
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('pose_frame', 'FRD')
        self.declare_parameter('min_position_variance', 0.0)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.publish_velocity = self.get_parameter('publish_velocity').value
        self.pose_frame = resolve_pose_frame(
            self.get_parameter('pose_frame').value)
        self.min_position_variance = float(
            self.get_parameter('min_position_variance').value)

        self.reset_counter = ResetCounter()
        self.identity_covariance_warned = False
        self.identity_covariance_streak = 0
        # ~1 s at 30 Hz. Long enough to ride out the tracker-startup transient,
        # short enough that a genuinely broken GetCovariance() is still loud.
        self.identity_covariance_grace = 30

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(VehicleOdometry, out_topic, px4_qos)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)
        self.status_sub = None
        if VisualSlamStatus is not None and status_topic:
            # cuVSLAM publishes visual_slam/status with ParseQosString(
            # "DEFAULT") -> RELIABLE/VOLATILE/KEEP_LAST(10).
            self.status_sub = self.create_subscription(
                VisualSlamStatus, status_topic, self.status_cb, 10)

        frame_name = 'FRD'
        if self.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            frame_name = 'NED'
        velocity_state = 'on' if self.publish_velocity else 'off'
        status_state = status_topic if self.status_sub else 'unavailable'
        self.get_logger().info(
            f'PX4 odom bridge up: {in_topic} -> {out_topic} '
            f'(velocity={velocity_state}, '
            f'pose_frame={frame_name}[{self.pose_frame}], '
            f'status={status_state})'
        )
        if self.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(
                'pose_frame=NED asserts the vision frame is north-referenced. '
                'cuVSLAM without IMU fusion has no heading reference; use FRD '
                'unless the odom frame has been measured to be yaw-aligned.'
            )
        if self.status_sub is None:
            self.get_logger().warn(
                'No /visual_slam/status subscription: reset_counter will stay '
                'at 0 and EKF2 will integrate cuVSLAM re-initialisations as '
                'real motion.'
            )

    def status_cb(self, msg):
        if self.reset_counter.update(msg.vo_state):
            self.get_logger().info(
                f'cuVSLAM re-acquired tracking (vo_state={msg.vo_state}); '
                f'reset_counter -> {self.reset_counter.value}'
            )

    def _check_covariance(self, pose_covariance):
        # cuVSLAM's PoseCache needs two poses before GetCovariance() can return
        # anything, so the first frame or two after the tracker starts always
        # carry the 1.0 identity fallback. Measured on the bench: 0 of 1313
        # steady-state messages were identity, but the very first ones were.
        # Warning on the first occurrence therefore cries wolf on every single
        # run. Only warn once identity has persisted for a full second.
        if self.identity_covariance_warned:
            return
        if not is_identity_covariance(pose_covariance):
            self.identity_covariance_streak = 0
            return
        self.identity_covariance_streak += 1
        if self.identity_covariance_streak < self.identity_covariance_grace:
            return
        self.identity_covariance_warned = True
        self.get_logger().warn(
            'cuVSLAM pose covariance is the identity fallback (all diagonal '
            'entries == 1.0), i.e. GetCovariance() failed and these are not '
            'real variances. With EKF2_EV_NOISE_MD=0 PX4 takes the message '
            'variance as a lower bound, so 1.0 m^2 / 1.0 rad^2 makes EKF2 '
            'nearly ignore the vision measurements. Set EKF2_EV_NOISE_MD=1 to '
            'use EKF2_EVP/EVV/EVA_NOISE directly, or set the '
            'min_position_variance parameter as a stopgap.'
        )

    def cb(self, msg: Odometry):
        self._check_covariance(msg.pose.covariance)
        out = odometry_to_vehicle_odometry(
            msg,
            pose_frame=self.pose_frame,
            reset_counter=self.reset_counter.value,
            publish_velocity=self.publish_velocity,
            min_position_variance=self.min_position_variance,
            timestamp_us=self.get_clock().now().nanoseconds // 1000,
        )
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
