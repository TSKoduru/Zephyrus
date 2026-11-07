"""ROS (ENU/FLU) <-> PX4 (NED/FRD) frame conversions for setpoints.

This is the *outbound* twin of ``oakd_vslam_bringup.px4_odom_bridge``: that
module converts cuVSLAM odometry ENU/FLU -> NED/FRD on the way *into* PX4,
this one converts competitor setpoints ENU -> NED on the way *out* to
``TrajectorySetpoint``.  The constants and the quaternion composition are
deliberately re-implemented here rather than imported, so
``skybeta_flight_interface`` has no build- or run-time dependency on
``oakd_vslam_bringup`` and a competitor can delete or replace the SLAM
package without breaking the flight interface.  The two must agree; the unit
tests in ``test/test_frames.py`` mirror the bridge's own cases.

Conventions (REP-103 vs PX4)
---------------------------
ROS:  ENU world (x East, y North, z Up),   FLU body (x Fwd, y Left, z Up).
PX4:  NED world (x North, y East, z Down), FRD body (x Fwd, y Right, z Down).

World-frame vectors (position, and ``TrajectorySetpoint.velocity``, which PX4
documents as "NED local world frame") take the x/y *permutation*:

    (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)

Body-frame vectors take *sign flips only*:

    (x_frd, y_frd, z_frd) = (x_flu, -y_flu, -z_flu)

Attitude follows px4_ros_com's frame_transforms:

    q_ned_frd = NED_ENU_Q * q_enu_flu * AIRCRAFT_BASELINK_Q

with both constants self-inverse 180 deg rotations.  The scalar consequence
used by every position setpoint is

    yaw_ned = pi/2 - yaw_enu

i.e. an ENU heading of 0 (body +x pointing East) is a NED heading of +90 deg,
and the map is its own inverse.
"""
import math

# from_euler(pi, 0, pi/2) in wxyz -- identical to px4_odom_bridge.NED_ENU_Q
NED_ENU_Q = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
# from_euler(pi, 0, 0) in wxyz -- identical to px4_odom_bridge.AIRCRAFT_BASELINK_Q
AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)

HALF_PI = math.pi / 2.0


def quat_mul(a, b):
    """Hamilton product of two wxyz quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def wrap_pi(angle):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_wxyz(q):
    """Yaw (rad) of a wxyz quaternion. Invariant under q -> -q."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def enu_to_ned(vector):
    """World-frame ENU -> NED: permute x/y, flip z.

    Use for positions and for ``TrajectorySetpoint.velocity``, which PX4
    expresses in the NED local *world* frame -- not the body frame.
    """
    x, y, z = vector
    return (float(y), float(x), float(-z))


# The permutation is an involution: NED -> ENU is the same operation.  Aliased
# rather than re-derived so the two can never drift apart.
ned_to_enu = enu_to_ned


def flu_to_frd(vector):
    """Body-frame FLU -> FRD: (x, -y, -z). Sign flips only, no permutation."""
    x, y, z = vector
    return (float(x), float(-y), float(-z))


def quat_enu_flu_to_ned_frd(q_xyzw):
    """ROS xyzw quaternion (ENU/FLU) -> PX4 wxyz quaternion (NED/FRD)."""
    x, y, z, w = (float(v) for v in q_xyzw)
    q_enu_flu = (w, x, y, z)
    return quat_mul(quat_mul(NED_ENU_Q, q_enu_flu), AIRCRAFT_BASELINK_Q)


def yaw_enu_to_ned(yaw_enu):
    """yaw_ned = pi/2 - yaw_enu, wrapped. Its own inverse."""
    return wrap_pi(HALF_PI - float(yaw_enu))


# Involution again: NED -> ENU is pi/2 - yaw either way.
yaw_ned_to_enu = yaw_enu_to_ned


def yaw_ned_from_quat_xyzw(q_xyzw):
    """NED yaw (rad) of an ENU/FLU orientation given as a ROS xyzw quaternion.

    Goes the long way round -- full quaternion composition, then extract the
    yaw -- so it is exercised by the same math the bridge uses.
    ``test_frames.py`` pins it to ``yaw_enu_to_ned(yaw_enu)``.
    """
    return yaw_from_wxyz(quat_enu_flu_to_ned_frd(q_xyzw))


def yaw_rate_enu_to_ned(yaw_rate_enu):
    """Yaw rate about ENU +z (Up) -> about NED +z (Down): negate."""
    return -float(yaw_rate_enu)


def setpoint_position_enu_to_ned(
    position_enu,
    odom_origin_enu=None,
    px4_origin_ned=None,
    yaw_offset_rad=0.0,
):
    """Convert an ENU setpoint position into PX4's NED local frame.

    With both origins ``None`` (the default) this is the plain REP-103
    rotation ``enu_to_ned`` and nothing else -- that is the documented
    contract of ``/skybeta/setpoint/pose``.

    The optional anchoring exists because the two frames are *not* the same
    frame.  Competitor setpoints are expressed in cuVSLAM's ``odom``, whose
    origin is the rig's pose when the SLAM node came up.  PX4's local NED
    frame is EKF2's, whose origin is wherever EKF2 initialised.  With
    ``pose_frame=FRD`` on the vision message EKF2 estimates the rotation
    between them itself, so the two frames differ by a translation and a yaw.
    Passing both origins expresses the setpoint as a *displacement* from the
    vision origin, rotates it by ``yaw_offset_rad`` about the NED down axis,
    and re-anchors it on the PX4 local position captured at that same instant.

    ``yaw_offset_rad`` is the residual heading of the vision frame relative to
    EKF2's NED and must be *measured* (see docs/competitor_guide.md); it is 0
    by default, which is only correct once that has been checked.
    """
    delta_enu = tuple(float(v) for v in position_enu)
    if odom_origin_enu is not None:
        delta_enu = tuple(
            float(p) - float(o) for p, o in zip(position_enu, odom_origin_enu)
        )
    x, y, z = enu_to_ned(delta_enu)

    offset = float(yaw_offset_rad)
    if offset != 0.0:
        # Rotation about the NED down (+z) axis: positive yaw turns North
        # toward East, which is the same handedness PX4 uses.
        cos_o, sin_o = math.cos(offset), math.sin(offset)
        x, y = x * cos_o - y * sin_o, x * sin_o + y * cos_o

    if px4_origin_ned is not None:
        x += float(px4_origin_ned[0])
        y += float(px4_origin_ned[1])
        z += float(px4_origin_ned[2])
    return (x, y, z)
