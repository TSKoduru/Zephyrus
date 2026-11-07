"""Frame-conversion tests.

These deliberately mirror ``oakd_vslam_bringup/test/test_px4_odom_bridge.py``:
the two packages implement the same ENU/FLU -> NED/FRD math independently, so
the tests are the only thing keeping them from drifting apart.  Run inside the
skybeta container:

    source /opt/ros/humble/setup.bash
    cd src/skybeta_flight_interface && python3 -m pytest -q test/
"""
import math

import pytest

from skybeta_flight_interface.frames import (
    AIRCRAFT_BASELINK_Q,
    NED_ENU_Q,
    enu_to_ned,
    flu_to_frd,
    ned_to_enu,
    quat_enu_flu_to_ned_frd,
    quat_mul,
    setpoint_position_enu_to_ned,
    wrap_pi,
    yaw_enu_to_ned,
    yaw_from_wxyz,
    yaw_ned_from_quat_xyzw,
    yaw_ned_to_enu,
    yaw_rate_enu_to_ned,
)

SQRT_HALF = math.sqrt(0.5)


def enu_yaw_quat_xyzw(yaw):
    """ROS xyzw quaternion for a pure yaw about ENU +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# --------------------------------------------------------------------------
# constants must be byte-identical to the bridge's
# --------------------------------------------------------------------------
def test_constants_match_the_bridge():
    assert NED_ENU_Q == (0.0, SQRT_HALF, SQRT_HALF, 0.0)
    assert AIRCRAFT_BASELINK_Q == (0.0, 1.0, 0.0, 0.0)
    # Both are self-inverse 180 deg rotations.
    for q in (NED_ENU_Q, AIRCRAFT_BASELINK_Q):
        product = quat_mul(q, q)
        assert abs(abs(product[0]) - 1.0) < 1e-12
        assert product[1:] == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_bridge_source_constants_are_identical():
    """Same literals as oakd_vslam_bringup, if that package is installed."""
    bridge = pytest.importorskip('oakd_vslam_bringup.px4_odom_bridge')
    assert NED_ENU_Q == bridge.NED_ENU_Q
    assert AIRCRAFT_BASELINK_Q == bridge.AIRCRAFT_BASELINK_Q


# --------------------------------------------------------------------------
# translation
# --------------------------------------------------------------------------
def test_position_enu_to_ned():
    # Exactly the bridge's `out.position = [p.y, p.x, -p.z]`.
    assert enu_to_ned((1.0, 2.0, 3.0)) == pytest.approx((2.0, 1.0, -3.0))


def test_enu_ned_permutation_is_an_involution():
    for vector in [(1.0, 2.0, 3.0), (-4.5, 0.0, 7.25), (0.0, 0.0, 0.0)]:
        assert ned_to_enu(enu_to_ned(vector)) == pytest.approx(vector)


def test_body_flu_to_frd_is_sign_flips_only():
    # Same as the bridge's velocity/angular_velocity mapping.
    assert flu_to_frd((1.0, 2.0, 3.0)) == pytest.approx((1.0, -2.0, -3.0))
    assert flu_to_frd((4.0, 5.0, 6.0)) == pytest.approx((4.0, -5.0, -6.0))


def test_world_and_body_maps_are_different():
    """The whole point of bug #4: these are not interchangeable."""
    vector = (1.0, 2.0, 3.0)
    assert enu_to_ned(vector) != flu_to_frd(vector)


# --------------------------------------------------------------------------
# attitude -- the bridge's headline case
# --------------------------------------------------------------------------
def test_identity_enu_attitude_is_ninety_degrees_in_ned():
    """ENU identity (body +x East) is heading +90 deg in NED."""
    q = quat_enu_flu_to_ned_frd((0.0, 0.0, 0.0, 1.0))
    expected = quat_mul(quat_mul(NED_ENU_Q, (1.0, 0.0, 0.0, 0.0)),
                        AIRCRAFT_BASELINK_Q)
    assert list(q) == pytest.approx(list(expected), abs=1e-9)
    assert math.sqrt(sum(v * v for v in q)) == pytest.approx(1.0, abs=1e-9)
    assert math.degrees(yaw_from_wxyz(q)) == pytest.approx(90.0, abs=1e-6)


@pytest.mark.parametrize('yaw_enu_deg', [0.0, 45.0, 90.0, 180.0, -90.0, 137.0])
def test_yaw_ned_is_ninety_minus_yaw_enu(yaw_enu_deg):
    """yaw_ned = 90 deg - yaw_enu, via the full quaternion composition."""
    yaw_enu = math.radians(yaw_enu_deg)
    got = yaw_ned_from_quat_xyzw(enu_yaw_quat_xyzw(yaw_enu))
    want = wrap_pi(math.radians(90.0) - yaw_enu)
    assert wrap_pi(got - want) == pytest.approx(0.0, abs=1e-6)
    # ...and the cheap scalar helper agrees with the quaternion route.
    assert wrap_pi(got - yaw_enu_to_ned(yaw_enu)) == pytest.approx(
        0.0, abs=1e-6)


def test_ninety_degree_enu_yaw_is_minus_ninety_in_ned():
    base = yaw_ned_from_quat_xyzw(enu_yaw_quat_xyzw(0.0))
    turned = yaw_ned_from_quat_xyzw(
        enu_yaw_quat_xyzw(math.radians(90.0)))
    assert math.degrees(wrap_pi(turned - base)) == pytest.approx(
        -90.0, abs=1e-6)
    # +90 deg in ENU points the body North, i.e. NED yaw 0.
    assert math.degrees(turned) == pytest.approx(0.0, abs=1e-6)


def test_yaw_map_is_an_involution():
    for deg in (-180.0, -90.0, 0.0, 30.0, 90.0, 179.0):
        yaw = math.radians(deg)
        assert wrap_pi(yaw_ned_to_enu(yaw_enu_to_ned(yaw)) - yaw) == (
            pytest.approx(0.0, abs=1e-9))


def test_yaw_stays_wrapped():
    for deg in (-350.0, -181.0, 181.0, 359.0, 720.0):
        result = yaw_enu_to_ned(math.radians(deg))
        assert -math.pi - 1e-9 <= result <= math.pi + 1e-9


def test_yaw_rate_flips_sign():
    """ENU z is Up, NED z is Down, so a yaw rate about it negates."""
    assert yaw_rate_enu_to_ned(0.5) == pytest.approx(-0.5)
    assert yaw_rate_enu_to_ned(-2.0) == pytest.approx(2.0)


# --------------------------------------------------------------------------
# setpoint conversion, with and without anchoring
# --------------------------------------------------------------------------
def test_setpoint_without_anchors_is_the_plain_rotation():
    assert setpoint_position_enu_to_ned((1.0, 2.0, 3.0)) == pytest.approx(
        enu_to_ned((1.0, 2.0, 3.0)))


def test_setpoint_anchors_relative_to_both_origins():
    # 1 m East + 2 m North + 1.5 m Up of an odom origin that is itself at
    # NED (10, 20, -5) in EKF2's frame.
    result = setpoint_position_enu_to_ned(
        position_enu=(4.0, 7.0, 1.5),
        odom_origin_enu=(3.0, 5.0, 0.0),
        px4_origin_ned=(10.0, 20.0, -5.0),
    )
    # delta ENU (1, 2, 1.5) -> NED (2, 1, -1.5), then + origin
    assert result == pytest.approx((12.0, 21.0, -6.5))


def test_setpoint_yaw_offset_rotates_about_down():
    # A 1 m North displacement, rotated 90 deg about NED down, becomes 1 m East.
    result = setpoint_position_enu_to_ned(
        position_enu=(0.0, 1.0, 0.0),
        odom_origin_enu=(0.0, 0.0, 0.0),
        px4_origin_ned=(0.0, 0.0, 0.0),
        yaw_offset_rad=math.radians(90.0),
    )
    assert result == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    # ...and zero offset leaves it alone.
    assert setpoint_position_enu_to_ned(
        (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
    ) == pytest.approx((1.0, 0.0, 0.0))


def test_setpoint_z_is_unaffected_by_yaw_offset():
    result = setpoint_position_enu_to_ned(
        (0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        yaw_offset_rad=1.234)
    assert result[2] == pytest.approx(-2.0)
