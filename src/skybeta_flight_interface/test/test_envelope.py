"""Safety-envelope clamp tests.

The envelope is the last thing between a competitor's arithmetic mistake and
a drone hitting the ceiling or the floor, so these cover the boundary
explicitly: on the limit is inside, past it is clipped back onto it, and
nothing is ever rejected outright.  There is no horizontal geofence: x and y
always pass through.
"""
import math

import pytest

from skybeta_flight_interface.envelope import SafetyEnvelope

ORIGIN = (0.0, 0.0, 0.0)


@pytest.fixture
def envelope():
    """The shipped defaults."""
    return SafetyEnvelope()


# --------------------------------------------------------------------------
# defaults and validation
# --------------------------------------------------------------------------
def test_defaults_match_the_documented_envelope(envelope):
    assert not hasattr(envelope, 'geofence_radius_m')
    assert envelope.max_altitude_m == 3.0
    assert envelope.min_altitude_m == 0.3
    assert envelope.max_speed_mps == 1.5


@pytest.mark.parametrize('kwargs', [
    {'max_speed_mps': 0.0},
    {'min_altitude_m': 5.0, 'max_altitude_m': 3.0},
])
def test_nonsense_envelopes_are_rejected_at_construction(kwargs):
    with pytest.raises(ValueError):
        SafetyEnvelope(**kwargs)


# --------------------------------------------------------------------------
# inside
# --------------------------------------------------------------------------
@pytest.mark.parametrize('position', [
    (0.0, 0.0, 1.5),
    (3.0, 4.0, 0.3),      # exactly 5 m out, exactly on the floor
    (-6.0, 8.0, 3.0),     # exactly 10 m out, exactly on the ceiling
    (0.0, 0.0, 0.3),
])
def test_inside_passes_through_untouched(envelope, position):
    clamp = envelope.clamp_position(position, ORIGIN)
    assert clamp.value == pytest.approx(position)
    assert clamp.reasons == ()
    assert not clamp.clamped
    assert envelope.contains(position, ORIGIN)


# --------------------------------------------------------------------------
# no horizontal geofence
# --------------------------------------------------------------------------
@pytest.mark.parametrize('position', [
    (18.0, 24.0, 1.5),        # 30 m out
    (-500.0, 1000.0, 2.0),
    (1e6, 0.0, 0.3),
])
def test_far_horizontal_setpoints_pass_through(envelope, position):
    clamp = envelope.clamp_position(position, ORIGIN)
    assert clamp.reasons == ()
    assert clamp.value == pytest.approx(position)
    assert envelope.contains(position, ORIGIN)


def test_clamped_result_is_itself_inside(envelope):
    for position in [(50.0, 0.0, 99.0), (-40.0, -40.0, -10.0),
                     (0.0, 11.0, 2.0)]:
        clamped = envelope.clamp_position(position, ORIGIN).value
        assert envelope.contains(clamped, ORIGIN), clamped


def test_altitude_band_is_measured_from_the_captured_origin(envelope):
    origin = (100.0, -50.0, 2.0)
    # 1.5 m up *of the origin* is inside...
    inside = (101.0, -50.0, 3.5)
    assert envelope.contains(inside, origin)
    assert envelope.clamp_position(inside, origin).value == pytest.approx(
        inside)
    # ...while the same absolute point measured from (0,0,0) is over 3 m.
    assert not envelope.contains(inside, ORIGIN)


def test_origin_offset_survives_the_clamp(envelope):
    origin = (100.0, -50.0, 2.0)
    clamp = envelope.clamp_position((200.0, -50.0, 9.0), origin)
    assert clamp.reasons == ('max_altitude',)
    assert clamp.value == pytest.approx((200.0, -50.0, 5.0))


def test_setpoint_exactly_on_the_origin_does_not_divide_by_zero(envelope):
    clamp = envelope.clamp_position((0.0, 0.0, 100.0), ORIGIN)
    assert clamp.reasons == ('max_altitude',)
    assert clamp.value == pytest.approx((0.0, 0.0, 3.0))


# --------------------------------------------------------------------------
# altitude band
# --------------------------------------------------------------------------
def test_above_the_ceiling_is_clamped(envelope):
    clamp = envelope.clamp_position((1.0, 1.0, 12.0), ORIGIN)
    assert clamp.reasons == ('max_altitude',)
    assert clamp.value == pytest.approx((1.0, 1.0, 3.0))


def test_below_the_floor_is_clamped(envelope):
    clamp = envelope.clamp_position((1.0, 1.0, -5.0), ORIGIN)
    assert clamp.reasons == ('min_altitude',)
    assert clamp.value == pytest.approx((1.0, 1.0, 0.3))


def test_a_ground_level_setpoint_is_lifted_to_the_floor(envelope):
    """Commanding z=0 is the most common competitor bug; it must not land."""
    clamp = envelope.clamp_position((0.0, 0.0, 0.0), ORIGIN)
    assert clamp.reasons == ('min_altitude',)
    assert clamp.value[2] == pytest.approx(0.3)


def test_far_and_high_clamps_only_the_height(envelope):
    clamp = envelope.clamp_position((100.0, 0.0, 100.0), ORIGIN)
    assert clamp.reasons == ('max_altitude',)
    assert clamp.value == pytest.approx((100.0, 0.0, 3.0))


def test_altitude_is_relative_to_the_origin_z(envelope):
    origin = (0.0, 0.0, 10.0)   # SLAM started on a 10 m high platform
    clamp = envelope.clamp_position((0.0, 0.0, 11.0), origin)
    assert clamp.reasons == ()
    assert clamp.value == pytest.approx((0.0, 0.0, 11.0))
    clamp = envelope.clamp_position((0.0, 0.0, 20.0), origin)
    assert clamp.reasons == ('max_altitude',)
    assert clamp.value == pytest.approx((0.0, 0.0, 13.0))


# --------------------------------------------------------------------------
# speed
# --------------------------------------------------------------------------
def test_speed_under_the_cap_passes_through(envelope):
    for velocity in [(0.0, 0.0, 0.0), (1.0, 0.5, -0.2), (1.5, 0.0, 0.0)]:
        clamp = envelope.clamp_velocity(velocity)
        assert clamp.value == pytest.approx(velocity)
        assert clamp.reasons == ()


def test_speed_over_the_cap_is_scaled_keeping_direction(envelope):
    clamp = envelope.clamp_velocity((3.0, 4.0, 0.0))   # 5 m/s
    assert clamp.reasons == ('max_speed',)
    vx, vy, vz = clamp.value
    assert math.sqrt(vx * vx + vy * vy + vz * vz) == pytest.approx(1.5)
    assert (vx, vy) == pytest.approx((0.9, 1.2))
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(4.0, 3.0))


def test_speed_cap_counts_the_vertical_component(envelope):
    """Norm, not per-axis: 1.4 m/s on three axes is 2.42 m/s."""
    clamp = envelope.clamp_velocity((1.4, 1.4, 1.4))
    assert clamp.reasons == ('max_speed',)
    assert math.sqrt(sum(v * v for v in clamp.value)) == pytest.approx(1.5)


def test_custom_envelope_is_honoured():
    envelope = SafetyEnvelope(max_altitude_m=1.0, min_altitude_m=0.5,
                              max_speed_mps=0.5)
    assert envelope.clamp_position((5.0, 0.0, 5.0), ORIGIN).value == (
        pytest.approx((5.0, 0.0, 1.0)))
    assert envelope.clamp_velocity((0.0, 0.0, -2.0)).value == pytest.approx(
        (0.0, 0.0, -0.5))


def test_describe_mentions_every_limit(envelope):
    text = envelope.describe()
    for token in ('0.30', '3.00', '1.50'):
        assert token in text
    assert 'radius' not in text


def test_geofence_radius_is_no_longer_accepted():
    with pytest.raises(TypeError):
        SafetyEnvelope(geofence_radius_m=10.0)


# --------------------------------------------------------------------------
# vehicle_inside -- what /skybeta/state reports, floor excluded
# --------------------------------------------------------------------------
def test_vehicle_on_the_ground_is_inside(envelope):
    """Below the floor is not a breach: the pad, and every landing, is there."""
    assert envelope.vehicle_inside((0.0, 0.0, 0.0), ORIGIN)
    assert envelope.vehicle_inside((1.0, 1.0, 0.05), ORIGIN)
    # ...while the strict test used for setpoints still says no.
    assert not envelope.contains((0.0, 0.0, 0.0), ORIGIN)


def test_vehicle_above_the_ceiling_is_not_inside(envelope):
    assert not envelope.vehicle_inside((0.0, 0.0, 9.0), ORIGIN)
    # Far out horizontally is fine: there is no geofence.
    assert envelope.vehicle_inside((30.0, 0.0, 1.5), ORIGIN)
    assert envelope.vehicle_inside((30.0, 0.0, 0.0), ORIGIN)


def test_vehicle_inside_respects_the_origin(envelope):
    origin = (100.0, -50.0, 2.0)
    assert envelope.vehicle_inside((105.0, -50.0, 4.5), origin)
    assert not envelope.vehicle_inside((105.0, -50.0, 5.5), origin)
