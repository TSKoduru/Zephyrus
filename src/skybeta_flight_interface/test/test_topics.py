"""Topic-name resolution against both px4_msgs generations.

The fixtures are real excerpts: ``release/1.14`` declares no MESSAGE_VERSION
anywhere, ``release/1.16`` declares it on most messages with VehicleStatus at
version 1 and everything else this package touches at 0 (and
OffboardControlMode with no constant at all).
"""
import pytest

from skybeta_flight_interface.topics import (
    FMU_ENDPOINTS,
    fmu_topic,
    message_version,
    resolve_endpoints,
    snake_case,
)

# --- fixtures --------------------------------------------------------------
# px4_msgs release/1.14: no message is versioned.
MSGS_1_14 = {
    'VehicleStatus': (
        '# Encodes the system state of the vehicle published by commander\n'
        'uint64 timestamp\n'
        'uint8 arming_state\n'
        'uint8 ARMING_STATE_ARMED = 2\n'
    ),
    'VehicleLocalPosition': 'uint64 timestamp\nbool xy_valid\n',
    'VehicleLandDetected': 'uint64 timestamp\nbool landed\n',
    'OffboardControlMode': 'uint64 timestamp\nbool position\n',
    'TrajectorySetpoint': 'uint64 timestamp\nfloat32[3] position\n',
    'VehicleCommand': 'uint64 timestamp\nuint32 command\n',
}

# px4_msgs release/1.16: VehicleStatus bumped to 1, the rest at 0, and
# OffboardControlMode still carries no constant.
MSGS_1_16 = {
    'VehicleStatus': (
        '# Encodes the system state of the vehicle published by commander\n'
        'uint32 MESSAGE_VERSION = 1\n'
        'uint64 timestamp\n'
        'uint8 ARMING_STATE_ARMED    = 2\n'
    ),
    'VehicleLandDetected': 'uint32 MESSAGE_VERSION = 0\nbool landed\n',
    'VehicleLocalPosition': (
        '# Fused local position in NED.\n'
        'uint32 MESSAGE_VERSION = 0\n'
        'uint64 timestamp\n'
    ),
    'OffboardControlMode': '# Off-board control mode\nuint64 timestamp\n',
    'TrajectorySetpoint': (
        '# Trajectory setpoint in NED frame\n'
        'uint32 MESSAGE_VERSION = 0\n'
        'uint64 timestamp\n'
    ),
    'VehicleCommand': (
        '# Vehicle Command uORB message.\n'
        'uint32 MESSAGE_VERSION = 0\n'
        'uint64 timestamp\n'
    ),
}


def _write(tmp_path, name, messages):
    directory = tmp_path / name
    directory.mkdir()
    for msg_name, body in messages.items():
        (directory / f'{msg_name}.msg').write_text(body, encoding='utf-8')
    return str(directory)


@pytest.fixture
def msgs_1_14(tmp_path):
    return _write(tmp_path, 'px4_msgs_1_14', MSGS_1_14)


@pytest.fixture
def msgs_1_16(tmp_path):
    return _write(tmp_path, 'px4_msgs_1_16', MSGS_1_16)


# --- snake_case ------------------------------------------------------------
@pytest.mark.parametrize('camel,snake', [
    ('VehicleStatus', 'vehicle_status'),
    ('VehicleLocalPosition', 'vehicle_local_position'),
    ('OffboardControlMode', 'offboard_control_mode'),
    ('TrajectorySetpoint', 'trajectory_setpoint'),
    ('VehicleOdometry', 'vehicle_odometry'),
])
def test_snake_case(camel, snake):
    assert snake_case(camel) == snake


# --- message_version -------------------------------------------------------
def test_message_version_1_14_is_all_zero(msgs_1_14):
    for name in MSGS_1_14:
        assert message_version(name, msgs_1_14) == 0


def test_message_version_1_16(msgs_1_16):
    assert message_version('VehicleStatus', msgs_1_16) == 1
    assert message_version('VehicleLocalPosition', msgs_1_16) == 0
    assert message_version('TrajectorySetpoint', msgs_1_16) == 0
    assert message_version('VehicleCommand', msgs_1_16) == 0
    # No constant at all is also version 0.
    assert message_version('OffboardControlMode', msgs_1_16) == 0


def test_message_version_missing_file_is_zero(tmp_path):
    assert message_version('NoSuchMessage', str(tmp_path)) == 0


def test_message_version_missing_dir_is_zero():
    assert message_version('VehicleStatus', '/nonexistent/px4_msgs/msg') == 0


def test_message_version_ignores_a_field_named_like_the_constant(tmp_path):
    directory = tmp_path / 'msg'
    directory.mkdir()
    (directory / 'Weird.msg').write_text(
        '# MESSAGE_VERSION = 9 in a comment\n'
        'uint32 message_version_field\n'
        'uint8 OTHER_MESSAGE_VERSION = 7\n',
        encoding='utf-8')
    assert message_version('Weird', str(directory)) == 0


def test_message_version_tolerates_trailing_comment(tmp_path):
    directory = tmp_path / 'msg'
    directory.mkdir()
    (directory / 'Thing.msg').write_text(
        'uint32 MESSAGE_VERSION = 3  # bumped for the new field\n',
        encoding='utf-8')
    assert message_version('Thing', str(directory)) == 3


# --- fmu_topic -------------------------------------------------------------
def test_topics_on_1_14_have_no_suffix(msgs_1_14):
    resolved = resolve_endpoints(msgs_1_14)
    assert resolved == {
        'offboard_control_mode': '/fmu/in/offboard_control_mode',
        'trajectory_setpoint': '/fmu/in/trajectory_setpoint',
        'vehicle_command': '/fmu/in/vehicle_command',
        'vehicle_status': '/fmu/out/vehicle_status',
        'vehicle_local_position': '/fmu/out/vehicle_local_position',
        'vehicle_land_detected': '/fmu/out/vehicle_land_detected',
    }


def test_topics_on_1_16_suffix_only_vehicle_status(msgs_1_16):
    resolved = resolve_endpoints(msgs_1_16)
    assert resolved == {
        'offboard_control_mode': '/fmu/in/offboard_control_mode',
        'trajectory_setpoint': '/fmu/in/trajectory_setpoint',
        'vehicle_command': '/fmu/in/vehicle_command',
        # The one that actually moved in v1.16.
        'vehicle_status': '/fmu/out/vehicle_status_v1',
        'vehicle_local_position': '/fmu/out/vehicle_local_position',
        'vehicle_land_detected': '/fmu/out/vehicle_land_detected',
    }


def test_endpoints_cover_every_publisher_and_subscriber():
    assert set(FMU_ENDPOINTS) == {
        'offboard_control_mode', 'trajectory_setpoint', 'vehicle_command',
        'vehicle_status', 'vehicle_local_position', 'vehicle_land_detected'}
    for direction, base, msg in FMU_ENDPOINTS.values():
        assert direction in ('in', 'out')
        assert snake_case(msg) == base


def test_topic_alias_keeps_the_uorb_name(msgs_1_16):
    """One message, several uORB topics: the base name wins, not the type."""
    assert fmu_topic('in', 'vehicle_visual_odometry', 'VehicleOdometry',
                     msgs_1_16) == '/fmu/in/vehicle_visual_odometry'


def test_msg_name_defaults_to_camel_case_of_base(msgs_1_16):
    assert fmu_topic('out', 'vehicle_status', msg_dir=msgs_1_16) == (
        '/fmu/out/vehicle_status_v1')


def test_bad_direction_raises(msgs_1_16):
    with pytest.raises(ValueError):
        fmu_topic('sideways', 'vehicle_status', msg_dir=msgs_1_16)


def test_version_greater_than_one_is_suffixed(tmp_path):
    directory = tmp_path / 'msg'
    directory.mkdir()
    (directory / 'FutureThing.msg').write_text(
        'uint32 MESSAGE_VERSION = 12\n', encoding='utf-8')
    assert fmu_topic('out', 'future_thing', 'FutureThing',
                     str(directory)) == '/fmu/out/future_thing_v12'


# --- against the px4_msgs actually installed --------------------------------
def test_resolves_against_the_installed_px4_msgs():
    """Whatever generation is in this image, the names must be well-formed."""
    pytest.importorskip('px4_msgs.msg')
    resolved = resolve_endpoints()
    for key, topic in resolved.items():
        direction = FMU_ENDPOINTS[key][0]
        assert topic.startswith(f'/fmu/{direction}/')
        assert topic.split('/')[-1].startswith(FMU_ENDPOINTS[key][1])
