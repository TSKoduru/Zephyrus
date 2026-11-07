"""Resolve /fmu/{in,out}/* topic names against the installed px4_msgs.

PX4 v1.16 introduced *message versioning*.  A versioned uORB message carries

    uint32 MESSAGE_VERSION = <N>

in its ``.msg`` file, and ``uxrce_dds_client`` advertises it on a topic with a
``_v<N>`` suffix so a mismatched client fails loudly instead of silently
decoding garbage.  ``VehicleStatus`` is at version 1 in release/1.16, so the
topic is ``/fmu/out/vehicle_status_v1``; ``VehicleLocalPosition``,
``TrajectorySetpoint``, ``VehicleCommand`` and ``VehicleOdometry`` are all at
version 0 and keep their bare names.  ``OffboardControlMode`` declares no
``MESSAGE_VERSION`` at all -- also a bare name.  In release/1.14 *no* message
declares one.

Hard-coding either spelling breaks the other, and the version numbers will
move again.  So instead of guessing, read the constant out of the ``.msg``
files that are actually installed next to the ``px4_msgs`` Python module and
build the name from that.  The rule (px4_msgs' own
``scripts/uorb_to_ros_urls.py`` and PX4's ``dds_topics.yaml``):

    /fmu/<direction>/<snake_name>              when N is 0 or absent
    /fmu/<direction>/<snake_name>_v<N>         when N > 0

A missing share directory or an unreadable ``.msg`` degrades to version 0,
i.e. the pre-1.16 spelling, rather than raising -- an unusable topic name is
easier to diagnose from a ``ros2 topic list`` than an exception at startup.
"""
import os
import re

_MESSAGE_VERSION_RE = re.compile(
    r'^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)\s*(?:#.*)?$')

_CAMEL_BOUNDARY_RE = re.compile(r'(?<!^)(?=[A-Z])')


def snake_case(camel_name):
    """'VehicleLocalPosition' -> 'vehicle_local_position'."""
    return _CAMEL_BOUNDARY_RE.sub('_', str(camel_name)).lower()


def px4_msgs_msg_dir():
    """Directory holding px4_msgs' installed .msg files, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('px4_msgs')
    except Exception:  # pragma: no cover - only when px4_msgs is absent
        return None
    msg_dir = os.path.join(share, 'msg')
    return msg_dir if os.path.isdir(msg_dir) else None


def message_version(msg_name, msg_dir=None):
    """MESSAGE_VERSION of ``msg_name`` ('VehicleStatus'), or 0 if absent.

    0 covers all three no-suffix cases: a pre-1.16 px4_msgs where no message
    is versioned, a 1.16 message that declares ``= 0``, and one that declares
    no constant (``OffboardControlMode``).
    """
    if msg_dir is None:
        msg_dir = px4_msgs_msg_dir()
    if not msg_dir:
        return 0
    path = os.path.join(msg_dir, f'{msg_name}.msg')
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            for line in handle:
                match = _MESSAGE_VERSION_RE.match(line)
                if match:
                    return int(match.group(1))
    except OSError:
        return 0
    return 0


def fmu_topic(direction, base_name, msg_name=None, msg_dir=None):
    """Build a versioned /fmu/{in,out}/<base_name> topic name.

    ``base_name`` is the uORB topic, not the message: PX4 publishes the one
    ``VehicleOdometry`` definition on ``vehicle_visual_odometry``,
    ``vehicle_mocap_odometry`` and ``vehicle_odometry``, so the two are
    passed separately.  When ``msg_name`` is omitted it is taken to be the
    CamelCase of ``base_name``.
    """
    if direction not in ('in', 'out'):
        raise ValueError(
            f"direction must be 'in' or 'out', got {direction!r}")
    if msg_name is None:
        msg_name = ''.join(part.capitalize()
                           for part in str(base_name).split('_'))
    version = message_version(msg_name, msg_dir)
    suffix = f'_v{version}' if version > 0 else ''
    return f'/fmu/{direction}/{base_name}{suffix}'


# (direction, base topic, message) for everything the flight interface binds.
FMU_ENDPOINTS = {
    'offboard_control_mode': ('in', 'offboard_control_mode',
                              'OffboardControlMode'),
    'trajectory_setpoint': ('in', 'trajectory_setpoint',
                            'TrajectorySetpoint'),
    'vehicle_command': ('in', 'vehicle_command', 'VehicleCommand'),
    'vehicle_status': ('out', 'vehicle_status', 'VehicleStatus'),
    'vehicle_local_position': ('out', 'vehicle_local_position',
                               'VehicleLocalPosition'),
    'vehicle_land_detected': ('out', 'vehicle_land_detected',
                              'VehicleLandDetected'),
}


def resolve_endpoints(msg_dir=None):
    """Resolve every FMU_ENDPOINTS entry to its topic name.

    Resolved once at node startup and logged, so the topic spelling actually
    in use is visible in the log next to the px4_msgs generation that
    produced it.
    """
    if msg_dir is None:
        msg_dir = px4_msgs_msg_dir()
    return {
        key: fmu_topic(direction, base, msg, msg_dir)
        for key, (direction, base, msg) in FMU_ENDPOINTS.items()
    }
