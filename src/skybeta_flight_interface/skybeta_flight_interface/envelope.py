"""Safety envelope: the clamp every competitor setpoint passes through.

Setpoints outside the envelope are **clamped, never rejected**.  Rejecting a
setpoint leaves PX4 holding whatever it had last, which for a velocity
setpoint means a vehicle that keeps flying at the command that took it out of
bounds; clamping keeps the stream continuous and pins the commanded state to
the boundary.  The clamp is deliberately dumb -- no hysteresis, no
projection along the violated axis -- because it has to be obviously correct
under review, not clever.

All quantities are in the ENU ``odom`` frame the competitor writes in, so the
clamp happens *before* any frame conversion.  Altitude is measured from the
origin captured at the first odometry sample, which for a vehicle sitting on
the ground at startup is ground level.

There is **no horizontal limit** (no geofence radius): it was removed for
0.3.0. Horizontal position setpoints pass through untouched; only the
altitude band and the speed cap are enforced here.
"""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Clamp:
    """Result of one clamp: the value to use, and why it changed."""

    value: tuple
    reasons: tuple = field(default=())

    @property
    def clamped(self):
        return bool(self.reasons)


@dataclass
class SafetyEnvelope:
    """Altitude band above the odom origin, plus a speed cap.

    max_altitude_m     ceiling above the origin
    min_altitude_m     floor above the origin -- NOT a landing inhibit; the
                       pilot lands on RC, this only stops a competitor
                       commanding the vehicle into the ground
    max_speed_mps      cap on the norm of a velocity setpoint
    """

    max_altitude_m: float = 3.0
    min_altitude_m: float = 0.3
    max_speed_mps: float = 1.5

    def __post_init__(self):
        self.max_altitude_m = float(self.max_altitude_m)
        self.min_altitude_m = float(self.min_altitude_m)
        self.max_speed_mps = float(self.max_speed_mps)
        if self.max_speed_mps <= 0.0:
            raise ValueError('max_speed_mps must be > 0')
        if self.min_altitude_m > self.max_altitude_m:
            raise ValueError(
                'min_altitude_m must be <= max_altitude_m '
                f'({self.min_altitude_m} > {self.max_altitude_m})')

    # ------------------------------------------------------------------
    def clamp_position(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """Clamp the altitude of an ENU position into the band above
        ``origin_enu``.  x and y are returned unchanged."""
        px, py, pz = (float(v) for v in position_enu)
        ox, oy, oz = (float(v) for v in origin_enu)
        dx, dy, dz = px - ox, py - oy, pz - oz
        reasons = []

        if dz > self.max_altitude_m:
            dz = self.max_altitude_m
            reasons.append('max_altitude')
        elif dz < self.min_altitude_m:
            dz = self.min_altitude_m
            reasons.append('min_altitude')

        return Clamp((ox + dx, oy + dy, oz + dz), tuple(reasons))

    # ------------------------------------------------------------------
    def clamp_velocity(self, velocity_enu):
        """Scale a velocity vector back to ``max_speed_mps`` if over."""
        vx, vy, vz = (float(v) for v in velocity_enu)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self.max_speed_mps:
            scale = self.max_speed_mps / speed
            return Clamp((vx * scale, vy * scale, vz * scale), ('max_speed',))
        return Clamp((vx, vy, vz), ())

    # ------------------------------------------------------------------
    def contains(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """True when an ENU position is inside the envelope, floor included.

        This is the strict test, and it is the right one for a *setpoint*.
        For the vehicle's own position use ``vehicle_inside``.
        """
        return not self.clamp_position(position_enu, origin_enu).clamped

    def vehicle_inside(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """True when the *vehicle* is under the ceiling.

        Drives the ``envelope_ok`` field of /skybeta/state, so a competitor
        can see the vehicle being carried above it by momentum or by the
        pilot.  Unlike ``contains`` it ignores ``min_altitude_m``: that floor
        exists to stop a competitor commanding the vehicle into the ground,
        and a vehicle sitting on the pad -- or being landed by the pilot -- is
        legitimately below it.  Reporting that as a breach would make
        ``envelope_ok`` false for the whole of every flight's start and end.
        """
        reasons = self.clamp_position(position_enu, origin_enu).reasons
        return not [r for r in reasons if r != 'min_altitude']

    def describe(self):
        return (f'altitude {self.min_altitude_m:.2f}-{self.max_altitude_m:.2f} m, '
                f'speed <= {self.max_speed_mps:.2f} m/s')
