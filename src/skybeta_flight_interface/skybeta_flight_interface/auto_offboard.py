"""When to ask PX4 for Offboard after the pilot arms.

The flow on the vehicle: the pilot arms on the ground, then switches to Position,
and the Jetson enters Offboard, so that switch is the go signal. This module
decides whether this switch may do that. It has no ROS in it, so every rule is
tested on its own.

The rules exist because the failure is a drone flown by team code when the pilot
did not expect it:

* **Only after a disarmed -> armed transition that we saw.** A flight interface
  that starts up, or a link that comes back, and finds the vehicle already armed
  does nothing: the pilot may be flying it.
* **Only on a switch into Position made after arming.** Arming with the switch
  already on Position gives no autonomy: that is how a pilot flies by hand. Moving
  out of Position and back on the ground counts as the switch.
* **Only on the ground.** A pilot flying by hand who flips to Position in the air
  wants position hold, not team code. That flip uses up this arming's attempt.
* **One attempt per arming.** Once the pilot takes the vehicle back, nothing here
  can take it again until it has been disarmed.
* **Only with valid position, a live setpoint and no hold in force.** With no team
  node running there is nothing to fly: the vehicle stays in Position, and PX4
  disarms it after COM_DISARM_PRFLT.
"""
from collections import namedtuple

#: PX4 can miss a command sent over the best-effort link, so it is repeated briefly.
RETRY_S = 0.5
WINDOW_S = 2.0

Inputs = namedtuple('Inputs', [
    'now_s',            # seconds, any monotonic origin
    'link',             # vehicle_status is arriving
    'armed',
    'in_position',      # nav_state is Position
    'landed',           # PX4 says landed; None until vehicle_land_detected arrives
    'offboard',         # nav_state is Offboard
    'position_valid',   # PX4 has a valid local position
    'setpoint_age_s',   # seconds since the team's last setpoint, None if never
    'frozen',           # the hold service is in force
])

WAIT_DISARMED = 'wait_disarmed'   # never saw this vehicle disarmed: not ours to touch
READY = 'ready'                   # disarmed: the next arming counts
ARMED_IN_POSITION = 'armed_in_position'   # armed on Position: the pilot's, until it leaves
ARMED = 'armed'                   # armed, off Position: the next switch to it is the go signal
REQUESTING = 'requesting'
DONE = 'done'                     # this arming has had its chance


class AutoOffboard:

    def __init__(self, enabled, setpoint_timeout_s):
        self.enabled = bool(enabled)
        self.setpoint_timeout_s = float(setpoint_timeout_s)
        self.state = WAIT_DISARMED
        self._deadline = 0.0
        self._next_send = 0.0

    def _refusal(self, s):
        """Why this switch to Position may not enter Offboard, or None."""
        if s.landed is None:
            return 'no landed state from PX4 (vehicle_land_detected)'
        if not s.landed:
            return 'in the air: Position stays with the pilot'
        if not s.position_valid:
            return 'no valid local position (is VIO tracking?)'
        if s.frozen:
            return 'the hold service is in force'
        if s.setpoint_age_s is None:
            return 'no team setpoint received'
        if s.setpoint_age_s > self.setpoint_timeout_s:
            return f'the last team setpoint is {s.setpoint_age_s:.1f} s old'
        return None

    def step(self, s):
        """One tick -> (send the mode request now, message to log or None)."""
        if not self.enabled:
            return False, None
        if not s.link:
            # The vehicle could be armed by the time the link returns.
            self.state = WAIT_DISARMED
            return False, None
        if not s.armed:
            self.state = READY
            return False, None
        if self.state == READY:
            if s.in_position:
                self.state = ARMED_IN_POSITION
                return False, ('armed already in Position: no autonomy. '
                               'To start it, arm first, then switch to Position')
            self.state = ARMED
            return False, 'armed: waiting for the switch to Position'
        if self.state == ARMED_IN_POSITION:
            if not s.in_position:
                self.state = ARMED
            return False, None
        if self.state == ARMED:
            if not s.in_position:
                return False, None
            self.state = DONE
            why = self._refusal(s)
            if why:
                return False, f'staying out of Offboard: {why}'
            self.state = REQUESTING
            self._deadline = s.now_s + WINDOW_S
            self._next_send = s.now_s + RETRY_S
            return True, 'switched to Position after arming: requesting Offboard'
        if self.state == REQUESTING:
            if s.offboard:
                self.state = DONE
                return False, 'Offboard active'
            if not s.in_position:
                self.state = DONE
                return False, 'mode changed while requesting Offboard: stopping'
            if s.now_s >= self._deadline:
                self.state = DONE
                return False, f'PX4 did not enter Offboard within {WINDOW_S:.0f} s'
            if s.now_s >= self._next_send:
                self._next_send = s.now_s + RETRY_S
                return True, None
        return False, None
