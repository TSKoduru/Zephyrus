"""Terminal low-speed touchdown, explicitly gated by final route location."""
import numpy as np
from .controller import limit_norm


class LandingFinish:
    def __init__(self, endpoint, max_speed):
        self.endpoint = np.asarray(endpoint, dtype=float)
        self.max_speed = max_speed
        self.started = None
        self.airborne = False
        self.done = False

    def update(self, now, pose, reference, landed, velocity):
        self.airborne |= landed is False
        if self.done:
            return 'landed', np.zeros(3)
        near_end = (np.linalg.norm(np.asarray(reference)[:2]-self.endpoint[:2]) < .03
                    and reference[2] <= self.endpoint[2]+.16
                    and np.linalg.norm(np.asarray(pose)[:2]-self.endpoint[:2]) < .10
                    and pose[2] <= self.endpoint[2]+.20)
        if self.started is None and self.airborne and near_end:
            self.started = now
        if self.started is None:
            return 'following', velocity
        if landed is True:
            self.done = True
            return 'landed', np.zeros(3)
        if now-self.started > 12 or pose[2] < self.endpoint[2]-.10:
            return 'landing_fault', np.zeros(3)
        if np.linalg.norm(np.asarray(pose)[:2]-self.endpoint[:2]) > .20:
            return 'landing_fault', np.zeros(3)
        velocity = np.asarray(velocity, dtype=float).copy()
        velocity[2] = -.08
        return 'landing', limit_norm(velocity, self.max_speed)
