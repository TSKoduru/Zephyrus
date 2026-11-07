"""IMU yaw/tilt aid and depth keyframes for measured visual odometry."""

import numpy as np
from scipy.spatial.transform import Rotation
from .particle_filter import ParticleFilter, wrap_angle
from .imu import InertialIncrement, InertialAid


def replace_vo_yaw(delta_body4, covariance4, increment, noise_scale=1.0, level_translation=True):
    """Keep measured VO displacement, express it in yaw-level body axes, aid yaw."""
    delta = np.asarray(delta_body4, dtype=float).copy()
    cov = np.asarray(covariance4, dtype=float).copy()
    if delta.shape != (4,) or cov.shape != (4, 4) or noise_scale <= 0:
        raise ValueError("Invalid odometry/covariance")
    if increment.valid:
        if level_translation:
            R = Rotation.from_euler("xyz", [*increment.start_roll_pitch, 0.0]).as_matrix()
            delta[:3] = R @ delta[:3]
            cov[:3, :3] = R @ cov[:3, :3] @ R.T
        delta[3] = increment.yaw_delta
        cov[3, :] = 0.0
        cov[:, 3] = 0.0
        cov[3, 3] = increment.yaw_variance * noise_scale**2
    return (delta, cov)


class InertialAidedParticleFilter(ParticleFilter):
    """Convenience wrapper for replay; accepted translation still comes from VO.

    If a VO motion gate rejects translation, its bounded uncertain translation
    fallback remains in force. Independently valid gyro yaw is retained rather
    than being discarded with the failed visual translation.
    """

    def predict_aided(self, delta_body4, covariance4, increment, decision=None):
        delta, cov = replace_vo_yaw(delta_body4, covariance4, increment)
        if decision is None or decision.accepted:
            self.predict(delta, cov)
        else:
            from .motion_model import apply_motion_decision

            oldyaw = self.particles[:, 3].copy()
            self.particles = apply_motion_decision(self.particles, decision, self.rng)
            if increment.valid:
                self.particles[:, 3] = wrap_angle(
                    oldyaw
                    + increment.yaw_delta
                    + self.rng.normal(0, np.sqrt(increment.yaw_variance), self.n)
                )
            self._posterior = None

    def update_aided(self, frame, increment, visual_roll_pitch=(0.0, 0.0)):
        return self.update(frame, increment.roll_pitch if increment.valid else visual_roll_pitch)


class KeyframeInertialParticleFilter(InertialAidedParticleFilter):
    """Assimilate a depth image only after measured motion changes the view.

    Repeated stationary images share map error and stereo artifacts, so treating
    them as independent observations can manufacture false certainty. Prediction
    still runs on every odometry increment. Motion thresholds use VO/gyro only.
    """

    def __init__(self, *args, keyframe_translation=0.08, keyframe_yaw_deg=5.0, **kwargs):
        super().__init__(*args, **kwargs)
        if keyframe_translation <= 0 or keyframe_yaw_deg <= 0:
            raise ValueError("Positive keyframe thresholds required")
        self.keyframe_translation = float(keyframe_translation)
        self.keyframe_yaw = np.deg2rad(keyframe_yaw_deg)
        self.view_translation = np.zeros(3)
        self.view_yaw = 0.0
        self.force_measurement = False
        self.measurement_count = 0
        self.skipped_measurement_count = 0

    def predict_aided(self, delta_body4, covariance4, increment, decision=None):
        super().predict_aided(delta_body4, covariance4, increment, decision)
        delta, _ = replace_vo_yaw(delta_body4, covariance4, increment)
        if decision is None or decision.accepted:
            c, s = (np.cos(self.view_yaw), np.sin(self.view_yaw))
            self.view_translation += np.array(
                [c * delta[0] - s * delta[1], s * delta[0] + c * delta[1], delta[2]]
            )
        else:
            self.force_measurement = True
        self.view_yaw = float(wrap_angle(self.view_yaw + delta[3]))

    def update(self, frame, roll_pitch=(0.0, 0.0)):
        assimilate = (
            self.measurement_count == 0
            or self.force_measurement
            or np.linalg.norm(self.view_translation) >= self.keyframe_translation
            or (abs(self.view_yaw) >= self.keyframe_yaw)
        )
        if assimilate:
            result = super().update(frame, roll_pitch)
            if self.measurement_valid:
                self.measurement_count += 1
                self.view_translation[:] = 0.0
                self.view_yaw = 0.0
                self.force_measurement = False
        else:
            self.skipped_measurement_count += 1
            result = self.estimate()
        result["keyframe_update"] = bool(assimilate)
        result["keyframe_count"] = self.measurement_count
        result["skipped_depth_updates"] = self.skipped_measurement_count
        return result
