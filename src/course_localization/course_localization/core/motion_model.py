"""Incremental uncertainty for measured relative odometry, not absolute pose.

For Isaac ROS 4.3 specifically, /visual_slam/tracking/odometry.pose.covariance
is empirical variation across recent POSES, not estimation-error covariance.
The twist covariance is likewise variation of recent velocities. Both default
to identity while the cache has fewer than ten samples. Treating these fields
as independent increment uncertainty injects actual motion as process noise.
The separate tracking/vo_pose_covariance publisher uses cuVSLAM's covariance;
even that absolute covariance cannot be naively summed as independent steps.
Official source (release-4.3):
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.3/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L885-L932
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.3/isaac_ros_visual_slam/src/impl/pose_cache.cpp#L195-L280
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.3/isaac_ros_visual_slam/src/impl/pose_cache.cpp#L304-L340

This model instead accumulates variance with elapsed time, measured travel and
measured rotation. It never consumes the misleading empirical covariance.

Defaults are explicit engineering assumptions for small, slow indoor motion,
NOT a fitted cuVSLAM uncertainty model. Tune them on held-out real odometry
recordings. Persistent bias and abrupt tracking resets require separate handling;
this zero-mean noise model does not make those failures disappear.
"""

from dataclasses import dataclass, fields
import numpy as np


@dataclass(frozen=True)
class IncrementalNoise:
    # Standard-deviation coefficients; squared coefficients are variance rates.
    # Position units: m/sqrt(s), m/sqrt(m), and m/sqrt(rad), respectively.
    xy_per_sqrt_second: float = 0.015
    xy_per_sqrt_meter: float = 0.020
    xy_per_sqrt_radian: float = 0.005
    z_per_sqrt_second: float = 0.008
    z_per_sqrt_meter: float = 0.012
    z_per_sqrt_radian: float = 0.003
    # Yaw coefficients: rad/sqrt(s), rad/sqrt(m), rad/sqrt(rad).
    yaw_per_sqrt_second: float = float(np.deg2rad(0.30))
    yaw_per_sqrt_meter: float = float(np.deg2rad(0.60))
    yaw_per_sqrt_radian: float = float(np.deg2rad(1.00))

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{field.name} must be finite and nonnegative")


def incremental_motion_covariance(delta_body4, dt, noise=None):
    """Return a PSD 4x4 body-increment covariance for [dx,dy,dz,dyaw].

    delta_body4 must be a *measured* relative increment, in metres/radians.
    dyaw is the principal relative yaw in [-pi, pi]. dt is seconds.
    Process variance is q_time*dt + q_travel*||translation|| + q_turn*|dyaw|.
    This preserves total variance when a straight, constant-heading increment
    is subdivided into smaller updates, unlike a fixed noise floor per update.
    The time term maintains uncertainty growth while stationary. No cumulative
    pose covariance or ground-truth pose is accepted by this interface.
    """
    delta = np.asarray(delta_body4, dtype=float)
    if delta.shape != (4,) or not np.isfinite(delta).all():
        raise ValueError("delta_body4 must be a finite four-vector")
    dt = float(dt)
    if not np.isfinite(dt) or dt < 0:
        raise ValueError("dt must be finite and nonnegative")
    if abs(delta[3]) > np.pi + 1e-10:
        raise ValueError("dyaw must be a wrapped relative angle in [-pi, pi]")
    if dt == 0 and np.any(delta != 0):
        raise ValueError("A nonzero motion increment requires positive dt")
    if noise is None:
        noise = IncrementalNoise()
    if not isinstance(noise, IncrementalNoise):
        raise TypeError("noise must be an IncrementalNoise configuration")
    components = np.array([dt, np.linalg.norm(delta[:3]), abs(delta[3])])
    xy = np.array([noise.xy_per_sqrt_second, noise.xy_per_sqrt_meter, noise.xy_per_sqrt_radian])
    z = np.array([noise.z_per_sqrt_second, noise.z_per_sqrt_meter, noise.z_per_sqrt_radian])
    yaw = np.array([noise.yaw_per_sqrt_second, noise.yaw_per_sqrt_meter, noise.yaw_per_sqrt_radian])
    variance = np.array(
        [xy**2 @ components, xy**2 @ components, z**2 @ components, yaw**2 @ components]
    )
    return np.diag(variance)


@dataclass(frozen=True)
class MotionIntegrityLimits:
    """Declared operating envelope, not a bound obtained from the true route.

    Acceleration checking is opt-in. Its midpoint-velocity allowance is
    max_acceleration * mean(previous_dt, dt) + velocity_tolerance. Three
    consistent readings can establish a new velocity anchor after rejection;
    they do not by themselves restore localization confidence. This detects
    abrupt changes, not sustained smooth bias, and uses yaw-only transport.
    """

    max_speed_m_s: float = 1.5
    max_yaw_rate_rad_s: float = 2.5
    translation_tolerance_m: float = 0.08
    yaw_tolerance_rad: float = 0.10
    max_interval_s: float = 1.0
    recovery_updates: int = 3
    enable_acceleration: bool = False
    max_acceleration_m_s2: float = 3.0
    velocity_tolerance_m_s: float = 0.3

    def __post_init__(self):
        for field in fields(self):
            if field.name == "enable_acceleration":
                if not isinstance(self.enable_acceleration, bool):
                    raise ValueError("enable_acceleration must be boolean")
                continue
            value = getattr(self, field.name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be finite and positive")
        if int(self.recovery_updates) != self.recovery_updates:
            raise ValueError("recovery_updates must be an integer")


@dataclass(frozen=True)
class MotionDecision:
    accepted: bool
    delta4: np.ndarray
    covariance4: np.ndarray
    status: str
    reason: str
    translation_radius: float = 0.0
    yaw_radius: float = 0.0


class MotionIntegrityGate:
    """Reject impossible complete increments; never clip them into fake motion.

    A rejection triggers zero-mean *bounded* reachable-motion uncertainty. The
    downstream estimate remains degraded until multiple plausible increments
    AND independent credible geometry updates have been seen. The caller must
    advance its raw odometry anchor even after rejection: otherwise a later
    delta reintroduces the accumulated bad jump.
    """

    def __init__(self, limits=None):
        self.limits = limits or MotionIntegrityLimits()
        self.degraded = False
        self.accepted_streak = 0
        self.geometry_streak = 0
        self.last_accepted = False
        self.last_reason = "initial"
        self.rejections = 0
        self._velocity = None
        self._previous_dt = None
        self._candidate_velocity = None
        self._candidate_streak = 0

    @property
    def status(self):
        if not self.degraded:
            return "tracking"
        return "recovering" if self.last_accepted else "odometry_rejected"

    def evaluate(self, raw_delta4, dt, covariance4):
        delta = np.asarray(raw_delta4, dtype=float)
        covariance = np.asarray(covariance4, dtype=float)
        dt = float(dt)
        if delta.shape != (4,) or covariance.shape != (4, 4):
            raise ValueError("Motion gate requires a four-vector and 4x4 covariance")
        if (
            not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T)
            or np.linalg.eigvalsh(covariance).min() < -1e-12
        ):
            raise ValueError("Motion covariance must be finite symmetric PSD")
        limits = self.limits
        interval = max(0.0, dt) if np.isfinite(dt) else 0.0
        radius = limits.max_speed_m_s * interval + limits.translation_tolerance_m
        angle = min(np.pi, limits.max_yaw_rate_rad_s * interval + limits.yaw_tolerance_rad)
        reason = "accepted"
        if not np.isfinite(delta).all() or not np.isfinite(dt) or dt <= 0:
            reason = "invalid_motion_or_timestamp"
        elif dt > limits.max_interval_s:
            reason = "odometry_gap"
        elif np.linalg.norm(delta[:3]) > radius:
            reason = "translation_exceeds_operating_envelope"
        elif abs(delta[3]) > angle:
            reason = "rotation_exceeds_operating_envelope"
        if limits.enable_acceleration:
            # Transport the previous velocity into the next body frame. Without
            # this step even constant world velocity appears to change on turns.
            valid = np.isfinite(delta).all() and np.isfinite(dt) and dt > 0
            if valid:
                velocity = delta[:3] / dt
                elapsed = 0.5 * (dt + (self._previous_dt or dt))
                threshold = limits.max_acceleration_m_s2 * elapsed + limits.velocity_tolerance_m_s
                if (
                    reason == "accepted"
                    and self._velocity is not None
                    and np.linalg.norm(velocity - self._velocity) > threshold
                ):
                    reason = "velocity_innovation_exceeds_operating_envelope"
                    consistent = (
                        self._candidate_velocity is not None
                        and np.linalg.norm(velocity - self._candidate_velocity) <= threshold
                    )
                    self._candidate_streak = self._candidate_streak + 1 if consistent else 1
                    self._candidate_velocity = velocity.copy()
                    # Re-anchor after consistent readings, but remain degraded
                    # until independent geometry also establishes recovery.
                    if self._candidate_streak >= limits.recovery_updates:
                        reason = "accepted"
                if reason == "accepted":
                    self._velocity = velocity.copy()
                    self._candidate_velocity = None
                    self._candidate_streak = 0
                elif reason != "velocity_innovation_exceeds_operating_envelope":
                    self._candidate_velocity = None
                    self._candidate_streak = 0
                c, sn = np.cos(delta[3]), np.sin(delta[3])
                transport = np.array([[c, sn, 0], [-sn, c, 0], [0, 0, 1]])
                if self._velocity is not None:
                    self._velocity = transport @ self._velocity
                if self._candidate_velocity is not None:
                    self._candidate_velocity = transport @ self._candidate_velocity
                self._previous_dt = dt
            else:
                self._velocity = self._candidate_velocity = self._previous_dt = None
                self._candidate_streak = 0
        accepted = reason == "accepted"
        self.last_accepted = accepted
        self.last_reason = reason
        if accepted:
            self.accepted_streak += 1
            return MotionDecision(True, delta.copy(), covariance.copy(), self.status, reason)
        self.degraded = True
        self.accepted_streak = self.geometry_streak = 0
        self.rejections += 1
        # Moments of a uniform 3D ball and independent bounded yaw uncertainty.
        fallback_covariance = np.diag([radius**2 / 5] * 3 + [angle**2 / 3])
        return MotionDecision(
            False, np.zeros(4), fallback_covariance, self.status, reason, radius, angle
        )

    def observe_geometry(self, credible):
        """Use rank/diversity/fit checks, NOT just ESS or inlier fraction."""
        if self.last_accepted and credible:
            self.geometry_streak += 1
        else:
            self.geometry_streak = 0
        needed = self.limits.recovery_updates
        if self.accepted_streak >= needed and self.geometry_streak >= needed:
            self.degraded = False
        return self.status


def apply_motion_decision(particles, decision, rng):
    """Pure array transition for offline/live replay; returns a new Nx4 array.

    Accepted decisions use the supplied Gaussian increment model. Rejected
    increments use uniform translation inside the declared reachable ball and
    bounded yaw, not unbounded Gaussian noise around the impossible reading.
    """
    state = np.asarray(particles, dtype=float)
    if state.ndim != 2 or state.shape[1] != 4 or not np.isfinite(state).all():
        raise ValueError("particles must be finite Nx4 states")
    count = len(state)
    if decision.accepted:
        motion = rng.multivariate_normal(
            decision.delta4, decision.covariance4, count, check_valid="raise"
        )
    else:
        direction = rng.normal(size=(count, 3))
        direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-15)
        radius = decision.translation_radius * np.cbrt(rng.random(count))
        motion = np.c_[
            direction * radius[:, None],
            rng.uniform(-decision.yaw_radius, decision.yaw_radius, count),
        ]
    out = state.copy()
    c, s = np.cos(state[:, 3]), np.sin(state[:, 3])
    out[:, 0] += c * motion[:, 0] - s * motion[:, 1]
    out[:, 1] += s * motion[:, 0] + c * motion[:, 1]
    out[:, 2] += motion[:, 2]
    out[:, 3] = (state[:, 3] + motion[:, 3] + np.pi) % (2 * np.pi) - np.pi
    return out
