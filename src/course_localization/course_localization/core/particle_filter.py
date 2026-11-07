"""Depth-to-structure particle localization, independent of simulation/truth.

State: world x,y,z and yaw [metres, radians]. Depth is optical-axis depth;
T_body_camera transforms optical camera coordinates into body coordinates.
"""

from __future__ import annotations
import numpy as np
from scipy.special import logsumexp


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


class ParticleFilter:
    def __init__(
        self,
        structural_map,
        initial_mean4,
        initial_std4,
        num_particles=800,
        seed=0,
        *,
        sigma_hit=0.12,
        outlier_probability=0.20,
        max_depth_samples=72,
        effective_rays=12.0,
        min_depth=0.10,
        max_depth=6.0,
    ):
        self.map = structural_map
        self.rng = np.random.default_rng(seed)
        mean = np.asarray(initial_mean4, dtype=float)
        std = np.asarray(initial_std4, dtype=float)
        if (
            mean.shape != (4,)
            or std.shape != (4,)
            or not np.isfinite(mean).all()
            or not np.isfinite(std).all()
            or np.any(std < 0)
        ):
            raise ValueError("Initial mean/std must be finite four-vectors with nonnegative std")
        if (
            num_particles < 2
            or sigma_hit <= 0
            or not 0 < outlier_probability < 1
            or max_depth_samples < 1
            or effective_rays <= 0
        ):
            raise ValueError("Invalid filter configuration")
        self.n = int(num_particles)
        self.particles = mean + self.rng.normal(size=(self.n, 4)) * std
        self.particles[:, 3] = wrap_angle(self.particles[:, 3])
        self.log_weights = np.full(self.n, -np.log(self.n))
        self.sigma_hit = float(sigma_hit)
        self.outlier_probability = float(outlier_probability)
        self.max_depth_samples = int(max_depth_samples)
        self.effective_rays = float(effective_rays)
        self.min_depth, self.max_depth = float(min_depth), float(max_depth)
        self.inlier_fraction = 0.0
        self.measurement_valid = False
        self.resample_count = 0
        self.last_update_ess = float(self.n)
        self.last_sample_count = 0
        self._posterior = None

    @property
    def weights(self):
        return np.exp(self.log_weights)

    def predict(self, delta_body4, covariance4x4):
        delta = np.asarray(delta_body4, dtype=float)
        covariance = np.asarray(covariance4x4, dtype=float)
        if (
            delta.shape != (4,)
            or covariance.shape != (4, 4)
            or not np.isfinite(delta).all()
            or not np.isfinite(covariance).all()
        ):
            raise ValueError("Motion needs a finite four-vector and 4x4 covariance")
        if (
            not np.allclose(covariance, covariance.T)
            or np.linalg.eigvalsh(covariance).min() < -1e-12
        ):
            raise ValueError("Motion covariance must be symmetric positive semidefinite")
        motion = self.rng.multivariate_normal(delta, covariance, self.n, check_valid="raise")
        yaw = self.particles[:, 3]
        c, s = np.cos(yaw), np.sin(yaw)
        self.particles[:, 0] += c * motion[:, 0] - s * motion[:, 1]
        self.particles[:, 1] += s * motion[:, 0] + c * motion[:, 1]
        self.particles[:, 2] += motion[:, 2]
        self.particles[:, 3] = wrap_angle(yaw + motion[:, 3])
        self._posterior = None

    def _sample_depth(self, depth):
        """One valid pixel nearest each tile centre; no simulator segmentation."""
        h, w = depth.shape
        cols = min(self.max_depth_samples, max(1, int(np.sqrt(self.max_depth_samples * w / h))))
        rows = max(1, self.max_depth_samples // cols)
        samples = []
        for r in range(rows):
            y0, y1 = r * h // rows, (r + 1) * h // rows
            for c in range(cols):
                x0, x1 = c * w // cols, (c + 1) * w // cols
                tile = depth[y0:y1, x0:x1]
                valid = np.isfinite(tile) & (tile >= self.min_depth) & (tile <= self.max_depth)
                yy, xx = np.nonzero(valid)
                if len(yy):
                    k = np.argmin((yy - (y1 - y0 - 1) / 2) ** 2 + (xx - (x1 - x0 - 1) / 2) ** 2)
                    y, x = int(yy[k] + y0), int(xx[k] + x0)
                    samples.append((x, y, depth[y, x]))
        return np.asarray(samples, dtype=float).reshape(-1, 3)

    def update(self, frame, roll_pitch=(0.0, 0.0)):
        depth = np.asarray(frame.depth_m, dtype=float)
        K = np.asarray(frame.K, dtype=float)
        extrinsic = np.asarray(frame.T_body_camera, dtype=float)
        if (
            depth.ndim != 2
            or min(depth.shape) == 0
            or K.shape != (3, 3)
            or extrinsic.shape != (4, 4)
        ):
            raise ValueError("Invalid depth, calibration or camera transform dimensions")
        if (
            not np.isfinite(K).all()
            or not np.isfinite(extrinsic).all()
            or K[0, 0] <= 0
            or K[1, 1] <= 0
        ):
            raise ValueError("Calibration must be finite with positive focal lengths")
        attitude = np.asarray(roll_pitch, dtype=float)
        if attitude.shape != (2,) or not np.isfinite(attitude).all():
            raise ValueError("roll_pitch must contain two finite radians")
        samples = self._sample_depth(depth)
        self.last_sample_count = len(samples)
        if not len(samples):
            self.measurement_valid = False
            self._posterior = None
            return self.estimate()
        pixel = np.c_[samples[:, :2], np.ones(len(samples))]
        camera = (pixel @ np.linalg.inv(K).T) * samples[:, 2, None]
        body = camera @ extrinsic[:3, :3].T + extrinsic[:3, 3]
        roll, pitch = attitude
        cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        body = body @ (Ry @ Rx).T
        yaw = self.particles[:, 3]
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        points = np.empty((self.n, len(body), 3))
        points[:, :, 0] = c * body[:, 0] - s * body[:, 1] + self.particles[:, 0, None]
        points[:, :, 1] = s * body[:, 0] + c * body[:, 1] + self.particles[:, 1, None]
        points[:, :, 2] = body[:, 2] + self.particles[:, 2, None]
        distances = np.asarray(self.map.distance(points.reshape(-1, 3))).reshape(self.n, -1)
        if not np.isfinite(distances).all() or np.any(distances < 0):
            raise ValueError("Structural map must return finite nonnegative distances")
        hit = np.exp(-0.5 * (distances / self.sigma_hit) ** 2)
        mixture = (1 - self.outlier_probability) * hit + self.outlier_probability
        # A robust pseudo-likelihood, tempered to avoid treating correlated
        # depth pixels as independent. Not a calibrated stereo noise model.
        log_likelihood = np.log(mixture).sum(axis=1) * min(1.0, self.effective_rays / len(body))
        self.log_weights += log_likelihood
        self.log_weights -= logsumexp(self.log_weights)
        self.measurement_valid = True
        self.inlier_fraction = float(self.weights @ np.mean(distances < 2 * self.sigma_hit, axis=1))
        self.last_update_ess = float(1 / np.sum(self.weights**2))
        self._posterior = self._summarize()
        if self.last_update_ess < self.n / 2:
            self._resample()
        return self.estimate()

    def _resample(self):
        # Stratified resampling, with independent offsets in each stratum.
        positions = (np.arange(self.n) + self.rng.random(self.n)) / self.n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices].copy()
        self.log_weights.fill(-np.log(self.n))
        self.resample_count += 1

    def _summarize(self):
        weights = self.weights
        mean = weights @ self.particles
        cs = weights @ np.cos(self.particles[:, 3])
        sn = weights @ np.sin(self.particles[:, 3])
        mean[3] = np.arctan2(sn, cs)
        residual = self.particles - mean
        residual[:, 3] = wrap_angle(residual[:, 3])
        covariance = (residual * weights[:, None]).T @ residual
        ess = float(1 / np.sum(weights**2))
        if not self.measurement_valid:
            health = "prediction_only"
        elif self.inlier_fraction < 0.25:
            health = "poor_match"
        elif np.sqrt(np.trace(covariance[:3, :3])) > 0.6 or np.hypot(cs, sn) < 0.7:
            health = "uncertain"
        else:
            health = "tracking"
        return {
            "pose": mean,
            "pose4": mean.copy(),
            "covariance": covariance,
            "ess": ess,
            "health": health,
            "inlier_fraction": self.inlier_fraction,
            "depth_samples": self.last_sample_count,
            "resample_count": self.resample_count,
        }

    def estimate(self):
        # Preserve weighted statistics before resampling removes weight detail.
        result = self._posterior if self._posterior is not None else self._summarize()
        output = {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in result.items()}
        output["resample_count"] = self.resample_count
        return output
