"""Measured scalar range and nominal-floor likelihood. Mount and beam calibration belong to hardware configuration."""

from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def cone_rays(full_fov_deg=27.0):
    directions = [[0.0, 0.0, 1.0]]
    weights = [1.0]
    for fraction, count, weight in [(0.5, 6, 0.8), (1.0, 12, 0.3)]:
        angle = np.deg2rad(full_fov_deg * 0.5 * fraction)
        for azimuth in np.arange(count) * 2 * np.pi / count:
            directions.append(
                [np.sin(angle) * np.cos(azimuth), np.sin(angle) * np.sin(azimuth), np.cos(angle)]
            )
            weights.append(weight)
    return (np.asarray(directions), np.asarray(weights))


def weighted_median(values, weights):
    order = np.argsort(values)
    values = np.asarray(values)[order]
    weights = np.asarray(weights)[order]
    return float(values[np.searchsorted(np.cumsum(weights), weights.sum() / 2)])


@dataclass(frozen=True)
class RangeMeasurement:
    timestamp: float
    range_m: float
    valid: bool
    status: str
    variance_m2: float
    field_of_view_rad: float = np.deg2rad(27.0)
    min_range_m: float = 0.04
    max_range_m: float = 4.0


class FloorRangeLikelihood:
    """Floor-range mixture; input poses are hypotheses, attitude is measured.

    Geometry: body->site origin[0,0,-.04], site+Z points body-Z. Yaw has no
    effect for an unbounded horizontal nominal floor. Unexpected short returns
    can be physical clutter; broad outliers handle false/long returns. Bias is
    marginalized into sigma, not estimated from truth. No height is overwritten.
    """

    def __init__(
        self,
        site_position=(0.0, 0.0, -0.04),
        site_rotation=None,
        floor_z=0.0,
        floor_sigma_m=0.02,
        short_probability=0.25,
        outlier_probability=0.05,
        full_fov_deg=27.0,
    ):
        self.site_position = np.asarray(site_position, float)
        self.site_rotation = (
            np.diag([1.0, -1.0, -1.0])
            if site_rotation is None
            else np.asarray(site_rotation, float)
        )
        if self.site_position.shape != (3,) or self.site_rotation.shape != (3, 3):
            raise ValueError("Invalid range mount extrinsic")
        if (
            floor_sigma_m < 0
            or min(short_probability, outlier_probability) < 0
            or short_probability + outlier_probability >= 1
        ):
            raise ValueError("Invalid mixture")
        self.floor_z = float(floor_z)
        self.floor_sigma_m = float(floor_sigma_m)
        self.short_probability = float(short_probability)
        self.outlier_probability = float(outlier_probability)
        self.rays, self.weights = cone_rays(full_fov_deg)

    def expected_ranges(self, poses, roll_pitch):
        poses = np.asarray(poses)
        rotation = Rotation.from_euler("xyz", [*roll_pitch, 0.0]).as_matrix()
        origin_z = poses[:, 2] + (rotation @ self.site_position)[2]
        dz = (self.rays @ self.site_rotation.T @ rotation.T)[:, 2]
        valid = dz < -0.2
        if self.weights[valid].sum() < 0.5 * self.weights.sum():
            return np.full(len(poses), np.nan)
        factor = weighted_median(-1 / dz[valid], self.weights[valid])
        return (origin_z - self.floor_z) * factor

    def log_likelihood(self, poses, measurement, roll_pitch=(0.0, 0.0)):
        if not measurement.valid or not np.isfinite(measurement.range_m):
            return np.zeros(len(poses))
        expected = self.expected_ranges(poses, roll_pitch)
        sigma = np.sqrt(max(measurement.variance_m2, 0) + self.floor_sigma_m**2)
        sigma = max(sigma, 0.005)
        observed = measurement.range_m
        hit = np.exp(-0.5 * ((observed - expected) / sigma) ** 2) / (np.sqrt(2 * np.pi) * sigma)
        short = np.where(
            (observed >= measurement.min_range_m) & (observed < expected),
            1 / np.maximum(expected - measurement.min_range_m, 0.01),
            0.0,
        )
        uniform = 1 / (measurement.max_range_m - measurement.min_range_m)
        likelihood = (
            (1 - self.short_probability - self.outlier_probability) * hit
            + self.short_probability * short
            + self.outlier_probability * uniform
        )
        result = np.log(np.maximum(likelihood, 1e-12))
        result[~np.isfinite(expected) | (expected <= 0)] = np.log(
            self.outlier_probability * uniform
        )
        if not np.isfinite(expected).any():
            return np.zeros(len(poses))
        return result
