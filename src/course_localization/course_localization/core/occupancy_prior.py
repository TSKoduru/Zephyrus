"""Soft known-map free-space prior for a level cylindrical drone envelope.

Uses nominal broad surfaces only. Never sees route, truth, image labels, timber
supports, or randomized scene geometry. Sampling and fabric uncertainty are
explicit; ceiling clearance uses actual body height rather than a sphere. The cylinder
assumes a level body; arbitrary vehicle tilt requires a different envelope.
"""

from pathlib import Path
import numpy as np
import hashlib, json, os, tempfile
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.special import logsumexp
from .course_map import StructuralMap
from .inertial_aid import KeyframeInertialParticleFilter


class CylinderOccupancyPrior:
    def __init__(self, map_directory, cache_directory, geometry_sigma=0.05):
        self.spacing = 0.02
        self.origin = np.array([-3.4, -3.4, -0.3])
        self.shape = np.array([341, 341, 181])
        self.geometry_sigma = float(geometry_sigma)
        # Cache identity includes every source mesh and all construction choices.
        sources = sorted(Path(map_directory).glob("*.obj"))
        parameters = {
            "spacing": self.spacing,
            "origin": self.origin.tolist(),
            "shape": self.shape.tolist(),
            "surface_spacing": 0.015,
            "below": 0.096,
            "above": 0.05,
            "version": 1,
        }
        digest = hashlib.sha256(json.dumps(parameters, sort_keys=True).encode())
        for source in sources:
            digest.update(source.name.encode())
            digest.update(source.read_bytes())
        self.geometry_digest = digest.hexdigest()
        cache = Path(cache_directory) / f"cylinder_clearance_{self.geometry_digest[:20]}.npz"
        cache.parent.mkdir(parents=True, exist_ok=True)
        if cache.exists():
            self.distance = np.load(cache)["distance"].astype(np.float32)
        else:
            surface = StructuralMap.from_assets(map_directory, spacing=0.015, include_floor=False)
            grid = np.zeros(self.shape, bool)
            idx = np.rint((surface.points - self.origin) / self.spacing).astype(int)
            idx = idx[np.all((idx >= 0) & (idx < self.shape), axis=1)]
            grid[tuple(idx.T)] = True
            self.distance = np.full(self.shape, 8.0, dtype=np.float32)
            for z in range(self.shape[2]):
                lo = max(0, int(np.ceil(z - 0.096 / self.spacing)))
                hi = min(self.shape[2], int(np.floor(z + 0.05 / self.spacing)) + 1)
                slab = grid[:, :, lo:hi].any(axis=2)
                if slab.any():
                    self.distance[:, :, z] = distance_transform_edt(~slab) * self.spacing
            with tempfile.NamedTemporaryFile(
                dir=cache.parent, suffix=".npz", delete=False
            ) as temporary:
                temporary_path = Path(temporary.name)
            try:
                np.savez_compressed(temporary_path, distance=self.distance)
                os.replace(temporary_path, cache)
            finally:
                temporary_path.unlink(missing_ok=True)

    def penalties(self, poses):
        coord = (poses[:, :3] - self.origin) / self.spacing
        distance = map_coordinates(self.distance, coord.T, order=1, mode="constant", cval=8.0)
        # Three-centimetre radial allowance accommodates map/gridding error;
        # additional Gaussian sigma makes support uncertain, not a hard wall.
        wall_pen = np.maximum(0, 0.29 - distance) / self.geometry_sigma
        floor_pen = np.maximum(0, 0.096 - poses[:, 2]) / 0.02
        cage_pen = np.maximum(0, np.max(abs(poses[:, :2]), axis=1) - (3.048 - 0.30)) / 0.03
        return np.maximum.reduce([wall_pen, floor_pen, cage_pen])

    def log_likelihood(self, poses):
        penalty = self.penalties(poses)
        return np.log(0.02 + 0.98 * np.exp(-0.5 * penalty**2))


class OccupancyParticleFilter(KeyframeInertialParticleFilter):
    def __init__(self, *args, occupancy_prior, **kwargs):
        self.occupancy_prior = occupancy_prior
        super().__init__(*args, **kwargs)
        self.occupancy_failed = False

    def update(self, frame, roll_pitch=(0.0, 0.0)):
        assimilate = (
            self.measurement_count == 0
            or self.force_measurement
            or np.linalg.norm(self.view_translation) >= self.keyframe_translation
            or abs(self.view_yaw) >= self.keyframe_yaw
        )
        if assimilate and len(self._sample_depth(np.asarray(frame.depth_m))):
            penalties = self.occupancy_prior.penalties(self.particles)
            self.occupancy_failed = bool(np.all(penalties > 3.0))
            self.log_weights += self.occupancy_prior.log_likelihood(self.particles)
            self.log_weights -= logsumexp(self.log_weights)
            self._posterior = None
        result = super().update(frame, roll_pitch)
        if self.occupancy_failed:
            result["health"] = "no_free_space_support"
        result["occupancy_failed"] = self.occupancy_failed
        return result
