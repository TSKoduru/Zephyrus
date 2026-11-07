"""Rectified depth packet in metres, aligned to the supplied optical camera."""

from dataclasses import dataclass
import numpy as np


@dataclass
class DepthFrame:
    timestamp: float
    depth_m: np.ndarray
    K: np.ndarray
    T_body_camera: np.ndarray

    @property
    def valid(self):
        return np.isfinite(self.depth_m) & (self.depth_m > 0)

    @property
    def timestamp_s(self):
        return self.timestamp
