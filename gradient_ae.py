"""
Gradient-based camera exposure control (Shim et al., IEEE TCSVT 2018).

Pipeline per frame:
  1. Downsample image (paper: 320x240).
  2. Gamma-map to 7 anchor images, score each with the gradient-information
     metric M (Eq. 1), fit a 5th-order polynomial over gamma, take argmax -> gamma_hat (Eq. 2).
  3. Convert gamma_hat to a new exposure with the linear (Eq. 3) or nonlinear (Eq. 4) update.
  4. Optional multi-camera balancing against overlapping neighbours (Eq. 5, 6).
  5. Split exposure into shutter/gain (shutter first, then gain).

Assumes a camera with a LINEAR response (as in the paper).
Only depends on numpy + opencv-python.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
@dataclass
class Params:
    # Eq. 1
    delta: float = 0.06            # noise activation threshold
    lam: float = 1e3               # mapping tendency (lambda)
    # Eq. 2
    anchors: Tuple[float, ...] = (0.6, 0.8, 1.0, 1.2, 1.4)
    gamma_range: Tuple[float, float] = (0.6, 1.4)
    poly_order: int = 5
    work_size: Tuple[int, int] = (320, 240)   # (w, h)
    # Eq. 3 / 4
    nonlinear: bool = True
    kp: float = 0.2
    d: float = 0.75
    # Exposure limits, in "exposure units" E = shutter_ms * linear_gain
    e_min: float = 0.05
    e_max: float = 1000.0
    # Shutter/gain split
    max_shutter_ms: float = 25.5   # paper: 25.51 ms single-cam, 49 ms multi-cam
    min_shutter_ms: float = 0.02
    max_gain: float = 30.0
    # Sobel normalisation: a 0->1 step edge gives |Gx| = 4 with the 3x3 Sobel kernel.
    # The paper only says "normalised to [0,1]"; a FIXED constant is required so that
    # gamma-mapped anchors remain comparable. Tune if your sensor is noisy.
    sobel_norm: float = 4.0


# --------------------------------------------------------------------------- #
# Eq. 1: gradient information metric
# --------------------------------------------------------------------------- #
def to_gray01(img: np.ndarray) -> np.ndarray:
    """uint8/uint16/float image (gray or BGR) -> float32 gray in [0, 1]."""
    if img.ndim == 3:
        if img.dtype != np.uint8:
            img = np.clip(img.astype(np.float32) / (65535.0 if img.dtype == np.uint16 else 1.0), 0, 1)
            img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.dtype == np.uint8:
        return img.astype(np.float32) / 255.0
    if img.dtype == np.uint16:
        return img.astype(np.float32) / 65535.0
    return np.clip(img.astype(np.float32), 0.0, 1.0)


def gradient_magnitude(gray01: np.ndarray, norm: float = 4.0) -> np.ndarray:
    gx = cv2.Sobel(gray01, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray01, cv2.CV_32F, 0, 1, ksize=3)
    return np.clip(np.sqrt(gx * gx + gy * gy) / norm, 0.0, 1.0)


def gradient_information(gray01: np.ndarray, p: Params, mask: Optional[np.ndarray] = None) -> float:
    """M = sum_i mbar_i  (Eq. 1). `mask` restricts the sum to an ROI (e.g. ground region)."""
    m = gradient_magnitude(gray01, p.sobel_norm)
    n = np.log(p.lam * (1.0 - p.delta) + 1.0)
    mbar = np.where(m >= p.delta, np.log(p.lam * np.maximum(m - p.delta, 0.0) + 1.0) / n, 0.0)
    if mask is not None:
        mbar = mbar[mask]
    return float(mbar.sum())


# --------------------------------------------------------------------------- #
# Eq. 2: gamma estimation
# --------------------------------------------------------------------------- #
def estimate_gamma(img: np.ndarray, p: Params, mask: Optional[np.ndarray] = None) -> Tuple[float, np.ndarray]:
    """Returns (gamma_hat, anchor_scores). gamma<1 brightens (I^gamma), gamma>1 darkens."""
    gray = cv2.resize(to_gray01(img), p.work_size, interpolation=cv2.INTER_AREA)
    if mask is not None:
        mask = cv2.resize(mask.astype(np.uint8), p.work_size, interpolation=cv2.INTER_NEAREST).astype(bool)

    anchors = np.asarray(p.anchors, dtype=np.float64)
    scores = np.array([gradient_information(np.power(gray, g).astype(np.float32), p, mask) for g in anchors])

    order = min(p.poly_order, len(anchors) - 1)
    # scale scores for conditioning; argmax is unaffected
    s = scores / (scores.max() + 1e-9)
    coef = np.polyfit(anchors, s, order)
    grid = np.linspace(p.gamma_range[0], p.gamma_range[1], 400)
    gamma_hat = float(grid[np.argmax(np.polyval(coef, grid))])
    return gamma_hat, scores


# --------------------------------------------------------------------------- #
# Eq. 3 / 4: update functions
# --------------------------------------------------------------------------- #
def _alpha_gamma(gamma_hat: float) -> float:
    return 0.5 if gamma_hat >= 1.0 else 1.0


def linear_update(E: float, gamma_hat: float, kp: float) -> float:
    return (1.0 + _alpha_gamma(gamma_hat) * kp * (1.0 - gamma_hat)) * E


def nonlinear_update(E: float, gamma_hat: float, kp: float, d: float) -> float:
    a = np.arctan(1.0 / d)
    # (2 - g)*a - a == (1 - g)*a
    R = d * np.tan((1.0 - gamma_hat) * a) + 1.0
    return (1.0 + _alpha_gamma(gamma_hat) * kp * (R - 1.0)) * E


# --------------------------------------------------------------------------- #
# Eq. 5 / 6: multi-camera balancing
# --------------------------------------------------------------------------- #
def patch_ratio(means_i: Sequence[float], means_j: Sequence[float],
                eps: float = 1e-3, sat_hi: float = 0.95, sat_lo: float = 0.005) -> Optional[float]:
    """
    r^{ij} = median_p( mean(p^j) / (mean(p^i) + eps) ) over corresponding overlap patches.
    Inputs are per-patch mean intensities in [0,1] (linear). Saturated patches are dropped
    since their ratios are meaningless. Returns None if nothing usable.
    """
    a, b = np.asarray(means_i, float), np.asarray(means_j, float)
    ok = (a < sat_hi) & (b < sat_hi) & (a > sat_lo) & (b > sat_lo)
    if not ok.any():
        return None
    return float(np.median(b[ok] / (a[ok] + eps)))


def balance_exposure(E_cur: float, E_new: float, ratios: Sequence[float]) -> float:
    """
    Closed-form minimiser of  alpha*(X-E_new)^2 + (1-alpha)/N * sum_j (X - r_ij*E_cur)^2
        X = alpha*E_new + (1-alpha) * mean_j(r_ij) * E_cur
    alpha from Eq. 6 using R = E_new/E_cur clipped to [0.5, 2].
    r_ij * E_cur is the exposure that would make camera i as bright as neighbour j.
    """
    ratios = [r for r in ratios if r is not None]
    if not ratios:
        return E_new
    R = float(np.clip(E_new / E_cur, 0.5, 2.0))
    alpha = (1.0 - R) + 0.5 if R < 1.0 else R / 2.0
    return alpha * E_new + (1.0 - alpha) * float(np.mean(ratios)) * E_cur


# --------------------------------------------------------------------------- #
# Exposure -> shutter / gain
# --------------------------------------------------------------------------- #
def split_exposure(E: float, p: Params) -> Tuple[float, float]:
    """Raise shutter first up to max_shutter_ms, then gain (paper's strategy)."""
    E = float(np.clip(E, p.e_min, p.e_max))
    if E <= p.max_shutter_ms:
        return max(E, p.min_shutter_ms), 1.0
    return p.max_shutter_ms, min(E / p.max_shutter_ms, p.max_gain)


def effective_exposure(shutter_ms: float, gain: float) -> float:
    return shutter_ms * gain


# --------------------------------------------------------------------------- #
# Controller
# --------------------------------------------------------------------------- #
@dataclass
class StepInfo:
    gamma_hat: float
    E_old: float
    E_unbalanced: float
    E_new: float
    shutter_ms: float
    gain: float


class GradientExposureController:
    """One instance per camera. Call step() every frame."""

    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()

    def step(self, img: np.ndarray, E_cur: float,
             neighbor_ratios: Optional[Sequence[float]] = None,
             roi_mask: Optional[np.ndarray] = None) -> StepInfo:
        p = self.p
        g, _ = estimate_gamma(img, p, roi_mask)
        E_new = (nonlinear_update(E_cur, g, p.kp, p.d) if p.nonlinear
                 else linear_update(E_cur, g, p.kp))
        E_unbal = E_new
        if neighbor_ratios:
            E_new = balance_exposure(E_cur, E_new, neighbor_ratios)
        E_new = float(np.clip(E_new, p.e_min, p.e_max))
        sh, gain = split_exposure(E_new, p)
        return StepInfo(g, E_cur, E_unbal, E_new, sh, gain)


# --------------------------------------------------------------------------- #
# Camera glue
# --------------------------------------------------------------------------- #
class CameraBase:
    """Implement these three for your driver (V4L2, Spinnaker, Argus, ROS topic, ...)."""
    def get_image(self) -> np.ndarray: raise NotImplementedError
    def get_exposure(self) -> Tuple[float, float]: raise NotImplementedError   # (shutter_ms, gain)
    def set_exposure(self, shutter_ms: float, gain: float) -> None: raise NotImplementedError


def run_loop(cam: CameraBase, ctrl: GradientExposureController, n_frames: int = 10**9,
             neighbor_fn=None, roi_mask=None, callback=None):
    """Algorithm 1. neighbor_fn(img) -> list of r^{ij} for this camera (or None)."""
    for _ in range(n_frames):
        sh, gain = cam.get_exposure()
        img = cam.get_image()
        ratios = neighbor_fn(img) if neighbor_fn else None
        info = ctrl.step(img, effective_exposure(sh, gain), ratios, roi_mask)
        cam.set_exposure(info.shutter_ms, info.gain)
        if callback:
            callback(info, img)
