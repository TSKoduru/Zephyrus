"""
OakGradientAE: drop-in gradient-based auto-exposure for any DepthAI v3 camera node.
(Shim et al., IEEE TCSVT 2018; core algorithm lives in gradient_ae.py)

    import depthai as dai
    from oak_ae import OakGradientAE

    ae = OakGradientAE(max_shutter_ms=2.7, max_iso=1600)

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        q = cam.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=30).createOutputQueue()
        ae.attach(cam)                  # must be before pipeline.start()
        pipeline.start()
        while pipeline.isRunning():
            frame = q.get()
            ae.update(frame)            # once per frame; does nothing most frames
            img = frame.getCvFrame()    # ...your pipeline

Behaviour (paper's Algorithm 1): for the first `settle_frames` frames the camera's own AE runs, then its
settled exposure is read back as the starting point and the controller takes over in manual mode.

Notes
  * The algorithm assumes a LINEAR image. Mono streams are close to linear; for colour pass decode_gamma=2.2.
  * Exposure E = shutter_ms * ISO/100. Shutter rises to max_shutter_ms first, then ISO.
  * Cameras apply a control ~2-3 frames late, hence update_every=3.
  * One OakGradientAE per camera. For stereo, create one per mono camera (independent control).
"""
from __future__ import annotations

from typing import Optional, Tuple, Union

import cv2
import numpy as np

from gradient_ae import GradientExposureController, Params, StepInfo, estimate_gamma, split_exposure

try:
    import depthai as dai
except ImportError:  # allows unit tests without depthai
    dai = None


def _meta_exposure(frame) -> Tuple[Optional[float], Optional[int]]:
    """(exposure_ms, iso) the camera reports for this frame, or (None, None)."""
    try:
        t = frame.getExposureTime()
        ms = t.total_seconds() * 1000.0 if hasattr(t, "total_seconds") else float(t) / 1000.0
        return ms, int(frame.getSensitivity())
    except Exception:
        return None, None


class OakGradientAE:
    def __init__(self,
                 max_shutter_ms: float = 2.7,
                 max_iso: int = 1600,
                 min_shutter_us: int = 20,
                 decode_gamma: float = 1.0,
                 update_every: int = 3,
                 settle_frames: int = 45,
                 kp: float = 0.3,
                 d: float = 0.75,
                 nonlinear: bool = True,
                 roi: Optional[Tuple[float, float, float, float]] = None,
                 params: Optional[Params] = None):
        """
        roi: (x0, y0, x1, y1) as fractions of the frame; only that region is scored. None = full frame.
        params: full Params override (anchors, delta, lam, ...). Shutter/ISO limits are filled in for you.
        """
        gain_max = max_iso / 100.0
        self.p = params or Params(kp=kp, d=d, nonlinear=nonlinear)
        self.p.max_shutter_ms = max_shutter_ms
        self.p.max_gain = gain_max
        self.p.e_min = min_shutter_us / 1000.0
        self.p.e_max = max_shutter_ms * gain_max
        self.max_iso = int(max_iso)
        self.min_shutter_us = int(min_shutter_us)
        self.update_every = max(1, int(update_every))
        self.settle_frames = int(settle_frames)
        self.roi = roi
        self._lut = ((np.arange(256) / 255.0) ** decode_gamma).astype(np.float32)
        self._ctrl = GradientExposureController(self.p)
        self._cq = None
        self._mask = None
        self.reset()

    # ------------------------------------------------------------------ setup
    def attach(self, cam_node) -> "OakGradientAE":
        """Create the control input on a dai.node.Camera. Call before pipeline.start()."""
        self._cq = cam_node.inputControl.createInputQueue()
        return self

    def reset(self) -> None:
        """Go back to the startup phase (built-in AE settle, then manual control)."""
        self._frame_i = 0
        self._phase = "settle"        # settle -> control | frozen | builtin
        self._E = 10.0
        self.gamma_hat = 1.0
        self.shutter_us = 0
        self.iso = 0
        self.last: Optional[StepInfo] = None

    # --------------------------------------------------------------- controls
    @property
    def exposure(self) -> float:
        """Current exposure value E = shutter_ms * ISO/100."""
        return self._E

    @property
    def mode(self) -> str:
        return {"settle": "settling (camera AE)", "control": "gradient AE",
                "frozen": "frozen (manual)", "builtin": "camera AE"}[self._phase]

    def enable(self) -> None:
        self._phase = "control"; self._send_manual(self._E)

    def freeze(self) -> None:
        """Hold the current exposure (manual, no updates)."""
        self._phase = "frozen"; self._send_manual(self._E)

    def use_builtin(self) -> None:
        """Hand exposure back to the camera's AE (same shutter/ISO caps)."""
        self._phase = "builtin"; self._send_auto()

    def set_exposure(self, shutter_ms: float, iso: int = 100) -> None:
        """Manually set exposure; the controller continues from here if enabled."""
        self._E = float(np.clip(shutter_ms * iso / 100.0, self.p.e_min, self.p.e_max))
        if self._phase in ("control", "frozen"):
            self._send_manual(self._E)

    # ------------------------------------------------------------------- main
    def update(self, frame: Union["dai.ImgFrame", np.ndarray]) -> Optional[StepInfo]:
        """Call once per frame with the ImgFrame (preferred) or a numpy image. Returns StepInfo on frames
        where exposure was updated, else None."""
        self._frame_i += 1
        is_msg = hasattr(frame, "getCvFrame")
        ms, iso = _meta_exposure(frame) if is_msg else (None, None)

        if self._phase == "settle":
            if self._frame_i == 1:
                self._send_auto()
            if self._frame_i >= self.settle_frames:
                if ms is not None:
                    self._E = float(np.clip(ms * iso / 100.0, self.p.e_min, self.p.e_max))
                self._phase = "control"
                self._send_manual(self._E)
            return None

        if self._phase == "builtin":
            if ms is not None:
                self._E = float(np.clip(ms * iso / 100.0, self.p.e_min, self.p.e_max))
            return None

        img = frame.getCvFrame() if is_msg else frame
        g8 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        lin = self._lut[g8] if g8.dtype == np.uint8 else np.clip(g8.astype(np.float32), 0, 1)
        if self.roi is not None and self._mask is None:
            h, w = lin.shape
            x0, y0, x1, y1 = self.roi
            self._mask = np.zeros((h, w), bool)
            self._mask[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)] = True

        if self._phase == "frozen" or self._frame_i % self.update_every:
            # still track gamma_hat for display/logging, without commanding anything
            if self._frame_i % self.update_every == 0:
                self.gamma_hat, _ = estimate_gamma(lin, self.p, self._mask)
            return None

        info = self._ctrl.step(lin, self._E, roi_mask=self._mask)
        self._E = info.E_new
        self.gamma_hat = info.gamma_hat
        self._send_manual(self._E)
        self.last = info
        return info

    # --------------------------------------------------------------- internals
    def _send_manual(self, E: float) -> None:
        sh_ms, gain = split_exposure(E, self.p)
        self.shutter_us = int(np.clip(sh_ms * 1000, self.min_shutter_us, 33000))
        self.iso = int(np.clip(round(gain * 100), 100, self.max_iso))
        if self._cq is None:
            return
        c = dai.CameraControl()
        c.setManualExposure(self.shutter_us, self.iso)
        self._cq.send(c)

    def _send_auto(self) -> None:
        if self._cq is None:
            return
        c = dai.CameraControl()
        c.setAutoExposureEnable()
        c.setAutoExposureLimit(int(self.p.max_shutter_ms * 1000))
        try:
            c.setAutoExposureMaxISO(self.max_iso)
        except Exception:
            pass
        self._cq.send(c)
