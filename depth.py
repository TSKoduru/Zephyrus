#!/usr/bin/env python3
"""OAK depth on Jetson: OakDepthCamera (capture), DepthMeasure (distances), MJPEGStreamer (browser view)."""
import argparse
import socket
import threading
import time
import traceback
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import List, Optional, Tuple

import cv2
import depthai as dai
import numpy as np

Roi = Tuple[int, int, int, int]  # x0, y0, x1, y1 in pixels


# =====================================================================
# Camera
# =====================================================================
@dataclass
class CameraConfig:
    fps: int = 35
    rgb_scale: Tuple[int, int] = (1, 3)      # 1080p * 1/3 = 640x360
    out_size: Tuple[int, int] = (640, 360)   # depth output, must match the RGB size
    min_mm: int = 100
    max_mm: int = 5000
    confidence: int = 255                    # 255 = accept everything, lower = stricter
    median: str = "5x5"                      # off | 3x3 | 5x5 | 7x7
    temporal: bool = True
    spatial: bool = False
    speckle: bool = False
    extended: bool = True                    # closer minimum range (~35 cm)
    subpixel: bool = False                   # can't be combined with extended


@dataclass
class DepthFrame:
    rgb: np.ndarray      # HxWx3 BGR
    depth: np.ndarray    # HxW uint16, millimeters, 0 = invalid, aligned to rgb
    seq: int
    timestamp: float


class OakDepthCamera:
    """Runs the DepthAI pipeline in a background thread and auto-reconnects.
    Use start(), then wait_for_frame(). Frames are independent copies, safe to draw on."""

    _MEDIAN = {
        "off": dai.MedianFilter.MEDIAN_OFF,
        "3x3": dai.MedianFilter.KERNEL_3x3,
        "5x5": dai.MedianFilter.KERNEL_5x5,
        "7x7": dai.MedianFilter.KERNEL_7x7,
    }

    def __init__(self, cfg: Optional[CameraConfig] = None):
        self.cfg = cfg or CameraConfig()
        self.connected = False
        self.fps = 0.0
        self.usb_speed: Optional[str] = None
        self.intrinsics: Optional[np.ndarray] = None   # 3x3, for the RGB-aligned depth size
        self._cond = threading.Condition()
        self._frame: Optional[DepthFrame] = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ---- lifecycle ----
    def start(self, wait: float = 10.0) -> bool:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        t0 = time.time()
        while not self.connected and time.time() - t0 < wait:
            time.sleep(0.1)
        return self.connected

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # ---- consuming frames ----
    def wait_for_frame(self, last_seq: int = -1, timeout: float = 2.0) -> Optional[DepthFrame]:
        """Block until a frame newer than last_seq exists. Returns None on timeout."""
        with self._cond:
            self._cond.wait_for(lambda: self._frame is not None and self._frame.seq != last_seq,
                                timeout=timeout)
            if self._frame is None or self._frame.seq == last_seq:
                return None
            return self._frame

    def latest(self) -> Optional[DepthFrame]:
        with self._cond:
            return self._frame

    # ---- internals ----
    def _build_pipeline(self) -> dai.Pipeline:
        c = self.cfg
        p = dai.Pipeline()
        camRgb = p.create(dai.node.ColorCamera)
        monoL = p.create(dai.node.MonoCamera)
        monoR = p.create(dai.node.MonoCamera)
        stereo = p.create(dai.node.StereoDepth)
        xRgb = p.create(dai.node.XLinkOut)
        xDepth = p.create(dai.node.XLinkOut)
        xRgb.setStreamName("rgb")
        xDepth.setStreamName("depth")

        res = dai.MonoCameraProperties.SensorResolution.THE_400_P
        monoL.setResolution(res)
        monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoL.setFps(c.fps)
        monoR.setResolution(res)
        monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoR.setFps(c.fps)

        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setIspScale(*c.rgb_scale)
        camRgb.setInterleaved(False)
        camRgb.setFps(c.fps)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(c.subpixel)
        stereo.setExtendedDisparity(c.extended)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setPostProcessingHardwareResources(3, 3)
        stereo.setOutputSize(*c.out_size)

        cfg = stereo.initialConfig.get()
        cfg.costMatching.confidenceThreshold = c.confidence
        cfg.postProcessing.median = self._MEDIAN[c.median]
        tf = cfg.postProcessing.temporalFilter
        tf.enable = c.temporal
        tf.alpha = 0.4
        tf.delta = 20
        tf.persistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4
        cfg.postProcessing.spatialFilter.enable = c.spatial
        cfg.postProcessing.spatialFilter.holeFillingRadius = 2
        cfg.postProcessing.spatialFilter.numIterations = 1
        cfg.postProcessing.spatialFilter.alpha = 0.5
        cfg.postProcessing.spatialFilter.delta = 20
        cfg.postProcessing.speckleFilter.enable = c.speckle
        cfg.postProcessing.speckleFilter.speckleRange = 30
        cfg.postProcessing.thresholdFilter.minRange = c.min_mm
        cfg.postProcessing.thresholdFilter.maxRange = c.max_mm
        stereo.initialConfig.set(cfg)

        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)
        camRgb.isp.link(xRgb.input)
        stereo.depth.link(xDepth.input)
        return p

    def _run(self):
        while not self._stop.is_set():
            try:
                self._run_once()
            except Exception as e:
                print(f"[camera] error: {e}")
                traceback.print_exc()
            self.connected = False
            if not self._stop.is_set():
                print("[camera] retrying in 3s...")
                self._stop.wait(3)

    def _run_once(self):
        w, h = self.cfg.out_size
        with dai.Device(self._build_pipeline()) as device:
            self.usb_speed = str(device.getUsbSpeed())
            try:
                calib = device.readCalibration()
                self.intrinsics = np.array(
                    calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, w, h), dtype=np.float64)
            except Exception:
                self.intrinsics = None

            q_rgb = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=1, blocking=False)
            rgb = None
            t_last = time.time()
            self.connected = True
            print(f"[camera] streaming, USB speed: {self.usb_speed}")

            while not self._stop.is_set():
                m = q_rgb.tryGet()
                if m is not None:
                    rgb = m.getCvFrame()
                d = q_depth.tryGet()
                if d is None:
                    time.sleep(0.002)
                    continue
                if rgb is None:
                    continue

                depth = d.getFrame()
                if rgb.shape[:2] != depth.shape[:2]:
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]),
                                       interpolation=cv2.INTER_NEAREST)

                now = time.time()
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
                t_last = now

                with self._cond:
                    self._seq += 1
                    self._frame = DepthFrame(rgb.copy(), depth, self._seq, now)
                    self._cond.notify_all()


# =====================================================================
# Measurement
# =====================================================================
class DepthMeasure:
    """Distance queries on a depth frame (uint16 mm, 0 = invalid). All results in meters,
    or None when there isn't enough valid data. Treat None as 'unknown', not 'free'."""

    def __init__(self, min_mm: int = 100, max_mm: int = 5000,
                 intrinsics: Optional[np.ndarray] = None):
        self.min_mm = min_mm
        self.max_mm = max_mm
        self.K = intrinsics

    def _valid(self, depth: np.ndarray, roi: Optional[Roi] = None) -> np.ndarray:
        if roi is not None:
            h, w = depth.shape
            x0, y0, x1, y1 = roi
            depth = depth[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        return depth[(depth >= self.min_mm) & (depth <= self.max_mm)]

    def distance_at(self, depth: np.ndarray, x: int, y: int, patch: int = 11,
                    min_pixels: int = 10) -> Optional[float]:
        """Median of the valid pixels in a patch around (x, y)."""
        r = patch // 2
        v = self._valid(depth, (x - r, y - r, x + r + 1, y + r + 1))
        return float(np.median(v)) / 1000.0 if v.size >= min_pixels else None

    def distance_roi(self, depth: np.ndarray, roi: Roi, percentile: float = 50,
                     min_pixels: int = 20) -> Optional[float]:
        v = self._valid(depth, roi)
        return float(np.percentile(v, percentile)) / 1000.0 if v.size >= min_pixels else None

    def nearest(self, depth: np.ndarray, roi: Optional[Roi] = None, percentile: float = 5,
                min_pixels: int = 50) -> Optional[float]:
        """Robust nearest obstacle distance (low percentile instead of min, to ignore speckle)."""
        v = self._valid(depth, roi)
        return float(np.percentile(v, percentile)) / 1000.0 if v.size >= min_pixels else None

    def sector_distances(self, depth: np.ndarray, n: int = 3,
                         rows: Tuple[float, float] = (0.3, 0.7)) -> List[Optional[float]]:
        """Nearest distance in n vertical sectors (left to right) over a horizontal band.
        Good for simple left / center / right obstacle avoidance."""
        h, w = depth.shape
        y0, y1 = int(h * rows[0]), int(h * rows[1])
        return [self.nearest(depth, (int(w * i / n), y0, int(w * (i + 1) / n), y1))
                for i in range(n)]

    def valid_ratio(self, depth: np.ndarray, roi: Optional[Roi] = None) -> float:
        total = depth.size if roi is None else max(1, self._crop_size(depth, roi))
        return self._valid(depth, roi).size / total

    @staticmethod
    def _crop_size(depth: np.ndarray, roi: Roi) -> int:
        h, w = depth.shape
        x0, y0, x1, y1 = roi
        return max(0, min(h, y1) - max(0, y0)) * max(0, min(w, x1) - max(0, x0))

    def to_xyz(self, depth: np.ndarray, x: int, y: int, patch: int = 5
               ) -> Optional[Tuple[float, float, float]]:
        """Pixel to a 3D point in the camera frame (meters; x right, y down, z forward)."""
        if self.K is None:
            return None
        z = self.distance_at(depth, x, y, patch)
        if z is None:
            return None
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        return ((x - cx) * z / fx, (y - cy) * z / fy, z)

    def colorize(self, depth: np.ndarray) -> np.ndarray:
        """Display only. Near = red, far = blue, invalid = black."""
        clipped = np.clip(depth, self.min_mm, self.max_mm)
        norm = ((clipped - self.min_mm) * (255.0 / (self.max_mm - self.min_mm))).astype(np.uint8)
        colored = cv2.applyColorMap(255 - norm, cv2.COLORMAP_JET)
        colored[depth == 0] = 0
        return colored


# =====================================================================
# Streaming
# =====================================================================
class _Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        s: "MJPEGStreamer" = self.server.streamer
        if self.path == '/':
            body = (f'<html><head><title>{s.title}</title></head>'
                    f'<body style="background:#111;color:#eee;text-align:center;font-family:sans-serif">'
                    f'<h3>{s.title}</h3>'
                    f'<img src="/stream.mjpg" style="width:95%;max-width:1800px"/></body></html>'
                    ).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            s._client_delta(+1)
            last = -1
            try:
                while s._running:
                    data, last = s._next(last)
                    if data is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n')
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                s._client_delta(-1)
        else:
            self.send_error(404)


class MJPEGStreamer:
    """Serve BGR frames as an MJPEG stream. Call update(frame) from your loop.
    JPEG encoding is skipped while no browser is connected, which saves CPU on the Jetson."""

    def __init__(self, port: int = 8080, quality: int = 75, scale: float = 1.0,
                 title: str = "Jetson Orin Nano - RGB + Aligned Depth"):
        self.port = port
        self.quality = quality
        self.scale = scale
        self.title = title
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._seq = 0
        self._clients = 0
        self._running = False
        self._server: Optional[_Server] = None

    @property
    def clients(self) -> int:
        return self._clients

    def start(self):
        self._running = True
        self._server = _Server(("0.0.0.0", self.port), _Handler)
        self._server.streamer = self
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def stop(self):
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._server:
            self._server.shutdown()
            self._server.server_close()

    def update(self, frame: np.ndarray):
        if self._clients <= 0:
            return
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if ok:
            with self._cond:
                self._jpeg = buf.tobytes()
                self._seq += 1
                self._cond.notify_all()

    def urls(self) -> List[str]:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = "127.0.0.1"
        finally:
            s.close()
        return [f"http://{socket.gethostname()}.local:{self.port}", f"http://{ip}:{self.port}"]

    # internals used by the handler
    def _next(self, last: int):
        with self._cond:
            self._cond.wait_for(lambda: self._jpeg is not None and self._seq != last, timeout=2)
            if self._jpeg is None or self._seq == last:
                return None, last
            return self._jpeg, self._seq

    def _client_delta(self, d: int):
        with self._cond:
            self._clients += d