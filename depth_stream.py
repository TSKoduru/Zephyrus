import depthai as dai
import cv2
import numpy as np
import time
import socket
import threading
import traceback
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# ---------------- Tunables ----------------
PORT = 8080
SCALE = 1.0
JPEG_QUALITY = 75
FPS = 35
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 5000
ENABLE_SPECKLE = False
ENABLE_SPATIAL = False
ENABLE_TEMPORAL = True
MEDIAN = dai.MedianFilter.KERNEL_5x5
# ------------------------------------------

latest_jpeg = None
frame_id = 0
cond = threading.Condition()
device_ready = False

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

class CamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def do_GET(self):
        global latest_jpeg, frame_id
        if self.path == '/':
            body = b'''<html><head><title>OAK Depth</title></head>
            <body style="background:#111;color:#eee;text-align:center;font-family:sans-serif">
            <h3>Jetson Orin Nano - RGB + Aligned Depth (Optimized 400p)</h3>
            <img src="/stream.mjpg" style="width:95%;max-width:1800px"/>
            </body></html>'''
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
            last = -1
            try:
                while True:
                    with cond:
                        cond.wait_for(lambda: latest_jpeg is not None and frame_id != last, timeout=2)
                        if frame_id == last or latest_jpeg is None: continue
                        data = latest_jpeg
                        last = frame_id
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n')
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError): pass
        else: self.send_error(404)

def build_pipeline():
    p = dai.Pipeline()
    camRgb = p.create(dai.node.ColorCamera)
    monoL = p.create(dai.node.MonoCamera)
    monoR = p.create(dai.node.MonoCamera)
    stereo = p.create(dai.node.StereoDepth)
    xRgb = p.create(dai.node.XLinkOut)
    xDepth = p.create(dai.node.XLinkOut)
    xRgb.setStreamName("rgb")
    xDepth.setStreamName("depth")

    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setFps(FPS)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setFps(FPS)

    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setIspScale(1, 3)
    camRgb.setInterleaved(False)
    camRgb.setFps(FPS)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setPostProcessingHardwareResources(3, 3)
    stereo.setOutputSize(640, 360)

    cfg = stereo.initialConfig.get()
    cfg.costMatching.confidenceThreshold = 255
    cfg.postProcessing.median = MEDIAN
    cfg.postProcessing.temporalFilter.enable = ENABLE_TEMPORAL
    cfg.postProcessing.temporalFilter.alpha = 0.4
    cfg.postProcessing.temporalFilter.delta = 20
    cfg.postProcessing.temporalFilter.persistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4
    cfg.postProcessing.spatialFilter.enable = ENABLE_SPATIAL
    cfg.postProcessing.spatialFilter.holeFillingRadius = 2
    cfg.postProcessing.spatialFilter.numIterations = 1
    cfg.postProcessing.spatialFilter.alpha = 0.5
    cfg.postProcessing.spatialFilter.delta = 20
    cfg.postProcessing.speckleFilter.enable = ENABLE_SPECKLE
    cfg.postProcessing.speckleFilter.speckleRange = 30
    cfg.postProcessing.thresholdFilter.minRange = DEPTH_MIN_MM
    cfg.postProcessing.thresholdFilter.maxRange = DEPTH_MAX_MM
    stereo.initialConfig.set(cfg)

    monoL.out.link(stereo.left)
    monoR.out.link(stereo.right)
    camRgb.isp.link(xRgb.input)
    stereo.depth.link(xDepth.input)
    return p

def _depth_worker_once():
    global latest_jpeg, frame_id, device_ready
    print("Starting Optimized 400p DepthAI pipeline...")
    try:
        with dai.Device(build_pipeline()) as device:
            q_rgb = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=1, blocking=False)
            rgb_frame = None
            t_last = time.time()
            fps = 0.0
            device_ready = True
            print("USB speed:", device.getUsbSpeed())
            print("USB speed:", device.getUsbSpeed())
            print("USB speed:", device.getUsbSpeed())
            print("Camera device linked and streaming frames.")

            while True:
                m = q_rgb.tryGet()
                if m is not None: rgb_frame = m.getCvFrame()
                d = q_depth.get()
                if rgb_frame is None: continue

                depth = d.getFrame()
                rgb = rgb_frame.copy()
                if rgb.shape[:2] != depth.shape[:2]:
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

                h, w = depth.shape
                cy, cx = h // 2, w // 2
                patch = depth[cy - 5:cy + 6, cx - 5:cx + 6]
                valid_patch = patch[patch > 0]
                center_m = float(np.median(valid_patch)) / 1000.0 if valid_patch.size > 10 else 0.0

                invalid = depth == 0
                valid_pct = 100.0 * (1.0 - invalid.mean())

                clipped = np.clip(depth, DEPTH_MIN_MM, DEPTH_MAX_MM)
                norm = ((clipped - DEPTH_MIN_MM) * (255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM))).astype(np.uint8)
                colored = cv2.applyColorMap(255 - norm, cv2.COLORMAP_JET)
                colored[invalid] = 0

                cv2.drawMarker(rgb, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.drawMarker(colored, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

                now = time.time()
                fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
                t_last = now

                txt = f"Center: {center_m:.2f} m" if center_m > 0 else "Center: no data"
                cv2.putText(colored, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(colored, f"{fps:.1f} FPS  valid {valid_pct:.0f}%", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                combined = np.hstack((rgb, colored))
                ok, buf = cv2.imencode(".jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ok:
                    with cond:
                        latest_jpeg = buf.tobytes()
                        frame_id += 1
    except Exception as e:
        print(f"Depth worker crash: {e}"); traceback.print_exc()
        device_ready = False

def depth_worker():
    global device_ready
    while True:
        _depth_worker_once()
        device_ready = False
        print("Camera stream ended, retrying in 3s...")
        time.sleep(3)

if __name__ == '__main__':
    # 1. Close open browsers on laptop before starting this to prevent lingering network polling
    t = threading.Thread(target=depth_worker, daemon=True)
    t.start()

    # 2. Wait up to 10 seconds for the camera hardware to hook up safely first
    print("Waiting for camera device to initialize...")
    for _ in range(20):
        if device_ready: break
        time.sleep(0.5)

    if not device_ready:
        print("CRITICAL: Camera hardware failed to start within timeout window.")

    server = ThreadedHTTPServer(('0.0.0.0', PORT), CamHandler)
    print(f"Server safely started at http://localhost:{PORT}")
    try: server.serve_forever()
    except KeyboardInterrupt: pass