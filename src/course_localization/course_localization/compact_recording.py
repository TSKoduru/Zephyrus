"""Bounded flight diagnostics, JPEG/depth PNG previews and measured PLY maps.

No NPZ, rosbag, raw image history or duplicate point chunks. Retention applies
only inside this recorder's newly created run directory; older runs are untouched.
"""
from collections import deque
import os
import threading
import time

import cv2
import numpy as np

from .recording import RunRecorder, _atomic, _json, _save_json


class RollingText:
    """Two bounded text files; errors propagate instead of silently losing logs."""
    def __init__(self, path, max_bytes=4 * 1024 * 1024):
        self.path, self.max_bytes = path, max_bytes
        self.stream = path.open('x')
        self.lock = threading.Lock()
        self.error = None

    def write(self, text):
        with self.lock:
            try:
                if len(text.encode()) > self.max_bytes:
                    raise ValueError('Diagnostic entry exceeds log size limit')
                if self.stream.tell() + len(text.encode()) > self.max_bytes:
                    self.stream.close()
                    os.replace(self.path, self.path.with_suffix(self.path.suffix + '.1'))
                    self.stream = self.path.open('w')
                self.stream.write(text)
                self.stream.flush()
            except Exception as exc:
                self.error = str(exc)
                raise

    def flush(self):
        self.stream.flush()

    def close(self):
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class CompactRecorder(RunRecorder):
    def __init__(self, root, **kwargs):
        self.preview_time = -float('inf')
        self.previews = deque()
        self.snapshots = deque()
        self.events = self.transitions = None
        self.last_transition = None
        super().__init__(root, snapshot_every=50, max_voxels=50000, **kwargs)
        (self.path / 'previews').mkdir()
        self.events = RollingText(self.path / 'events.jsonl')
        self.transitions = RollingText(self.path / 'mission_transitions.jsonl')
        _save_json(self.path / 'storage_policy.json', dict(
            format='compact; no NPZ or rosbag', preview_hz=1, retained_preview_sets=60,
            preview_dimensions_max=[320, 200], retained_maps=2, map_voxels_max=50000,
            text_files_max_bytes_each=4 * 1024 * 1024, text_files_backups=1,
            retention='Only this run; older runs are never deleted',
            map='Measured stereo points accumulated with cuVSLAM; not ground truth'))

    def event(self, topic, value):
        line = _json(dict(unix_ns=time.time_ns(), topic=topic, data=value)) + '\n'
        self.events.write(line)
        if topic == '/odom_test/mission_status':
            key = _json({k:value.get(k) for k in ('stage', 'fault', 'gates', 'enabled')})
            if key != self.last_transition:
                self.transitions.write(line)
                self.last_transition = key

    def _write_frame(self, item, log):
        seq, arrays, diagnostics, points, transforms, enqueue_ns = item
        if points is not None:
            finite = points[np.isfinite(points).all(axis=1)]
            for name, matrix in transforms.items():
                self._accumulate(name, finite @ matrix[:3, :3].T + matrix[:3, 3])
        stamp = float(arrays['timestamp']) if 'timestamp' in arrays else None
        if stamp is not None and stamp - self.preview_time >= 1.:
            paths = []
            for key in ('left_rectified', 'right_rectified', 'depth_m'):
                if key not in arrays:
                    continue
                pixels = arrays[key]
                depth = key == 'depth_m'
                if depth:
                    pixels = np.where(np.isfinite(pixels) & (pixels > 0),
                                      np.clip(pixels * 1000, 1, 65535), 0).astype(np.uint16)
                if pixels.shape[0] > 200 or pixels.shape[1] > 320:
                    scale = min(200 / pixels.shape[0], 320 / pixels.shape[1])
                    pixels = cv2.resize(pixels, (int(pixels.shape[1]*scale), int(pixels.shape[0]*scale)),
                                        interpolation=cv2.INTER_NEAREST if depth else cv2.INTER_AREA)
                ext = '.png' if depth else '.jpg'
                ok, encoded = cv2.imencode(ext, pixels, [] if depth else [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok:
                    raise RuntimeError('Preview encoding failed')
                path = self.path / 'previews' / f'{seq:09d}_{key}{ext}'
                _atomic(path, lambda f, data=encoded: f.write(data.tobytes()))
                paths.append(path)
            self.previews.append(paths)
            while len(self.previews) > 60:
                for path in self.previews.popleft():
                    path.unlink(missing_ok=True)
            self.preview_time = stamp
        with self._lock:
            self._stats['written'] += 1
        # Keep exact depth calibration and poses; image previews are display-only.
        measurements = {k:v for k,v in arrays.items() if k not in
                        ('left_rectified','right_rectified','depth_m')}
        log.write(_json(dict(sequence=seq, timestamp=stamp, measurements=measurements,
                             diagnostics=diagnostics, enqueue_unix_ns=enqueue_ns)) + '\n')
        if self._stats['written'] % self._snapshot_every == 0:
            self._snapshot()

    def _snapshot(self):
        index = self._snapshot_index
        self._snapshot_index += 1
        paths = []
        for name, cloud in self._clouds.items():
            if not cloud:
                continue
            xyz = np.array([total/count for total,count in cloud.values()], dtype='<f4').reshape(-1,3)
            path = self.path / 'snapshots' / f'{index:06d}_{name}.ply'
            header = ('ply\nformat binary_little_endian 1.0\ncomment frame odom; measured stereo, cuVSLAM alignment\n'
                      f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode()
            _atomic(path, lambda f: f.write(header + xyz.tobytes()))
            paths.append(path)
        self.snapshots.append(paths)
        while len(self.snapshots) > 2:
            for path in self.snapshots.popleft():
                path.unlink(missing_ok=True)

    def _run(self):
        try:
            with RollingText(self.path / 'diagnostics.jsonl') as log:
                while True:
                    item = self._queue.get()
                    try:
                        if item is None:
                            break
                        if self._stats['error']:
                            with self._lock:
                                self._stats['failed'] += 1
                            continue
                        try:
                            self._write_frame(item, log)
                        except Exception as exc:
                            with self._lock:
                                self._stats['error'] = repr(exc)
                                self._stats['failed'] += 1
                    finally:
                        self._queue.task_done()
                if not self._stats['error']:
                    self._snapshot()
        except Exception as exc:
            with self._lock:
                self._stats['error'] = repr(exc)
        finally:
            with self._lock:
                self._stats['closed'] = True
            try:
                _save_json(self.path / 'final_health.json', self.health)
            except Exception as exc:
                with self._lock:
                    self._stats['error'] = repr(exc)

    def close(self):
        try:
            return super().close()
        finally:
            for stream in (self.events, self.transitions):
                if stream is not None:
                    stream.close()
