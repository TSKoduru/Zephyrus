"""ROS-free asynchronous run archives. No truth input or automatic map replacement.

Call record_frame with named numeric arrays (raw/reduced depth, K, extrinsics,
VO/PF poses, covariance, particles, weights, optional images) and diagnostics
(exposure/processing timestamps, range/IMU validity, tracking status).
points_body must be metric body-frame points. T_map_body_vo must align VO to the
configured initial map frame, NOT to the evolving PF estimate. PF-aligned clouds
are circular map-conditioned reconstructions, not independent map validation.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np


def _safe(value):
    if isinstance(value, np.ndarray):
        return _safe(value.tolist())
    if isinstance(value, np.generic):
        return _safe(value.item())
    if isinstance(value, dict):
        return {str(k): _safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _json(value):
    return json.dumps(_safe(value), allow_nan=False, sort_keys=True)


def _atomic(path, write):
    """Publish only complete files; callers use unique immutable output names."""
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with tmp.open('xb') as stream:
            write(stream)
            stream.flush()
            os.fsync(stream.fileno())
        # Hard-link publishes without replacing an existing file.
        os.link(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def _save_json(path, value):
    _atomic(path, lambda f: f.write((_json(value) + '\n').encode()))


class RunRecorder:
    """Bounded queue; record_frame returns False on drops or writer failure.

    health is authoritative even if disk failure prevents writing failure logs.
    close drains accepted frames and raises on any I/O failure. Queue overload
    drops the newest frame, retains exact missing sequence IDs via gaps, and
    records cumulative counters in JSONL and final health. Arrays are copied at
    enqueue so later sensor-buffer reuse cannot change archived observations.
    """
    def __init__(self, root, metadata=None, config_paths=None, map_identity=None,
                 queue_size=16, voxel_size=.05, max_voxels=200000,
                 snapshot_every=100, max_frame_bytes=64 * 1024 * 1024):
        if queue_size < 1 or voxel_size <= 0 or not math.isfinite(voxel_size) or max_voxels < 1 or snapshot_every < 1 or max_frame_bytes < 1:
            raise ValueError('Recorder bounds must be positive and finite')
        self.path = Path(root) / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '_' + uuid.uuid4().hex[:12])
        self.path.mkdir(parents=True, exist_ok=False)
        for name in ('frames', 'points', 'snapshots', 'configs'):
            (self.path / name).mkdir()
        configs = {}
        for index, (name, filename) in enumerate((config_paths or {}).items()):
            data = Path(filename).read_bytes()
            copy = f'configs/{index:03d}_{Path(filename).name}'
            _atomic(self.path / copy, lambda f, data=data: f.write(data))
            configs[str(name)] = {'source': str(filename), 'archive': copy, 'sha256': hashlib.sha256(data).hexdigest()}
        self.provenance = {
            'vo': {'frame': 'configured_initial_map', 'alignment': 'fixed initial-map-to-VO alignment supplied by caller', 'independent_of_pf': True},
            'pf': {'frame': 'configured_map', 'alignment': 'PF pose supplied by caller', 'independent_of_pf': False,
                   'warning': 'Circular: PF localization uses the nominal map. Not independent evidence that the map is correct.'}}
        _save_json(self.path / 'manifest.json', {
            'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'metadata': metadata or {}, 'configs': configs, 'map_identity': map_identity or {},
            'clouds': self.provenance, 'automatic_map_replacement': False,
            'nonfinite_json': 'null; numeric NPZ arrays preserve original values',
            'voxel_size_m': voxel_size, 'max_voxels_per_cloud': max_voxels,
            'queue_size': queue_size, 'max_frame_bytes': max_frame_bytes})
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._closing = False
        self._stats = dict(submitted=0, accepted=0, written=0, dropped=0, dropped_queue_full=0, dropped_oversize=0, dropped_writer_failed=0, failed=0, error=None, closed=False)
        self._clouds = {'vo': {}, 'pf': {}}
        self._cloud_rejected = {'vo': 0, 'pf': 0}
        self._voxel_size, self._max_voxels = voxel_size, max_voxels
        self._snapshot_every, self._max_frame_bytes = snapshot_every, max_frame_bytes
        self._snapshot_index = 0
        self._worker = threading.Thread(target=self._run, name='course-run-recorder', daemon=True)
        self._worker.start()

    @property
    def health(self):
        with self._lock:
            return dict(self._stats, queue_depth=self._queue.qsize(), closing=self._closing,
                        unarchived=self._stats['accepted'] - self._stats['written'])

    def record_frame(self, arrays, diagnostics=None, *, points_body=None,
                     T_map_body_vo=None, T_map_body_pf=None):
        payload = {}
        for name, value in arrays.items():
            if not isinstance(name, str) or not name.isidentifier() or name.startswith('recorder_'):
                raise ValueError('Array names must be identifiers outside recorder_ namespace')
            array = np.asarray(value)
            if array.dtype.kind not in 'biufcSU':
                raise ValueError('Only non-object numeric/string arrays are supported')
            payload[name] = array
        points = None if points_body is None else np.asarray(points_body, dtype=float)
        if points is not None and (points.ndim != 2 or points.shape[1] != 3):
            raise ValueError('points_body must have shape (N,3)')
        transforms = {}
        for name, matrix in [('vo', T_map_body_vo), ('pf', T_map_body_pf)]:
            if matrix is not None:
                matrix = np.asarray(matrix, dtype=float)
                if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
                    raise ValueError('Cloud transform must be a finite homogeneous 4x4 matrix')
                transforms[name] = matrix.copy()
        diagnostic_json = _json(diagnostics or {})
        diagnostics = json.loads(diagnostic_json)
        size = sum(a.nbytes for a in payload.values()) + (0 if points is None else points.nbytes) + len(diagnostic_json.encode())
        with self._lock:
            if self._closing:
                raise RuntimeError('Recorder is closing/closed')
            seq = self._stats['submitted']
            self._stats['submitted'] += 1
            if self._stats['error'] or size > self._max_frame_bytes or self._queue.full():
                self._stats['dropped'] += 1
                reason = 'writer_failed' if self._stats['error'] else ('oversize' if size > self._max_frame_bytes else 'queue_full')
                self._stats['dropped_' + reason] += 1
                return False
            payload = {k: v.copy() for k, v in payload.items()}
            item = (seq, payload, diagnostics, None if points is None else points.copy(), transforms, time.time_ns())
            self._queue.put_nowait(item)
            self._stats['accepted'] += 1
        return True

    def _accumulate(self, name, points):
        cloud = self._clouds[name]
        for point in points:
            key = tuple(np.floor(point / self._voxel_size).astype(np.int64))
            if key in cloud:
                cloud[key][0] += point
                cloud[key][1] += 1
            elif len(cloud) < self._max_voxels:
                cloud[key] = [point.copy(), 1]
            else:
                self._cloud_rejected[name] += 1

    def _write_frame(self, item, log):
        seq, arrays, diagnostics, points, transforms, enqueue_ns = item
        stem = f'{seq:09d}'
        arrays.update(recorder_sequence=np.array(seq), recorder_enqueue_unix_ns=np.array(enqueue_ns),
                      recorder_diagnostics_json=np.array(_json(diagnostics)))
        _atomic(self.path / 'frames' / (stem + '.npz'), lambda f: np.savez_compressed(f, **arrays))
        if points is not None:
            finite = points[np.isfinite(points).all(axis=1)]
            chunk = {'points_body': points}
            for name, matrix in transforms.items():
                aligned = finite @ matrix[:3, :3].T + matrix[:3, 3]
                chunk['T_map_body_' + name] = matrix
                chunk['points_' + name] = aligned
                self._accumulate(name, aligned)
            _atomic(self.path / 'points' / (stem + '.npz'), lambda f: np.savez_compressed(f, **chunk))
        with self._lock:
            self._stats['written'] += 1
        log.write(_json({'event': 'frame', 'sequence': seq, 'diagnostics': diagnostics,
                         'enqueue_unix_ns': enqueue_ns, 'written_unix_ns': time.time_ns(), 'health': self.health}) + '\n')
        log.flush()
        if self._stats['written'] % self._snapshot_every == 0:
            self._snapshot()

    def _snapshot(self):
        index = self._snapshot_index
        self._snapshot_index += 1
        for name, cloud in self._clouds.items():
            xyz = np.array([total / count for total, count in cloud.values()], dtype=float).reshape(-1, 3)
            counts = np.array([count for _, count in cloud.values()], dtype=np.int64)
            stem = f'{index:06d}_{name}'
            base = self.path / 'snapshots'
            provenance = dict(self.provenance[name], voxel_size_m=self._voxel_size,
                              unique_voxels=len(cloud), rejected_new_voxel_points=self._cloud_rejected[name],
                              accumulation='centroid per voxel; new voxels rejected at capacity; raw chunks retained')
            _atomic(base / (stem + '.npz'), lambda f: np.savez_compressed(f, points=xyz, counts=counts, provenance_json=np.array(_json(provenance))))
            def ply(f):
                header = ('ply\nformat ascii 1.0\ncomment provenance ' + _json(provenance) + '\n'
                          f'element vertex {len(xyz)}\nproperty double x\nproperty double y\nproperty double z\nend_header\n')
                f.write(header.encode())
                np.savetxt(f, xyz, fmt='%.9g')
            _atomic(base / (stem + '.ply'), ply)
            _save_json(base / (stem + '.json'), provenance)

    def _run(self):
        try:
            with (self.path / 'diagnostics.jsonl').open('x') as log:
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
                                self._stats['failed'] += 1
                                self._stats['error'] = repr(exc)
                            log.write(_json({'event': 'write_failure', 'sequence': item[0], 'health': self.health}) + '\n')
                            log.flush()
                    finally:
                        self._queue.task_done()
                if not self._stats['error']:
                    self._snapshot()
                log.write(_json({'event': 'final_health', 'health': self.health}) + '\n')
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
        with self._lock:
            first = not self._closing
            self._closing = True
        if first:
            while self._worker.is_alive():
                try:
                    self._queue.put(None, timeout=.1)
                    break
                except queue.Full:
                    pass
        self._worker.join()
        if self.health['error']:
            raise RuntimeError('Recording failed: ' + self.health['error'])
        return self.health

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
