"""Local CPU checks; no ROS, drone connection, or sensor hardware required."""
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

MODULE = Path(__file__).resolve().parents[1] / 'course_localization/recording.py'
spec = importlib.util.spec_from_file_location('course_recording', MODULE)
recording = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recording)
RunRecorder = recording.RunRecorder


class RecorderTests(unittest.TestCase):
    def test_roundtrip_metadata_images_and_nonfinite(self):
        with tempfile.TemporaryDirectory() as root:
            config = Path(root) / 'settings.yaml'
            config.write_text('sigma: 0.18\n')
            recorder = RunRecorder(root, metadata={'test': True}, config_paths={'pf': config}, map_identity={'sha256': 'abc'})
            depth = np.array([[1., np.nan]], np.float32)
            arrays = {'depth_raw': depth, 'K': np.eye(3), 'left_image': np.ones((2, 3), np.uint8), 'weights': np.array([.2, .8])}
            self.assertTrue(recorder.record_frame(arrays, {'exposure_ns': 123, 'range_m': np.inf, 'imu': {'age': np.float32(np.nan)}}))
            depth[:] = 9
            self.assertEqual(recorder.close()['written'], 1)
            with np.load(recorder.path / 'frames/000000000.npz', allow_pickle=False) as frame:
                self.assertEqual(frame['depth_raw'][0, 0], 1)
                self.assertTrue(np.isnan(frame['depth_raw'][0, 1]))
                self.assertEqual(frame['left_image'].shape, (2, 3))
                self.assertEqual(json.loads(str(frame['recorder_diagnostics_json']))['range_m'], None)
            manifest = json.loads((recorder.path / 'manifest.json').read_text())
            self.assertEqual(manifest['map_identity']['sha256'], 'abc')
            archived = manifest['configs']['pf']
            self.assertEqual((recorder.path / archived['archive']).read_bytes(), config.read_bytes())
            self.assertEqual(len(archived['sha256']), 64)
            events = [json.loads(line) for line in (recorder.path / 'diagnostics.jsonl').read_text().splitlines()]
            self.assertEqual(events[0]['diagnostics']['exposure_ns'], 123)
            self.assertIsNone(events[0]['diagnostics']['imu']['age'])
            with self.assertRaises(RuntimeError):
                recorder.record_frame({})
            recorder.close()  # idempotent
            other = RunRecorder(root)
            other.close()
            self.assertNotEqual(other.path, recorder.path)

    def test_queue_overflow_visible_and_close_drains(self):
        entered, release = threading.Event(), threading.Event()
        original = RunRecorder._write_frame
        def stalled(self, item, log):
            entered.set()
            if not release.wait(5):
                raise RuntimeError('test timeout')
            original(self, item, log)
        with tempfile.TemporaryDirectory() as root, patch.object(RunRecorder, '_write_frame', stalled):
            recorder = RunRecorder(root, queue_size=1)
            self.assertTrue(recorder.record_frame({'depth': np.zeros((1, 1))}))
            self.assertTrue(entered.wait(2))
            self.assertTrue(recorder.record_frame({}))
            self.assertFalse(recorder.record_frame({}))
            self.assertEqual(recorder.health['dropped'], 1)
            release.set()
            health = recorder.close()
            self.assertEqual((health['submitted'], health['written'], health['dropped']), (3, 2, 1))
            saved = json.loads((recorder.path / 'final_health.json').read_text())
            self.assertEqual(saved['dropped'], 1)
            self.assertTrue(saved['closed'])

    def test_cloud_transform_bound_and_provenance(self):
        with tempfile.TemporaryDirectory() as root:
            recorder = RunRecorder(root, max_voxels=1, voxel_size=.1)
            vo, pf = np.eye(4), np.eye(4)
            vo[0, 3], pf[0, 3] = 10, 20
            points = np.array([[0., 0., 0.], [.02, 0, 0], [2, 0, 0], [np.nan, 0, 0]])
            recorder.record_frame({}, points_body=points, T_map_body_vo=vo, T_map_body_pf=pf)
            recorder.close()
            with np.load(recorder.path / 'points/000000000.npz') as chunk:
                self.assertEqual(len(chunk['points_body']), 4)
                self.assertEqual(len(chunk['points_vo']), 3)
            for name, expected in [('vo', 10.01), ('pf', 20.01)]:
                stem = recorder.path / 'snapshots' / ('000000_' + name)
                with np.load(str(stem) + '.npz') as cloud:
                    self.assertEqual(cloud['points'].shape, (1, 3))
                    self.assertAlmostEqual(cloud['points'][0, 0], expected)
                    self.assertEqual(cloud['counts'][0], 2)
                metadata = json.loads(Path(str(stem) + '.json').read_text())
                self.assertEqual(metadata['rejected_new_voxel_points'], 1)
                ply = Path(str(stem) + '.ply').read_text()
                self.assertIn('element vertex 1', ply)
                if name == 'pf':
                    self.assertIn('Circular', ply)
                    self.assertFalse(metadata['independent_of_pf'])
                else:
                    self.assertTrue(metadata['independent_of_pf'])

    def test_write_failure_is_visible(self):
        with tempfile.TemporaryDirectory() as root, patch.object(RunRecorder, '_write_frame', side_effect=OSError('disk full')):
            recorder = RunRecorder(root)
            recorder.record_frame({})
            with self.assertRaisesRegex(RuntimeError, 'disk full'):
                recorder.close()
            self.assertEqual(recorder.health['failed'], 1)
            self.assertIn('disk full', recorder.health['error'])

    def test_oversize_and_invalid_packets(self):
        with tempfile.TemporaryDirectory() as root:
            recorder = RunRecorder(root, max_frame_bytes=8)
            self.assertFalse(recorder.record_frame({'large': np.zeros(20)}))
            with self.assertRaises(ValueError):
                recorder.record_frame({'object': np.array([{}])})
            with self.assertRaises(ValueError):
                recorder.record_frame({}, T_map_body_vo=np.zeros((4, 4)))
            recorder.close()
            self.assertEqual(recorder.health['dropped'], 1)

    def test_atomic_write_never_overwrites(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'metadata.json'
            recording._save_json(path, {'first': True})
            with self.assertRaises(FileExistsError):
                recording._save_json(path, {'first': False})
            self.assertTrue(json.loads(path.read_text())['first'])
            self.assertEqual(len(list(Path(root).iterdir())), 1)


if __name__ == '__main__':
    unittest.main()
