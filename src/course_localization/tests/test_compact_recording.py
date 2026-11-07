"""Check disk retention and preserved map evidence without ROS or aircraft."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from course_localization.compact_recording import CompactRecorder, RollingText


class CompactRecordingTest(unittest.TestCase):
    def test_rotation_keeps_bounded_recent_text(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'events.jsonl'
            with RollingText(path, max_bytes=64) as log:
                for i in range(100):
                    log.write(json.dumps({'sequence':i}) + '\n')
            files = list(Path(tmp).glob('*'))
            self.assertEqual(len(files), 2)
            self.assertTrue(all(p.stat().st_size <= 64 for p in files))
            self.assertEqual(json.loads(path.read_text().splitlines()[-1])['sequence'], 99)

    def test_no_npz_bounded_previews_and_map_accumulates(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = CompactRecorder(tmp)
            rng = np.random.default_rng(7)
            for i in range(105):
                arrays = dict(timestamp=float(i), depth_m=np.full((100,160),2.,np.float32),
                              left_rectified=rng.integers(0,255,(400,640),dtype=np.uint8),
                              right_rectified=rng.integers(0,255,(400,640),dtype=np.uint8),
                              K=np.eye(3),T_body_camera=np.eye(4),T_odom_body=np.eye(4))
                self.assertTrue(recorder.record_frame(arrays, {}, points_body=np.array([[i*.1,0,2]]),
                                                     T_map_body_vo=np.eye(4)))
                recorder._queue.join()
                recorder.event('/odom_test/mission_status',dict(stage='UP' if i<50 else 'STOPPED',fault=''))
            recorder.close()
            self.assertFalse(list(recorder.path.rglob('*.npz')))
            self.assertFalse((recorder.path/'rosbag').exists())
            previews = list((recorder.path/'previews').glob('*'))
            self.assertEqual(len(previews), 180)
            self.assertFalse(any(p.name.startswith('000000000_') for p in previews))
            self.assertEqual(len(list((recorder.path/'snapshots').glob('*.ply'))),2)
            latest = sorted((recorder.path/'snapshots').glob('*.ply'))[-1].read_bytes()
            self.assertIn(b'element vertex 105\n',latest)
            depth = cv2.imread(str(recorder.path/'previews/000000104_depth_m.png'),cv2.IMREAD_UNCHANGED)
            self.assertEqual(depth.dtype,np.uint16)
            self.assertTrue(np.all(depth==2000))
            self.assertEqual(len((recorder.path/'mission_transitions.jsonl').read_text().splitlines()),2)
            self.assertEqual(recorder.health['written'],105)
            self.assertIsNone(recorder.health['error'])
            print('105-frame bounded archive bytes:',sum(p.stat().st_size for p in recorder.path.rglob('*') if p.is_file()))


if __name__=='__main__':
    unittest.main()
