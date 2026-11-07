"""Optional recorded-input regression; no truth is passed to the estimator."""
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
import json
import numpy as np

from course_localization.filter_session import FilterSession
from course_localization.core.inertial_aid import InertialIncrement
from course_localization.core.range_sensor import RangeMeasurement


@unittest.skipUnless(os.environ.get('COURSE_REFERENCE_ROOT'), 'Recorded reference is optional')
class RecordedSessionParity(unittest.TestCase):
    def test_selected_hardware_session_matches_recorded_sim_sensor_inputs(self):
        root = Path(os.environ['COURSE_REFERENCE_ROOT'])
        folder = root / 'localization_results/robustness/datasets/seed_7301'
        if not folder.exists():
            self.skipTest('Reference recording7301 not present')
        manifest = json.loads((folder / 'manifest.json').read_text())
        config = {k: manifest[k] for k in ('initial_mean4', 'initial_std4', 'particle_seed')}
        config.update(range_position_body_m=[0, 0, -.04], range_fov_deg=27.)
        session = FilterSession(config, root / 'assets/real_course/localization_map',
                                root / 'localization_results/robustness/map_cache')
        for file in sorted(folder.glob('frame_*.npz')):
            with np.load(file) as packet:
                inc = InertialIncrement(float(packet['imu_yaw_delta']), packet['imu_roll_pitch'],
                    float(packet['imu_yaw_variance']), bool(packet['imu_aid_valid']),
                    str(packet['imu_aid_status']), np.zeros(3), bool(packet['imu_aid_valid']),
                    packet['imu_start_roll_pitch'])
                session.inertial = SimpleNamespace(interval=lambda start, end, inc=inc: inc)
                frame = SimpleNamespace(timestamp=float(packet['timestamp']), depth_m=packet['depth_m'],
                                        K=packet['K'], T_body_camera=packet['T_body_camera'])
                measurement = RangeMeasurement(frame.timestamp, float(packet['range_m']),
                    bool(packet['range_valid']), 'recorded', float(packet['range_variance_m2']),
                    float(packet['range_field_of_view_rad']), float(packet['range_min_m']),
                    float(packet['range_max_m']))
                state, _, _, _ = session.step(frame, packet['odom_position'],
                    packet['odom_quaternion_xyzw'], measurement, True)
                np.testing.assert_array_equal(state['pose'], packet['baseline_pose4'])
                np.testing.assert_array_equal(session.pf.particles, packet['baseline_particles'])
                np.testing.assert_array_equal(session.pf.weights, packet['baseline_weights'])


if __name__ == '__main__':
    unittest.main()
