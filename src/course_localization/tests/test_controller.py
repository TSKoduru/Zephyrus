"""Reference validation, coordinate transforms, feedback limits, and stop latch."""

from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from course_localization.controller import CourseFollower, TimedPath


def path(yaw=0.0):
    return TimedPath(
        [0.0, 1.0, 2.0], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], [yaw, yaw, yaw]
    )


def step(controller, now, pose, **kwargs):
    return controller.update(
        now,
        0.1,
        pose,
        kwargs.pop("odom_yaw", pose[3]),
        measurement_time=now,
        odom_time=now,
        **kwargs,
    )


class ControllerTests(unittest.TestCase):
    def test_uniform_time_scaling_and_stationary_prefix(self):
        reference = TimedPath(
            [0.0, 1.0, 2.0, 3.0, 4.0],
            [[0, 0, 0.116], [0, 0, 0.116], [0, 0, 0.116], [1, 0, 1.116], [2, 0, 0.116]],
            np.zeros(5),
        )
        self.assertEqual(reference.trimmed_prefix_s, 2.0)
        self.assertEqual(reference.authored_time(0), 2.0)
        np.testing.assert_array_equal(reference.sample(0)[0], [0, 0, 0.116])
        self.assertAlmostEqual(reference.authored_time(reference.duration), 4.0)
        for stamp in np.linspace(0, reference.duration, 100):
            self.assertLessEqual(np.linalg.norm(reference.sample(stamp)[2]), 0.3 + 1e-12)
        np.testing.assert_array_equal(reference.sample(reference.duration)[0], [2, 0, 0.116])
        self.assertGreater(reference.sample(0.1)[0][2], 0.116)

    def test_file_loading_and_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / "path.npz"
            np.savez(file, time=[0, 1], position=[[0, 0, 0], [1, 0, 0]], yaw=[0, 0])
            self.assertAlmostEqual(TimedPath.load(file).duration, 1 / 0.3)
        for times, positions, yaw in [
            ([0, 0], [[0, 0, 0], [1, 0, 0]], [0, 0]),
            ([0, 1], [[0, 0, 0], [1, 0, 1]], [0, 0]),
            ([0, 1], [[0, 0, 0], [1, 0, 0]], [0, np.nan]),
        ]:
            with self.assertRaises(ValueError):
                TimedPath(times, positions, yaw)

    def test_ninety_degree_map_to_odom_rotation(self):
        controller = CourseFollower(path(np.pi / 2), max_acceleration=10.0)
        controller.reset(0.0)
        result = step(controller, 0.1, [0, 0, 0, np.pi / 2], odom_yaw=0.0)
        np.testing.assert_allclose(result["velocity_enu"], [0, -0.3, 0], atol=1e-12)
        self.assertAlmostEqual(result["yaw_rate"], 0.0)

    def test_zero_feedback_is_scaled_feedforward(self):
        controller = CourseFollower(path(), position_kp=0.0, max_acceleration=10.0)
        controller.reset(0.0)
        result = step(controller, 0.1, [0, 0, 0, 0])
        np.testing.assert_allclose(result["velocity_enu"], [0.3, 0, 0])
        self.assertEqual(result["progress_s"], 0.0)
        self.assertEqual(controller.progress, 0.1)

    def test_tracking_lag_freezes_reference_and_removes_feedforward(self):
        controller = CourseFollower(path(), position_kp=1.0, max_acceleration=10.0)
        controller.reset(0.0)
        result = step(controller, 0.1, [0, 0.3, 0, 0])
        self.assertEqual(controller.progress, 0.0)
        self.assertEqual(result["status"], "paused_tracking_error")
        np.testing.assert_allclose(result["velocity_enu"], [0, -0.3, 0])
        result = step(controller, 0.2, [0, 0, 0, 0])
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(controller.progress, 0.1)

    def test_velocity_acceleration_and_yaw_limits(self):
        controller = CourseFollower(path(np.pi))
        controller.reset(0.0)
        previous = np.zeros(3)
        for i in range(1, 20):
            result = step(controller, i * 0.1, [-1, -1, 0, 0])
            velocity = np.array(result["velocity_enu"])
            self.assertLessEqual(np.linalg.norm(velocity), 0.3 + 1e-12)
            self.assertLessEqual(np.linalg.norm(velocity - previous), 0.08 + 1e-12)
            self.assertLessEqual(abs(result["yaw_rate"]), 0.6)
            previous = velocity
        self.assertTrue(np.isfinite(previous).all())

    def test_stale_invalid_and_covariance_faults_latch_until_reset(self):
        for reason in ["stale", "invalid", "covariance", "exposure"]:
            controller = CourseFollower(path())
            controller.reset(0.0)
            kwargs = dict(measurement_time=0.1, odom_time=0.1)
            if reason == "stale":
                kwargs.update(measurement_time=-1.0, odom_time=-1.0)
            elif reason == "invalid":
                kwargs["valid"] = False
            elif reason == "covariance":
                kwargs["covariance"] = np.diag([-1.0, 1.0, 1.0, 1.0])
            else:
                kwargs["odom_time"] = 0.09
            failed = controller.update(0.1, 0.1, [0, 0, 0, 0], 0, **kwargs)
            self.assertTrue(failed["stop_requested"])
            self.assertEqual(controller.progress, 0.0)
            good = step(controller, 0.2, [0, 0, 0, 0])
            self.assertTrue(good["stop_requested"])
            np.testing.assert_array_equal(good["velocity_enu"], [0, 0, 0])
            self.assertEqual(controller.progress, 0.0)
            controller.reset(0.2)
            self.assertFalse(step(controller, 0.3, [0, 0, 0, 0])["stop_requested"])

    def test_end_has_no_feedforward_but_corrects_position(self):
        controller = CourseFollower(path(), max_acceleration=10.0)
        controller.reset(0.0)
        controller.progress = controller.path.duration
        exact = step(controller, 0.1, [2, 0, 0, 0])
        np.testing.assert_array_equal(exact["velocity_enu"], [0, 0, 0])
        self.assertEqual(exact["status"], "complete")
        correction = step(controller, 0.2, [1.9, 0, 0, 0])
        np.testing.assert_allclose(correction["velocity_enu"], [0.1, 0, 0], atol=1e-12)
        self.assertEqual(controller.progress, controller.path.duration)
        self.assertEqual(correction["status"], "settling")


if __name__ == "__main__":
    unittest.main()
