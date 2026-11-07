import unittest
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from course_localization.sensor_geometry import (
    StereoGeometry, floor_height_from_range, interpolate_pose, matrix_pose, pose_matrix)


class PoseGeometryTests(unittest.TestCase):
    def test_interpolation_translation_and_shortest_quaternion_arc(self):
        q = Rotation.from_euler('z', [[170.], [-170.]], degrees=True).as_quat()
        p, r = interpolate_pose([1., 1.1], [[0., 1., 2.], [2., 3., 4.]], q, 1.05, .11)
        np.testing.assert_allclose(p, [1., 2., 3.], atol=1e-12)
        self.assertAlmostEqual(abs(Rotation.from_quat(r).as_euler('xyz', degrees=True)[2]), 180.)

    def test_exact_sample_and_sign_equivalent_quaternions(self):
        q = np.array([0., 0., 0., 1.])
        p, result = interpolate_pose([4.], [[1., 2., 3.]], [q], 4.)
        np.testing.assert_array_equal(p, [1., 2., 3.])
        np.testing.assert_array_equal(result, q)
        _, result = interpolate_pose([0., .1], np.zeros((2, 3)), [q, -q], .05)
        np.testing.assert_allclose(Rotation.from_quat(result).as_matrix(), np.eye(3))

    def test_no_extrapolation_gaps_duplicates_or_invalid_quaternions(self):
        args = ([0., 1.], np.zeros((2, 3)), [[0., 0., 0., 1.]]*2)
        for t in (-.001, 1.001, .5):
            with self.subTest(t=t), self.assertRaises(ValueError):
                interpolate_pose(*args, t, .1)
        for stamps in ([0., 0.], [1., 0.]):
            with self.assertRaises(ValueError):
                interpolate_pose(stamps, args[1], args[2], 0.)
        with self.assertRaises(ValueError):
            interpolate_pose([0.], [[0., 0., 0.]], [[0., 0., 0., 0.]], 0.)

    def test_pose_matrix_round_trip_and_reject_reflection(self):
        p = np.array([.1, -.2, .3])
        q = Rotation.from_euler('xyz', [.1, -.2, .3]).as_quat()
        m = pose_matrix(p, q)
        pp, qq = matrix_pose(m)
        np.testing.assert_array_equal(pp, p)
        np.testing.assert_allclose(pose_matrix(pp, qq), m, atol=1e-12)
        m[0, :3] *= -1
        with self.assertRaises(ValueError):
            matrix_pose(m)


class StereoGeometryTests(unittest.TestCase):
    K = np.array([[600., 0., 511.5], [0., 600., 319.5], [0., 0., 1.]])

    def make(self, rotation=None, translation=None, **kwargs):
        return StereoGeometry(self.K, np.zeros(5), self.K, np.zeros(5),
                              np.eye(3) if rotation is None else rotation,
                              [-.091, 0., 0.] if translation is None else translation,
                              (1024, 640), **kwargs)

    def test_baseline_scaling_and_mono_remap(self):
        geometry = self.make()
        self.assertAlmostEqual(geometry.baseline_m, .091)
        np.testing.assert_allclose(geometry.K_depth[:2], geometry.K_right[:2]/4)
        image = np.full((640, 1024), 123, np.uint8)
        left, right = geometry.rectify(image, image)
        self.assertEqual(left.shape, (400, 640))
        np.testing.assert_array_equal(left[4:-4, 4:-4], 123)
        np.testing.assert_array_equal(right[4:-4, 4:-4], 123)
        with self.assertRaises(ValueError):
            geometry.rectify(image.astype(np.float32), image)
        with self.assertRaises(ValueError):
            geometry.rectify(image[:400, :640], image[:400, :640])

    def test_rectified_epipolar_and_right_optical_depth_convention(self):
        rotation = Rotation.from_euler('xyz', [.015, -.025, .01]).as_matrix()
        centre_right_in_left = np.array([.091, .001, .002])
        translation = -rotation @ centre_right_in_left
        geometry = self.make(rotation, translation)
        points_left = np.array([[.1, .1, 1.], [-.2, .05, 2.], [.4, -.2, 3.]])
        points_right = points_left @ rotation.T + translation
        rect_left = points_left @ geometry.R_left_rectification.T
        rect_right = points_right @ geometry.R_right_rectification.T
        uv_left = rect_left @ geometry.K_left.T
        uv_right = rect_right @ geometry.K_right.T
        uv_left = uv_left[:, :2]/uv_left[:, 2, None]
        uv_right = uv_right[:, :2]/uv_right[:, 2, None]
        np.testing.assert_allclose(uv_left[:, 1], uv_right[:, 1], atol=1e-9)
        recovered_z = geometry.K_right[0, 0]*geometry.baseline_m/(uv_left[:, 0]-uv_right[:, 0])
        np.testing.assert_allclose(recovered_z, rect_right[:, 2], atol=1e-9)
        raw_mount = pose_matrix([.03, -.04, .02], Rotation.from_euler('xyz', [.3, .2, -.1]).as_quat())
        rect_mount = geometry.body_from_rectified_right(raw_mount)
        np.testing.assert_allclose(rect_right @ rect_mount[:3, :3].T+rect_mount[:3, 3],
                                   points_right @ raw_mount[:3, :3].T+raw_mount[:3, 3], atol=1e-10)

    def test_reject_wrong_eye_sign_zero_vertical_and_aspect(self):
        for translation in ([0., 0., 0.], [.091, 0., 0.], [0., -.091, 0.]):
            with self.subTest(translation=translation), self.assertRaises(ValueError):
                self.make(translation=translation)
        with self.assertRaises(ValueError):
            self.make(processing_size=(640, 480))
        with self.assertRaises(ValueError):
            self.make(depth_size=(128, 80), processing_size=(600, 375))


class RangeGeometryTests(unittest.TestCase):
    def test_explicit_mount_and_downward_range(self):
        # +X sensor beam rotated to body -Z, mounted 4cm below body origin.
        mount = pose_matrix([.02, 0., -.04], Rotation.from_euler('y', np.pi/2).as_quat())
        self.assertAlmostEqual(floor_height_from_range(.76, mount, [0., 0.]), .8)
        with self.assertRaises(ValueError):
            floor_height_from_range(.76, np.eye(4), [0., 0.])
        with self.assertRaises(ValueError):
            floor_height_from_range(-.1, mount, [0., 0.])


if __name__ == '__main__':
    unittest.main()
