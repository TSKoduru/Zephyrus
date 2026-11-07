"""Calibrated, ROS-independent sensor geometry. Transforms map source to target.

Quaternions are always [x, y, z, w]. Times are seconds. Stereo extrinsics must
come from calibration/TF, not an assumed OAK baseline or camera mounting angle.
Only pinhole/OpenCV distortion is supported; callers must reject fisheye models.
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def _finite(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return result.copy()


def _rotation(value, name):
    result = _finite(value, (3, 3), name)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(np.linalg.det(result), 1., atol=1e-6, rtol=0):
        raise ValueError(f'{name} must be a proper rotation')
    return result


def _quaternion(value):
    q = _finite(value, (4,), 'quaternion_xyzw')
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('Quaternion must have nonzero norm')
    return q / norm


def pose_matrix(position, quaternion_xyzw):
    """Return target_from_source from its translation and XYZW quaternion."""
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(_quaternion(quaternion_xyzw)).as_matrix()
    result[:3, 3] = _finite(position, (3,), 'position')
    return result


def matrix_pose(matrix):
    """Validate a rigid transform and return (position, quaternion_xyzw)."""
    matrix = _finite(matrix, (4, 4), 'transform')
    if not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-9, rtol=0):
        raise ValueError('Transform must have homogeneous last row [0,0,0,1]')
    rotation = _rotation(matrix[:3, :3], 'transform rotation')
    return matrix[:3, 3].copy(), Rotation.from_matrix(rotation).as_quat()


def interpolate_pose(stamps, positions, quaternions_xyzw, t, max_gap_s=.1):
    """Interpolate measured poses only inside a strictly ordered time buffer.

    Exact measurements do not require a neighbouring sample. Interpolation
    requires both brackets and a gap <= max_gap_s; extrapolation is forbidden.
    Translation is linear and orientation uses shortest-arc quaternion SLERP.
    """
    stamps = np.asarray(stamps, dtype=np.float64)
    if stamps.ndim != 1 or len(stamps) == 0 or not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError('Pose timestamps must be finite and strictly increasing')
    positions = _finite(positions, (len(stamps), 3), 'positions')
    quaternions = _finite(quaternions_xyzw, (len(stamps), 4), 'quaternions_xyzw')
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-12):
        raise ValueError('Pose quaternion must have nonzero norm')
    quaternions /= norms[:, None]
    if not np.isfinite(t) or not np.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError('Query time and positive maximum gap must be finite')
    if t < stamps[0] or t > stamps[-1]:
        raise ValueError('Pose timestamp is outside measured coverage; no extrapolation')
    upper = int(np.searchsorted(stamps, t))
    if upper < len(stamps) and stamps[upper] == t:
        return positions[upper].copy(), quaternions[upper].copy()
    lower = upper - 1
    gap = stamps[upper] - stamps[lower]
    if gap > max_gap_s:
        raise ValueError('Odometry interpolation bracket exceeds maximum gap')
    fraction = (t - stamps[lower]) / gap
    position = positions[lower] + fraction * (positions[upper] - positions[lower])
    # Relative times avoid precision loss in the SLERP implementation at epochs.
    rotation = Slerp([0., gap], Rotation.from_quat(quaternions[[lower, upper]]))([t-stamps[lower]])[0]
    return position, rotation.as_quat()


def _size(value, name):
    array = np.asarray(value)
    if array.shape != (2,) or not np.isfinite(array).all() or np.any(array <= 0) or not np.all(array == np.floor(array)):
        raise ValueError(f'{name} must be positive integer (width,height)')
    return tuple(int(v) for v in array)


def _intrinsic(value, name):
    matrix = _finite(value, (3, 3), name)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0., 0., 1.], atol=1e-9, rtol=0):
        raise ValueError(f'{name} requires positive focal lengths and pinhole last row')
    if not np.allclose([matrix[0, 1], matrix[1, 0]], 0., atol=1e-9, rtol=0):
        raise ValueError('OpenCV stereo rectification requires zero-skew camera matrices')
    return matrix


def _distortion(value):
    if value is None:
        return np.zeros(5)
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if len(array) == 0:
        return np.zeros(5)
    if len(array) not in (4, 5, 8, 12, 14) or not np.isfinite(array).all():
        raise ValueError('Expected finite OpenCV pinhole distortion coefficients')
    return array.copy()


class StereoGeometry:
    """Rectify raw stereo using explicit right-optical-from-left-optical TF.

    X_right = R_right_from_left @ X_left + t_right_from_left. For an ordinary
    parallel rig, the right camera is physically to the right, so t_x is
    negative. Reversed eyes and vertical stereo are rejected, never auto-flipped.
    K_right/P_right describe the rectified processing image. K_depth follows the
    same simple block scaling as the existing right-aligned stereo estimator.
    """
    def __init__(self, K_left, D_left, K_right, D_right,
                 R_right_from_left, t_right_from_left, image_size,
                 processing_size=(640, 400), depth_size=(160, 100)):
        self.image_size = _size(image_size, 'image_size')
        self.processing_size = _size(processing_size, 'processing_size')
        self.depth_size = _size(depth_size, 'depth_size')
        width, height = self.image_size
        pw, ph = self.processing_size
        dw, dh = self.depth_size
        if width*ph != height*pw or pw*dh != ph*dw:
            raise ValueError('Raw, processing and depth images must have matching aspect ratios')
        if pw % dw or ph % dh:
            raise ValueError('Depth resolution must evenly divide processing resolution')
        left = _intrinsic(K_left, 'K_left')
        right = _intrinsic(K_right, 'K_right')
        dl, dr = _distortion(D_left), _distortion(D_right)
        rotation = _rotation(R_right_from_left, 'R_right_from_left')
        translation = _finite(t_right_from_left, (3,), 't_right_from_left')
        if np.linalg.norm(translation) <= 1e-9:
            raise ValueError('Stereo baseline is zero; calibrated extrinsics are required')
        r1, r2, p1, p2, self.Q, self.roi_left, self.roi_right = cv2.stereoRectify(
            left, dl.reshape(-1, 1), right, dr.reshape(-1, 1), self.image_size,
            rotation, translation.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0, newImageSize=self.processing_size)
        if not all(np.isfinite(v).all() for v in (r1, r2, p1, p2, self.Q)):
            raise ValueError('Stereo calibration produced nonfinite rectification')
        self.R_left_rectification, self.R_right_rectification = r1, r2
        self.P_left, self.P_right = p1, p2
        self.K_left, self.K_right = p1[:, :3].copy(), p2[:, :3].copy()
        if p1[0, 0] <= 0 or p2[0, 0] <= 0:
            raise ValueError('Rectification produced nonpositive focal length')
        self.baseline_m = float(p1[0, 3]/p1[0, 0] - p2[0, 3]/p2[0, 0])
        if self.baseline_m <= 1e-9 or abs(p2[1, 3]) > 1e-7 or abs(p1[1, 3]) > 1e-7:
            raise ValueError('Expected positive horizontal baseline: check eye order and optical TF')
        if not np.allclose(self.K_left, self.K_right, atol=1e-6, rtol=1e-9):
            raise ValueError('Rectified images require matching intrinsics for the stereo matcher')
        self.K_depth = self.K_right.copy()
        self.K_depth[0] *= dw/pw
        self.K_depth[1] *= dh/ph
        self._left_maps = cv2.initUndistortRectifyMap(left, dl, r1, self.K_left, self.processing_size, cv2.CV_32FC1)
        self._right_maps = cv2.initUndistortRectifyMap(right, dr, r2, self.K_right, self.processing_size, cv2.CV_32FC1)

    def rectify(self, left, right):
        """Return equal uint8 mono rectified images at processing_size."""
        expected = (self.image_size[1], self.image_size[0])
        for value in (left, right):
            if not isinstance(value, np.ndarray) or value.shape != expected or value.dtype != np.uint8:
                raise ValueError('Raw stereo must be uint8 mono with the calibrated image resolution')
        return (cv2.remap(left, *self._left_maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT),
                cv2.remap(right, *self._right_maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT))

    def body_from_rectified_right(self, T_body_raw_right):
        """Rotate rectified-right rays back to raw-right before applying TF."""
        matrix_pose(T_body_raw_right)
        raw_from_rectified = np.eye(4)
        raw_from_rectified[:3, :3] = self.R_right_rectification.T
        return np.asarray(T_body_raw_right, dtype=float) @ raw_from_rectified


def floor_height_from_range(range_m, T_body_sensor, roll_pitch,
                            beam_axis_sensor=(1., 0., 0.), floor_z=0.):
    """Narrow-beam nominal-floor geometry, not a probabilistic range filter.

    ROS Range uses +X as its centre ray. Pass a different axis explicitly if the
    publisher's documented sensor frame differs. Reject horizontal/upward beams.
    Finite FoV, obstacles and range uncertainty remain the caller's responsibility.
    """
    matrix_pose(T_body_sensor)
    attitude = _finite(roll_pitch, (2,), 'roll_pitch')
    axis = _finite(beam_axis_sensor, (3,), 'beam_axis_sensor')
    if np.linalg.norm(axis) < 1e-12 or not np.isfinite([range_m, floor_z]).all() or range_m < 0:
        raise ValueError('Range and beam axis must be valid')
    level = Rotation.from_euler('xyz', [*attitude, 0.]).as_matrix()
    origin = level @ np.asarray(T_body_sensor)[:3, 3]
    direction = level @ np.asarray(T_body_sensor)[:3, :3] @ (axis/np.linalg.norm(axis))
    if direction[2] >= -1e-6:
        raise ValueError('Range beam does not point down toward the nominal floor')
    return float(floor_z-origin[2]-float(range_m)*direction[2])
