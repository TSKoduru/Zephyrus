"""Measured-IMU startup calibration and causal attitude integration."""

from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .particle_filter import wrap_angle


@dataclass(frozen=True)
class InertialIncrement:
    yaw_delta: float
    roll_pitch: np.ndarray
    yaw_variance: float
    valid: bool
    status: str
    gyro_bias: np.ndarray
    calibrated: bool
    start_roll_pitch: np.ndarray


class InertialAid:
    def __init__(
        self,
        *,
        startup_hold_s=2.0,
        gravity=9.81,
        gravity_gain=0.08,
        gyro_noise_density=0.0005,
        max_gap_s=0.05,
        max_extrapolation_s=0.01,
    ):
        if (
            startup_hold_s < 0
            or gravity <= 0
            or gravity_gain < 0
            or (gyro_noise_density < 0)
            or (max_gap_s <= 0)
        ):
            raise ValueError("Invalid inertial aid configuration")
        self.startup_hold_s = float(startup_hold_s)
        self.gravity = float(gravity)
        self.gravity_gain = float(gravity_gain)
        self.gyro_noise_density = float(gyro_noise_density)
        self.max_gap_s = float(max_gap_s)
        self.max_extrapolation_s = float(max_extrapolation_s)
        self.rotation = Rotation.identity()
        self.bias = np.zeros(3)
        self.bias_variance = np.full(3, 0.01**2)
        self.calibrated = False
        self.calibration_time = None
        self.calibration_attempted = False
        self.calibration_status = "collecting_stationary_startup"
        self.startup = []
        self.times = []
        self.quaternions = []
        self.gaps = []
        self.last_gyro = None
        self.last_time = None
        self.start_time = None
        self.last_accel = None
        self.gravity_correction_count = 0

    def _calibrate(self):
        sample = np.asarray(self.startup)
        gyro = sample[:, :3]
        accel = sample[:, 3:]
        if len(sample) < 30:
            self.calibration_status = "startup_insufficient_samples"
            return
        med = np.median(gyro, axis=0)
        mad = np.median(abs(gyro - med), axis=0)
        use = np.all(abs(gyro - med) < np.maximum(0.003, 4.5 * mad), axis=1)
        if use.sum() < 30:
            self.calibration_status = "startup_insufficient_inliers"
            return
        gyro = gyro[use]
        accel = accel[use]
        average = np.mean(accel, axis=0)
        if (
            np.linalg.norm(med) > 0.08
            or np.linalg.norm(np.std(gyro, axis=0)) > 0.06
            or np.linalg.norm(np.std(accel, axis=0)) > 0.35
            or (abs(np.linalg.norm(average) - self.gravity) > 0.5)
        ):
            self.calibration_status = "startup_not_stationary"
            return
        self.bias = np.mean(gyro, axis=0)
        self.bias_variance = np.var(gyro, axis=0) / len(gyro) + 0.00015**2
        n = average / np.linalg.norm(average)
        roll = np.arctan2(n[1], n[2])
        pitch = np.arctan2(-n[0], np.hypot(n[1], n[2]))
        yaw = self.rotation.as_euler("xyz")[2]
        self.rotation = Rotation.from_euler("xyz", [roll, pitch, yaw])
        self.calibrated = True
        self.calibration_status = "calibrated_stationary_startup"

    def add_batch(self, times, gyro, accel):
        times = np.asarray(times, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        if (
            times.ndim != 1
            or gyro.shape != (len(times), 3)
            or accel.shape != (len(times), 3)
            or (not all((np.isfinite(a).all() for a in (times, gyro, accel))))
            or np.any(np.diff(times) <= 0)
        ):
            raise ValueError("IMU batch must contain finite increasing timestamps and Nx3 samples")
        for t, w, a in zip(times, gyro, accel):
            t = float(t)
            if self.last_time is not None and t <= self.last_time:
                continue
            if self.start_time is None:
                self.start_time = t
            if not self.calibration_attempted:
                if t - self.start_time <= self.startup_hold_s + 1e-09:
                    self.startup.append(np.r_[w, a])
                if t - self.start_time >= self.startup_hold_s - 1e-09:
                    self.calibration_attempted = True
                    self._calibrate()
                    if self.calibrated:
                        self.calibration_time = t
            if self.last_time is not None:
                dt = t - self.last_time
                if dt > self.max_gap_s:
                    self.gaps.append((self.last_time, t))
                else:
                    angular = 0.5 * (w + self.last_gyro) - self.bias
                    norm = np.linalg.norm(a)
                    if self.calibrated and norm > 1e-06:
                        predicted = self.rotation.inv().apply([0.0, 0.0, 1.0])
                        measured = a / norm
                        angle = np.arccos(np.clip(predicted @ measured, -1, 1))
                        if abs(norm - self.gravity) < 0.6 and angle < np.deg2rad(15.0):
                            residual = np.cross(measured, predicted)
                            correction = self.gravity_gain * residual
                            correction *= min(
                                1.0, np.deg2rad(0.35) / max(np.linalg.norm(correction), 1e-12)
                            )
                            angular += correction
                            self.gravity_correction_count += 1
                    self.rotation = self.rotation * Rotation.from_rotvec(angular * dt)
            self.last_time = t
            self.last_gyro = w.copy()
            self.last_accel = a.copy()
            self.times.append(t)
            self.quaternions.append(self.rotation.as_quat())

    def _at(self, t):
        if not self.times:
            return None
        times = np.asarray(self.times)
        index = np.searchsorted(times, t)
        if t < times[0] - 1e-09 or t > times[-1] + self.max_extrapolation_s:
            return None
        if index == len(times):
            return self.rotation * Rotation.from_rotvec(
                (self.last_gyro - self.bias) * (t - times[-1])
            )
        if index == 0 or abs(times[index] - t) < 1e-09:
            return Rotation.from_quat(self.quaternions[index])
        return Slerp(
            times[index - 1 : index + 1],
            Rotation.from_quat(self.quaternions[index - 1 : index + 1]),
        )([t])[0]

    def interval(self, start, end):
        start, end = (float(start), float(end))
        if not np.isfinite([start, end]).all() or end < start:
            raise ValueError("Invalid inertial interval")
        first = self._at(start)
        last = self._at(end)
        status = self.calibration_status
        valid = first is not None and last is not None and self.calibrated
        if self.calibration_time is not None and start < self.calibration_time - 1e-09:
            valid = False
            status = "startup_not_yet_calibrated_at_interval"
        if any((a < end - 1e-09 and b > start + 1e-09 for a, b in self.gaps)):
            valid = False
            status = "imu_sample_gap"
        if first is None or last is None:
            status = "imu_interval_uncovered"
        attitude = np.zeros(2) if last is None else last.as_euler("xyz")[:2]
        delta = (
            0.0
            if first is None or last is None
            else float(wrap_angle(last.as_euler("xyz")[2] - first.as_euler("xyz")[2]))
        )
        duration = end - start
        variance = (
            self.gyro_noise_density**2 * duration + float(np.max(self.bias_variance)) * duration**2
        )
        variance += np.deg2rad(0.1) ** 2 * duration
        start_attitude = np.zeros(2) if first is None else first.as_euler("xyz")[:2]
        return InertialIncrement(
            delta,
            attitude,
            variance,
            valid,
            status,
            self.bias.copy(),
            self.calibrated,
            start_attitude,
        )
