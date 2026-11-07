"""One calibrated, measured-data PF session. No flight commands or simulator input."""
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.special import logsumexp

from .core.course_map import StructuralMap
from .core.inertial_aid import InertialAid, replace_vo_yaw
from .core.motion_model import MotionIntegrityGate, MotionIntegrityLimits, incremental_motion_covariance
from .core.occupancy_prior import CylinderOccupancyPrior, OccupancyParticleFilter
from .core.particle_filter import wrap_angle
from .core.range_sensor import FloorRangeLikelihood
from .sensor_geometry import pose_matrix


class FilterSession:
    def __init__(self, config, map_directory, cache_directory):
        self.config = config
        self.initial = np.asarray(config['initial_mean4'], dtype=float)
        course = StructuralMap.from_assets(Path(map_directory))
        prior = CylinderOccupancyPrior(map_directory, cache_directory, geometry_sigma=0.05)
        self.pf = OccupancyParticleFilter(
            course, self.initial, config['initial_std4'], num_particles=800,
            seed=config['particle_seed'], occupancy_prior=prior, sigma_hit=0.18,
            outlier_probability=0.35, effective_rays=6.0,
            keyframe_translation=0.3, keyframe_yaw_deg=10.0,
        )
        self.inertial = InertialAid()
        self.gate = MotionIntegrityGate(MotionIntegrityLimits(enable_acceleration=True))
        self.range_model = FloorRangeLikelihood(
            site_position=config['range_position_body_m'], floor_sigma_m=0.10,
            full_fov_deg=config['range_fov_deg'],
        )
        self.previous = None
        self.last_attitude = np.zeros(2)
        self.map_from_odom = None

    def step(self, frame, position, quaternion, measurement, imu_covered):
        stamp = frame.timestamp
        start = self.previous[0] if self.previous is not None else stamp
        increment = self.inertial.interval(start, stamp)
        if not imu_covered:
            increment = replace(increment, valid=False, status='imu_exposure_not_covered')
        attitude = Rotation.from_quat(quaternion).as_euler('xyz')[:2]
        if increment.valid:
            attitude = increment.roll_pitch.copy()
        delta = np.zeros(4)
        covariance = np.zeros((4, 4))
        decision = None
        dt = 0.0
        if self.previous is not None:
            old_stamp, old_position, old_quaternion = self.previous
            dt = stamp - old_stamp
            if dt <= 0:
                raise ValueError('Exposure timestamps must increase; restart after a clock reset')
            old_rotation = Rotation.from_quat(old_quaternion)
            old_yaw = old_rotation.as_euler('xyz')[2]
            new_yaw = Rotation.from_quat(quaternion).as_euler('xyz')[2]
            delta[:3] = old_rotation.inv().apply(position - old_position)
            delta[3] = wrap_angle(new_yaw - old_yaw)
            covariance = incremental_motion_covariance(delta, dt) * 4.0**2
            aided_delta, aided_covariance = replace_vo_yaw(delta, covariance, increment)
            decision = self.gate.evaluate(aided_delta, dt, aided_covariance)
            self.pf.predict_aided(delta, covariance, increment, decision)
            if not decision.accepted and not increment.valid:
                attitude = self.last_attitude.copy()
        self.last_attitude = attitude.copy()
        self.previous = (stamp, position.copy(), quaternion.copy())
        if self.map_from_odom is None:
            initial_quaternion = Rotation.from_euler('xyz', [*attitude, self.initial[3]]).as_quat()
            self.map_from_odom = pose_matrix(self.initial[:3], initial_quaternion) @ np.linalg.inv(
                pose_matrix(position, quaternion)
            )
        if measurement.valid:
            self.pf.log_weights += self.range_model.log_likelihood(
                self.pf.particles, measurement, attitude
            )
            self.pf.log_weights -= logsumexp(self.pf.log_weights)
            self.pf._posterior = None
        result = self.pf.update(frame, roll_pitch=attitude)
        if measurement.valid and 1 / np.sum(self.pf.weights**2) < self.pf.n / 2:
            self.pf._posterior = self.pf._summarize()
            self.pf._resample()
        pose = result['pose']
        pf_quaternion = Rotation.from_euler('xyz', [*attitude, pose[3]]).as_quat()
        transforms = {
            'T_map_body_vo': self.map_from_odom @ pose_matrix(position, quaternion),
            'T_map_body_pf': pose_matrix(pose[:3], pf_quaternion),
        }
        arrays = {
            'timestamp': stamp, 'depth_m': frame.depth_m, 'K': frame.K,
            'T_body_camera': frame.T_body_camera, 'vo_position': position,
            'vo_quaternion_xyzw': quaternion, 'pose4': pose, 'covariance4': result['covariance'],
            'particles': self.pf.particles, 'weights': self.pf.weights,
            'raw_delta4': delta, 'motion_covariance4': covariance, 'dt': dt,
            'roll_pitch': attitude, 'range_m': measurement.range_m,
            'range_timestamp': measurement.timestamp, 'range_valid': measurement.valid,
            'imu_yaw_delta': increment.yaw_delta, 'imu_yaw_variance': increment.yaw_variance,
            'imu_gyro_bias': increment.gyro_bias, **transforms,
        }
        diagnostics = {
            'timestamp': stamp, 'health': result['health'], 'ess': result['ess'],
            'inlier_fraction': result['inlier_fraction'], 'range_status': measurement.status,
            'imu_valid': increment.valid, 'imu_status': increment.status,
            'motion_accepted': decision.accepted if decision else True,
            'motion_reason': decision.reason if decision else 'first_observation',
            'depth_keyframe': result.get('keyframe_update', False),
            'resample_count': self.pf.resample_count,
        }
        return result, arrays, diagnostics, transforms
