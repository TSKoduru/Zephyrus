"""Portable-core checks; set COURSE_REFERENCE_ROOT to run extraction parity."""

import ast
import importlib
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np
from scipy.special import logsumexp

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from course_localization.core import course_map, inertial_aid, motion_model
from course_localization.core import occupancy_prior, particle_filter, range_sensor
from course_localization.core.depth_sensor import DepthFrame
from course_localization.core.stereo_depth import StereoDepthEstimator

REFERENCE = os.environ.get("COURSE_REFERENCE_ROOT")


class PortableCoreTests(unittest.TestCase):
    def test_core_has_no_simulator_or_command_dependencies(self):
        for path in (PACKAGE / "course_localization/core").glob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [node.module or ""]
                else:
                    continue
                self.assertFalse(
                    any(name.split(".")[0] in {"mujoco", "subprocess", "rclpy"} for name in names),
                    path.name,
                )

    def test_range_beam_configuration(self):
        poses = np.array([[0.0, 0.0, 1.0, 0.0]])
        narrow = range_sensor.FloorRangeLikelihood(full_fov_deg=1.0)
        default = range_sensor.FloorRangeLikelihood()
        self.assertLess(
            narrow.expected_ranges(poses, [0.0, 0.0])[0],
            default.expected_ranges(poses, [0.0, 0.0])[0],
        )
        np.testing.assert_array_equal(narrow.rays, range_sensor.cone_rays(1.0)[0])

    def test_validation(self):
        with self.assertRaises(ValueError):
            course_map.StructuralMap(np.zeros((0, 3)), (), 0.05)
        with self.assertRaises(ValueError):
            motion_model.incremental_motion_covariance(np.zeros(4), -1)
        with self.assertRaises(ValueError):
            StereoDepthEstimator(np.eye(3), baseline_m=-1)
        packet = DepthFrame(1.0, np.array([[1.0, np.nan, 0.0]]), np.eye(3), np.eye(4))
        np.testing.assert_array_equal(packet.valid, [[True, False, False]])


@unittest.skipUnless(REFERENCE, "Set COURSE_REFERENCE_ROOT for source parity")
class ReferenceParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(REFERENCE))
        cls.reference = {
            name: importlib.import_module("localization." + name)
            for name in [
                "particle_filter",
                "inertial_aid",
                "motion_model",
                "range_sensor",
                "course_map",
                "occupancy_prior",
            ]
        }

    def test_selected_definitions_ast_exact(self):
        selected = {
            "particle_filter": ["wrap_angle", "ParticleFilter"],
            "range_sensor": [
                "cone_rays",
                "weighted_median",
                "RangeMeasurement",
                "FloorRangeLikelihood",
            ],
            "course_map": ["is_structure", "StructuralMap"],
            "occupancy_prior": ["CylinderOccupancyPrior", "OccupancyParticleFilter"],
            "inertial_aid": [
                "replace_vo_yaw",
                "InertialAidedParticleFilter",
                "KeyframeInertialParticleFilter",
            ],
            "imu": ["InertialIncrement", "InertialAid"],
            "stereo_depth": ["StereoDepthEstimator"],
            "image_noise_estimation": ["image_noise_sigma"],
            "depth_sensor": ["DepthFrame"],
        }
        for target, names in selected.items():
            source = "inertial_aid" if target == "imu" else target
            old = ast.parse((Path(REFERENCE) / "localization" / (source + ".py")).read_text())
            new = ast.parse((PACKAGE / "course_localization/core" / (target + ".py")).read_text())
            old_nodes = {n.name: n for n in old.body if hasattr(n, "name")}
            new_nodes = {n.name: n for n in new.body if hasattr(n, "name")}
            for name in names:
                if target == "range_sensor" and name == "FloorRangeLikelihood":
                    constructor = next(
                        n
                        for n in new_nodes[name].body
                        if isinstance(n, ast.FunctionDef) and n.name == "__init__"
                    )
                    self.assertEqual(constructor.args.args[-1].arg, "full_fov_deg")
                    self.assertEqual(constructor.args.defaults[-1].value, 27.0)
                    constructor.args.args.pop()
                    constructor.args.defaults.pop()
                    call = next(
                        n
                        for n in ast.walk(constructor)
                        if isinstance(n, ast.Call)
                        and isinstance(n.func, ast.Name)
                        and n.func.id == "cone_rays"
                    )
                    self.assertEqual(
                        ast.dump(call.args[0]),
                        ast.dump(ast.Name(id="full_fov_deg", ctx=ast.Load())),
                    )
                    call.args.clear()
                self.assertEqual(
                    ast.dump(old_nodes[name]), ast.dump(new_nodes[name]), (target, name)
                )

    def test_pf_motion_range_and_keyframe_parity(self):
        reference = self.reference
        axis = np.linspace(-2, 2, 35)
        a, b = np.meshgrid(axis, axis)
        points = np.r_[
            np.c_[a.ravel(), b.ravel(), np.zeros(a.size)],
            np.c_[np.full(a.size, 2.0), a.ravel(), b.ravel()],
        ]
        filters, gates, ranges = [], [], []
        for modules in [
            reference,
            dict(
                course_map=course_map,
                occupancy_prior=occupancy_prior,
                motion_model=motion_model,
                range_sensor=range_sensor,
            ),
        ]:
            structural = modules["course_map"].StructuralMap(points, (), 0.1)
            prior = modules["occupancy_prior"].CylinderOccupancyPrior.__new__(
                modules["occupancy_prior"].CylinderOccupancyPrior
            )
            prior.origin = np.full(3, -3.0)
            prior.spacing = 0.25
            prior.geometry_sigma = 0.05
            prior.distance = np.full((25, 25, 25), 0.5, np.float32)
            filters.append(
                modules["occupancy_prior"].OccupancyParticleFilter(
                    structural,
                    [0, 0, 0.7, 0],
                    [0.15, 0.15, 0.07, 0.1],
                    num_particles=96,
                    seed=2026,
                    occupancy_prior=prior,
                    effective_rays=6,
                    sigma_hit=0.18,
                    outlier_probability=0.35,
                    keyframe_translation=0.3,
                    keyframe_yaw_deg=10,
                )
            )
            gates.append(modules["motion_model"].MotionIntegrityGate())
            ranges.append(modules["range_sensor"].FloorRangeLikelihood(floor_sigma_m=0.1))
        K = np.array([[25.0, 0, 11.5], [0, 25.0, 7.5], [0, 0, 1.0]])
        transform = np.eye(4)
        transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        for step in range(24):
            delta = np.array([0.035, -0.003, 0.002, 0.025])
            if step == 9:
                delta[0] = 3.0  # Rejected-motion fallback must retain RNG parity.
            inc = inertial_aid.InertialIncrement(
                0.02,
                np.array([0.01, -0.02]),
                1e-5,
                True,
                "valid",
                np.zeros(3),
                True,
                np.array([0.01, -0.02]),
            )
            states = []
            for pf, gate, model in zip(filters, gates, ranges):
                cov = motion_model.incremental_motion_covariance(delta, 0.2) * 16
                aided, aided_cov = inertial_aid.replace_vo_yaw(delta, cov, inc)
                decision = gate.evaluate(aided, 0.2, aided_cov)
                pf.predict_aided(delta, cov, inc, decision)
                reading = range_sensor.RangeMeasurement(step * 0.2, 0.66, True, "valid", 0.0005)
                pf.log_weights += model.log_likelihood(pf.particles, reading, inc.roll_pitch)
                pf.log_weights -= logsumexp(pf.log_weights)
                pf._posterior = None
                depth = np.full((16, 24), 1.8, np.float32)
                if step == 5:
                    depth[:] = np.nan
                states.append(
                    pf.update(DepthFrame(step * 0.2, depth, K, transform), inc.roll_pitch)
                )
            np.testing.assert_array_equal(filters[0].particles, filters[1].particles)
            np.testing.assert_array_equal(filters[0].weights, filters[1].weights)
            for key in ["pose", "covariance"]:
                np.testing.assert_array_equal(states[0][key], states[1][key])
            self.assertEqual(states[0]["resample_count"], states[1]["resample_count"])
            self.assertEqual(states[0]["keyframe_count"], states[1]["keyframe_count"])

    def test_imu_integration_parity(self):
        old = self.reference["inertial_aid"].InertialAid(startup_hold_s=0.5)
        new = inertial_aid.InertialAid(startup_hold_s=0.5)
        times = np.arange(201) * 0.01
        gyro = np.zeros((201, 3))
        gyro[:, 2] = 0.001
        gyro[times > 0.7, 2] += 0.2
        accel = np.tile([0.0, 0.0, 9.81], (201, 1))
        for obj in [old, new]:
            obj.add_batch(times, gyro, accel)
        for start, end in [(0.1, 0.3), (0.8, 1.0), (1.2, 1.8)]:
            a, b = old.interval(start, end), new.interval(start, end)
            for key in vars(a):
                np.testing.assert_array_equal(getattr(a, key), getattr(b, key))

    def test_stereo_fixed_images_parity(self):
        # Execute only the original stereo class; never import simulator depth_sensor.
        source = Path(REFERENCE) / "localization/stereo_depth.py"
        tree = ast.parse(source.read_text())
        node = next(
            n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "StereoDepthEstimator"
        )
        import cv2

        namespace = dict(
            np=np,
            cv2=cv2,
            DepthFrame=DepthFrame,
            __name__="localization.stereo_depth",
            __package__="localization",
        )
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
        rng = np.random.default_rng(83)
        left = rng.integers(0, 256, (80, 128), dtype=np.uint8)
        right = np.zeros_like(left)
        right[:, :-8] = left[:, 8:]
        K = np.array([[90.0, 0, 63.5], [0, 90.0, 39.5], [0, 0, 1.0]])
        old = namespace["StereoDepthEstimator"](
            K, output_size=(32, 20), texture_noise_multiplier=0.5
        )
        new = StereoDepthEstimator(K, output_size=(32, 20), texture_noise_multiplier=0.5)
        a, b = [obj.compute(left, right, 2.0, np.eye(4)) for obj in [old, new]]
        for key in ["depth_m", "K", "T_body_camera"]:
            np.testing.assert_array_equal(getattr(a, key), getattr(b, key))
        self.assertEqual(old.last_diagnostics, new.last_diagnostics)
        self.assertGreater(np.isfinite(b.depth_m).sum(), 0)


if __name__ == "__main__":
    unittest.main()
