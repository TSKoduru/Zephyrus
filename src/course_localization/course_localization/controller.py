"""Optional bounded velocity feedback; no messages, arming, or estimator inputs."""

from pathlib import Path
import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def limit_norm(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector * min(1.0, limit / norm) if norm else vector.copy()


class TimedPath:
    """Piecewise-linear reference with one uniform time scale and no pose changes."""

    def __init__(
        self,
        time,
        position,
        yaw,
        *,
        max_speed=0.3,
        ground_z=None,
        ground_tolerance=0.01,
        trim_initial_hold=True,
    ):
        time = np.asarray(time, dtype=float).copy()
        position = np.asarray(position, dtype=float).copy()
        yaw = np.asarray(yaw, dtype=float).copy()
        if (
            time.ndim != 1
            or len(time) < 2
            or position.shape != (len(time), 3)
            or yaw.shape != time.shape
            or not np.isfinite(time).all()
            or not np.isfinite(position).all()
            or not np.isfinite(yaw).all()
            or np.any(np.diff(time) <= 0)
        ):
            raise ValueError("Reference needs finite increasing time, N×3 position, and N yaw")
        if not np.isfinite(max_speed) or max_speed <= 0:
            raise ValueError("max_speed must be positive and finite")
        if not np.isfinite(ground_tolerance) or ground_tolerance < 0:
            raise ValueError("ground_tolerance must be finite and nonnegative")
        ground = float(position[0, 2] if ground_z is None else ground_z)
        if not np.isfinite(ground) or np.any(abs(position[[0, -1], 2] - ground) > ground_tolerance):
            raise ValueError("Reference must start and end at the configured ground height")
        if np.any(position[:, 2] < ground - ground_tolerance):
            raise ValueError("Reference goes below the configured ground height")
        first = 0
        if trim_initial_hold:
            moved = np.linalg.norm(position - position[0], axis=1) > 1e-9
            turned = abs(wrap_angle(yaw - yaw[0])) > 1e-9
            changes = np.flatnonzero(moved | turned)
            if len(changes):
                first = max(0, int(changes[0]) - 1)
        self.original_time = time[first:]
        self.position = position[first:]
        self.yaw = np.unwrap(yaw[first:])
        peak_speed = float(
            np.max(
                np.linalg.norm(np.diff(self.position, axis=0), axis=1) / np.diff(self.original_time)
            )
        )
        self.time_scale = max(1.0, peak_speed / max_speed)
        self.time = (self.original_time - self.original_time[0]) * self.time_scale
        self.duration = float(self.time[-1])
        self.ground_z = ground
        self.trimmed_prefix_s = float(self.original_time[0] - time[0])

    @classmethod
    def load(cls, path, **kwargs):
        with np.load(Path(path), allow_pickle=False) as data:
            return cls(data["time"], data["position"], data["yaw"], **kwargs)

    def sample(self, progress):
        progress = float(np.clip(progress, 0.0, self.duration))
        if progress >= self.duration:
            return self.position[-1].copy(), float(self.yaw[-1]), np.zeros(3), 0.0
        index = min(int(np.searchsorted(self.time, progress, side="right")) - 1, len(self.time) - 2)
        index = max(0, index)
        duration = self.time[index + 1] - self.time[index]
        fraction = (progress - self.time[index]) / duration
        velocity = (self.position[index + 1] - self.position[index]) / duration
        yaw_rate = (self.yaw[index + 1] - self.yaw[index]) / duration
        return (
            self.position[index] + fraction * (self.position[index + 1] - self.position[index]),
            float(self.yaw[index] + fraction * (self.yaw[index + 1] - self.yaw[index])),
            velocity,
            float(yaw_rate),
        )

    def authored_time(self, progress):
        return float(
            self.original_time[0] + np.clip(progress, 0.0, self.duration) / self.time_scale
        )


class CourseFollower:
    """Position feedback in map axes; velocity output in the chosen command-frame axes.

    A fault latches zero velocity and requests stop until explicit reset. The odom_yaw argument is the body heading in the command frame
    (PX4 local ENU in the hardware adapter). Caller
    supplies matching PF/command-heading exposure stamps and owns stale-state/flight handling.
    """

    def __init__(
        self,
        path,
        *,
        position_kp=1.0,
        yaw_kp=1.0,
        max_speed=0.3,
        max_acceleration=0.8,
        max_yaw_rate=0.6,
        max_position_error=0.25,
        max_measurement_age=0.5,
    ):
        values = [
            position_kp,
            yaw_kp,
            max_speed,
            max_acceleration,
            max_yaw_rate,
            max_position_error,
            max_measurement_age,
        ]
        if not np.isfinite(values).all() or min(values[:2]) < 0 or min(values[2:]) <= 0:
            raise ValueError("Gains must be nonnegative and limits positive and finite")
        self.path = path
        self.position_kp, self.yaw_kp = position_kp, yaw_kp
        self.max_speed, self.max_acceleration = max_speed, max_acceleration
        self.max_yaw_rate = max_yaw_rate
        self.max_position_error = max_position_error
        self.max_measurement_age = max_measurement_age
        self.progress = 0.0
        self.velocity = np.zeros(3)
        self.last_time = None
        self.fault = "not_reset"

    def reset(self, now):
        if not np.isfinite(now):
            raise ValueError("Reset time must be finite")
        self.progress = 0.0
        self.velocity = np.zeros(3)
        self.last_time = float(now)
        self.fault = None

    def _result(self, status, yaw_rate=0.0, position_error=None):
        target, target_yaw, _, _ = self.path.sample(self.progress)
        error = None if position_error is None else np.asarray(position_error)
        return dict(
            velocity_enu=self.velocity.tolist(),
            yaw_rate=float(yaw_rate),
            progress_s=self.progress,
            original_time_s=self.path.authored_time(self.progress),
            reference_position=target.tolist(),
            reference_yaw=target_yaw,
            position_error=None if error is None else error.tolist(),
            position_error_norm=None if error is None else float(np.linalg.norm(error)),
            status=status,
            stop_requested=self.fault is not None,
            fault=self.fault,
        )

    def _stop(self, reason):
        if self.fault is None:
            self.fault = reason
        self.velocity = np.zeros(3)
        return self._result("stopped")

    def update(
        self,
        now,
        dt,
        pose4,
        odom_yaw,
        valid=True,
        *,
        measurement_time=None,
        odom_time=None,
        covariance=None,
    ):
        if self.fault is not None:
            return self._stop(self.fault)
        pose = np.asarray(pose4, dtype=float)
        if (
            not valid
            or pose.shape != (4,)
            or not np.isfinite(pose).all()
            or not np.isfinite([now, dt, odom_yaw]).all()
            or dt <= 0
            or now < self.last_time
            or now - self.last_time > self.max_measurement_age
            or dt > self.max_measurement_age
        ):
            return self._stop("invalid_state_or_timing")
        if (
            measurement_time is None
            or odom_time is None
            or not np.isfinite([measurement_time, odom_time]).all()
            or abs(measurement_time - odom_time) > 1e-6
        ):
            return self._stop("unmatched_exposure")
        age = now - measurement_time
        if age < -1e-6 or age > self.max_measurement_age:
            return self._stop("stale_measurement")
        if covariance is not None:
            cov = np.asarray(covariance, dtype=float)
            if (
                cov.shape != (4, 4)
                or not np.isfinite(cov).all()
                or not np.allclose(cov, cov.T)
                or np.linalg.eigvalsh(cov).min() < -1e-10
            ):
                return self._stop("invalid_covariance")
        self.last_time = float(now)
        target, target_yaw, feedforward, yaw_feedforward = self.path.sample(self.progress)
        error = target - pose[:3]
        frozen = np.linalg.norm(error) > self.max_position_error
        # A frozen target must not retain its moving-reference feedforward.
        if frozen:
            feedforward = np.zeros(3)
            yaw_feedforward = 0.0
        desired_map = self.position_kp * error + feedforward
        alignment = pose[3] - odom_yaw  # map = R(alignment) * odom
        c, s = np.cos(alignment), np.sin(alignment)
        desired_odom = np.array(
            [
                c * desired_map[0] + s * desired_map[1],
                -s * desired_map[0] + c * desired_map[1],
                desired_map[2],
            ]
        )
        desired_odom = limit_norm(desired_odom, self.max_speed)
        change = limit_norm(desired_odom - self.velocity, self.max_acceleration * dt)
        self.velocity += change
        yaw_rate = np.clip(
            yaw_feedforward + self.yaw_kp * wrap_angle(target_yaw - pose[3]),
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        ended = self.progress >= self.path.duration
        status = "settling" if ended else ("paused_tracking_error" if frozen else "tracking")
        if (
            ended
            and np.linalg.norm(error) < 0.01
            and abs(wrap_angle(target_yaw - pose[3])) < 0.01
            and np.linalg.norm(self.velocity) < 0.01
        ):
            status = "complete"
        result = self._result(status, yaw_rate, error)
        if not frozen and not ended:
            self.progress = min(self.path.duration, self.progress + dt)
        return result
