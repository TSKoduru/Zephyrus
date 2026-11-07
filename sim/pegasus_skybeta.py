#!/usr/bin/env python
"""Pegasus standalone app: the skybeta airframe in Isaac Sim.

One Iris multirotor carrying

  * ``PX4MavlinkBackend``  -> PX4 SITL over MAVLink (tcpin, localhost, 4560 + id,
                              lockstep on). This is the flight dynamics link.
  * ``ROS2Backend``        -> the sensor/state topics the skybeta stack consumes.
  * two ``MonocularCamera``s named ``left`` / ``right`` at a 7.5 cm baseline,
                              1280x800 @ 30 Hz, standing in for the OAK-D-S2.

Run it with Isaac Sim's own interpreter (Pegasus v5.1.0 / Isaac Sim 5.1.0):

    isaac_run sim/pegasus_skybeta.py

Everything else -- MicroXRCEAgent, cuVSLAM, the PX4 bridge -- runs in the
``skybeta-sim`` container against ``sim/skybeta_sim.launch.py``. See sim/README.md.

Nothing in this file may be imported outside Isaac Sim: ``SimulationApp`` has to be
constructed before any ``omni``/``pegasus`` import or the simulator crashes.

--------------------------------------------------------------------------------
Two settings MUST agree with sim/skybeta_sim.launch.py or the stereo rig cuVSLAM
solves for is not the one Isaac rendered:

    SKYBETA_BASELINE      <-> launch arg  baseline
    SKYBETA_CAMERA_{X,Z}  <-> launch args camera_x / camera_z
    SKYBETA_CAMERA_{ROLL,PITCH,YAW} <-> camera_roll / camera_pitch / camera_yaw

The launch file carries the same defaults. Change one, change both.
--------------------------------------------------------------------------------
"""

import os

# ---------------------------------------------------------------------------
# Isaac Sim bootstrap. MUST be the first thing that runs.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp


def _env_flag(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0").strip().lower() in (
        "1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def _env_int(name: str, default: int) -> int:
    return int(os.environ.get(name, default))


# Headless is the faster option for a CI-style run, but you cannot see whether the
# camera is upside down, so the default is a visible window.
HEADLESS = _env_flag("SKYBETA_HEADLESS", False)
simulation_app = SimulationApp({"headless": HEADLESS})

# ---------------------------------------------------------------------------
# Everything below here needs the simulator to exist first.
# ---------------------------------------------------------------------------
import carb                                                        # noqa: E402
import numpy as np                                                 # noqa: E402
import omni.timeline                                               # noqa: E402
from omni.isaac.core.world import World                            # noqa: E402
from scipy.spatial.transform import Rotation                       # noqa: E402

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus.simulator.logic.backends.px4_mavlink_backend import (  # noqa: E402
    PX4MavlinkBackend, PX4MavlinkBackendConfig)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend  # noqa: E402
from pegasus.simulator.logic.graphical_sensors.monocular_camera import (  # noqa: E402
    MonocularCamera)
from pegasus.simulator.logic.interface.pegasus_interface import (   # noqa: E402
    PegasusInterface)
from pegasus.simulator.logic.vehicles.multirotor import (           # noqa: E402
    Multirotor, MultirotorConfig)

# ===========================================================================
# Configuration
# ===========================================================================

# --- world -----------------------------------------------------------------
# Any key of pegasus.simulator.params.SIMULATION_ENVIRONMENTS. "Curved Gridroom"
# is the upstream example default; "Full Warehouse" has far more visual texture,
# which matters a great deal for a stereo VO front end. Assets stream from
# NVIDIA's Nucleus server on first load -- expect a slow first run.
ENVIRONMENT = os.environ.get("SKYBETA_ENV", "Curved Gridroom")

# --- vehicle ---------------------------------------------------------------
# MAVLink vehicle id. PX4MavlinkBackend listens on connection_baseport + this id,
# and PX4 SITL's `-i <n>` instance opens 4560 + n. Keep them equal.
MAV_VEHICLE_ID = _env_int("SKYBETA_MAV_VEHICLE_ID", 0)

# ROS 2 vehicle id and namespace. NOTE THE UPSTREAM QUIRK: ros2_backend.py builds
# every topic as `namespace + str(vehicle_id) + "/" + topic`, and the *default*
# namespace is already "drone" + str(vehicle_id). Leaving `namespace` unset with
# vehicle_id=1 therefore yields "drone11/...", not "drone1/...". Passing
# namespace="drone" explicitly (as upstream examples/8_camera_vehicle.py does) is
# what produces the documented "{namespace}{vehicle_id}/" layout.
ROS2_NAMESPACE = os.environ.get("SKYBETA_NAMESPACE", "drone")
ROS2_VEHICLE_ID = _env_int("SKYBETA_ROS2_VEHICLE_ID", 1)

# Spawn pose, ENU, metres. 0.07 m is the upstream Iris ground clearance.
SPAWN_POSITION = [
    _env_float("SKYBETA_SPAWN_X", 0.0),
    _env_float("SKYBETA_SPAWN_Y", 0.0),
    _env_float("SKYBETA_SPAWN_Z", 0.07),
]
SPAWN_YAW_RAD = _env_float("SKYBETA_SPAWN_YAW", 0.0)

# --- PX4 SITL --------------------------------------------------------------
# px4_autolaunch defaults to OFF here, unlike the Pegasus examples. Pegasus's
# PX4LaunchTool runs px4 with cwd=tempfile.TemporaryDirectory(), so
# `param save` writes into a directory that is deleted when the process dies:
# nothing you set at the pxh> prompt survives a restart. The spike needs
# UXRCE_DDS_SYNCT, the EKF2 vision block and EKF2_GPS_CTRL to persist, so
# sim/README.md launches PX4 by hand from a fixed working directory instead.
# Set SKYBETA_PX4_AUTOLAUNCH=1 once you no longer care about persistence.
PX4_AUTOLAUNCH = _env_flag("SKYBETA_PX4_AUTOLAUNCH", False)
PX4_DIR = os.path.expanduser(os.environ.get("PX4_DIR", "~/PX4-Autopilot"))
# Selects ROMFS/px4fmu_common/init.d-posix/airframes/<id>_<model>. Verified
# present in PX4 v1.16.2 as 10015_gazebo-classic_iris; that autostart routes to
# px4-rc.mavlinksim, i.e. `simulator_mavlink start -c 4560`, which is the TCP
# server PX4MavlinkBackend dials into.
PX4_VEHICLE_MODEL = os.environ.get("PX4_VEHICLE_MODEL", "gazebo-classic_iris")

# --- cameras ---------------------------------------------------------------
IMAGE_WIDTH = _env_int("SKYBETA_IMAGE_WIDTH", 1280)
IMAGE_HEIGHT = _env_int("SKYBETA_IMAGE_HEIGHT", 800)
# 30 Hz to match the OAK-D-S2 and the Jetson stack. The ROS2Backend converts this
# into an Isaac "simulation gate" step of int(60 / frequency) against the 60 Hz
# render tick that pegasus.params WORLD_SETTINGS['px4'] sets, so only integer
# divisors of 60 (60, 30, 20, 15, 12, 10, ...) come out at the rate you asked for.
IMAGE_FPS = _env_float("SKYBETA_IMAGE_FPS", 30.0)

# OAK-D-S2 stereo pair: OV9282, 1280x800, ~80 deg horizontal FOV.
HFOV_DEG = _env_float("SKYBETA_HFOV_DEG", 80.0)
BASELINE = _env_float("SKYBETA_BASELINE", 0.075)          # metres, OAK-D-S2 spec

# Camera rig pose in the vehicle body frame (FLU: +X forward, +Y left, +Z up).
# Metres and RADIANS -- the same units and the same frame as the camera_* launch
# arguments in sim/skybeta_sim.launch.py, so the two definitions are directly
# comparable. Rotation order is Rz(yaw) * Ry(pitch) * Rx(roll), matching
# tf2_ros static_transform_publisher.
CAMERA_X = _env_float("SKYBETA_CAMERA_X", 0.10)
CAMERA_Z = _env_float("SKYBETA_CAMERA_Z", 0.0)
CAMERA_ROLL = _env_float("SKYBETA_CAMERA_ROLL", 0.0)
CAMERA_PITCH = _env_float("SKYBETA_CAMERA_PITCH", 0.0)
CAMERA_YAW = _env_float("SKYBETA_CAMERA_YAW", 0.0)

LEFT_CAMERA_NAME = "left"     # -> {ns}{id}/left/color/image_raw, frame_id "left"
RIGHT_CAMERA_NAME = "right"

# Isaac's default 35 mm-equivalent horizontal aperture. Only the ratio
# focal_length / aperture matters; this value is arbitrary but keeps the numbers
# in a range the Isaac UI displays sensibly.
HORIZONTAL_APERTURE_MM = 20.955


# ===========================================================================
# Derived camera intrinsics
# ===========================================================================
# Pinhole, no distortion: Isaac renders an ideal projection, so the sim launch
# sets cuVSLAM's rectified_images=True and there is nothing to undistort.
FX = (IMAGE_WIDTH / 2.0) / np.tan(np.deg2rad(HFOV_DEG) / 2.0)
FY = FX                                                    # square pixels
CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# USD lens attributes that reproduce FX/FY. Isaac Sim 5.1's
# Camera.set_rational_polynomial_properties() -- which is what
# MonocularCamera.start() calls -- IGNORES the `intrinsics` fx/fy it is handed
# (it unpacks them and never uses them) and IGNORES `diagonal_fov` (the `max_fov`
# argument is deprecated). It derives fx/fy from the prim instead:
#     fx = nominal_width  * focal_length / horizontal_aperture
#     fy = nominal_height * focal_length / vertical_aperture
# So the only way to get the OAK-D's field of view is to set the lens itself,
# which _apply_camera_intrinsics() below does once the cameras are live.
FOCAL_LENGTH_MM = FX * HORIZONTAL_APERTURE_MM / IMAGE_WIDTH
VERTICAL_APERTURE_MM = IMAGE_HEIGHT * FOCAL_LENGTH_MM / FY

# 12 OpenCV pinhole coefficients [k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4], all zero.
# This matters for more than fidelity: isaacsim.ros2.bridge.read_camera_info()
# reports distortion_model "plumb_bob" when k4/k5/k6 are zero and
# "rational_polynomial" otherwise. Both are accepted by cuVSLAM 3.2
# (FillCameraDistortion throws std::runtime_error on anything else), but a
# non-zero set would also make rectified_images=True a lie.
ZERO_DISTORTION = [0.0] * 12


def _camera_config(y_offset: float) -> dict:
    """Config dict for one MonocularCamera, mounted at the rig pose + y_offset.

    The rig rotation is applied to the lateral offset so that the two cameras
    stay on a baseline perpendicular to the optical axis when the mount is
    tilted. This is exactly the composition
    ``base_link -> oak_parent_frame -> {left,right}`` that the sim launch file
    publishes as static TFs, so cuVSLAM's extrinsics and Isaac's geometry agree.
    """
    rig_rotation = Rotation.from_euler(
        "ZYX", [CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL], degrees=False)
    offset = rig_rotation.apply(np.array([0.0, y_offset, 0.0]))
    position = np.array([CAMERA_X, 0.0, CAMERA_Z]) + offset

    return {
        # No depth annotator. cuVSLAM is a stereo tracker; it never reads depth,
        # and the extra render target costs GPU time. (Upstream's depth writer is
        # dead code anyway: ROS2Backend keys it off a "depth" entry in the
        # sensor state dict that MonocularCamera.update() no longer writes.)
        "depth": False,
        "position": position,
        # Pegasus feeds this to scipy as Rotation.from_euler("ZYX", ..., degrees=True),
        # i.e. [yaw, pitch, roll] in DEGREES, and then to
        # Camera.set_local_pose(..., camera_axes="world") -- and Isaac's "world"
        # camera convention is +X forward, +Z up. So [0, 0, 0] is a forward-facing,
        # upright camera in the FLU body frame. (Upstream's own default is
        # [0, 0, 180], i.e. rolled 180 deg / upside down. If step 4 of the
        # checklist shows an inverted image, that is the knob.)
        "orientation": np.array([
            np.rad2deg(CAMERA_YAW),
            np.rad2deg(CAMERA_PITCH),
            np.rad2deg(CAMERA_ROLL),
        ]),
        "resolution": (IMAGE_WIDTH, IMAGE_HEIGHT),
        # Also becomes the GraphicalSensor update_rate (MonocularCamera passes
        # config["frequency"] to super().__init__(update_rate=...)).
        "frequency": IMAGE_FPS,
        # Only cx/cy are read out of this by MonocularCamera.start(); see the
        # FOCAL_LENGTH_MM comment. Kept complete and correct so the file documents
        # the intended intrinsics rather than a half-truth.
        "intrinsics": np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]]),
        "distortion_coefficients": ZERO_DISTORTION,
        # Deprecated in Isaac Sim 5.1 (max_fov). Left consistent with HFOV_DEG for
        # documentation value only -- it has no effect.
        "diagonal_fov": float(np.rad2deg(
            2.0 * np.arctan(np.hypot(CX, CY) / FX))),
    }


class SkybetaSimApp:
    """The skybeta airframe, its PX4 link and its ROS 2 sensor bridge."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        # World settings come from pegasus.simulator.params WORLD_SETTINGS['px4']:
        # physics_dt 1/250 s (PX4 SITL's IMU_INTEG_RATE is 250 Hz) and
        # rendering_dt 1/60 s (the tick the camera publish gate divides down).
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS[ENVIRONMENT])

        config_multirotor = MultirotorConfig()

        # --- PX4 MAVLink backend -------------------------------------------
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": MAV_VEHICLE_ID,
            # PX4 SITL's simulator_mavlink opens a TCP *server*; Pegasus connects
            # to it. "tcpin" + baseport 4560 is the upstream default, restated
            # here so the port is greppable.
            "connection_type": "tcpin",
            "connection_ip": "localhost",
            "connection_baseport": 4560,
            # Lockstep: PX4 and the simulator advance in step, so a slow render
            # slows PX4's clock instead of desynchronising it. Required for
            # repeatable runs, and the reason UXRCE_DDS_SYNCT must be 0 (see
            # sim/px4_sitl.env).
            "enable_lockstep": True,
            "px4_autolaunch": PX4_AUTOLAUNCH,
            "px4_dir": PX4_DIR,
            "px4_vehicle_model": PX4_VEHICLE_MODEL,
        })

        # --- ROS 2 backend --------------------------------------------------
        ros2_backend = ROS2Backend(
            vehicle_id=ROS2_VEHICLE_ID,
            config={
                "namespace": ROS2_NAMESPACE,
                # sensors/imu, sensors/mag, sensors/gps, sensors/gps_twist.
                # WARNING: ROS2Backend publishes the IMU in PX4's NED/FRD
                # convention, NOT in a camera or ROS FLU frame. The sim launch
                # mirrors it to /oak/imu/data for topic-name parity only --
                # do not turn cuVSLAM's enable_imu_fusion on against it.
                "pub_sensors": True,
                # The two cameras. Publishing is done by Omniverse Replicator
                # writers attached to each camera's render product, not by the
                # CPU-side frame grab that is commented out in
                # MonocularCamera.update(), so RGB does reach ROS 2. Checklist
                # step 4 exists to prove that on the actual install.
                "pub_graphical_sensors": True,
                # state/pose (ENU), state/twist (body), state/twist_inertial,
                # state/accel -- ground truth, which is how you score the square.
                "pub_state": True,
                # map -> {ns}{id}_base_link, plus the static FLU->FRD child.
                # Deliberately a SEPARATE tree from cuVSLAM's odom -> base_link:
                # the two never share a child frame, so tf2 sees no conflict, and
                # both hang off "map" so RViz overlays truth and estimate.
                "pub_tf": True,
                # We command PX4, not the rotors. Leaving this on would create
                # four Float64 subscriptions nothing ever writes.
                "sub_control": False,
            })

        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ros2_backend,
        ]

        self.cameras = [
            MonocularCamera(LEFT_CAMERA_NAME, config=_camera_config(+BASELINE / 2.0)),
            MonocularCamera(RIGHT_CAMERA_NAME, config=_camera_config(-BASELINE / 2.0)),
        ]
        config_multirotor.graphical_sensors = list(self.cameras)

        Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            MAV_VEHICLE_ID,
            SPAWN_POSITION,
            Rotation.from_euler(
                "XYZ", [0.0, 0.0, SPAWN_YAW_RAD], degrees=False).as_quat(),
            config=config_multirotor,
        )

        self.world.reset()
        self._intrinsics_applied = False
        self.stop_sim = False

    # -----------------------------------------------------------------------
    def _apply_camera_intrinsics(self) -> bool:
        """Overwrite the lens Pegasus configured, once the cameras are live.

        Returns True when it has been done (or can never be done).

        Must happen before ROS2Backend.add_monocular_camera_writter() snapshots
        the CameraInfo, which it does on the first graphical-sensor update that
        reaches the backend. MonocularCamera.update() swallows its first 100
        calls, so at 30 Hz there are ~3 s of slack -- but this runs on the very
        first step after start(), so the margin is never used.
        """
        if not all(getattr(c, "_camera_full_set", False) for c in self.cameras):
            return False

        for cam in self.cameras:
            prim = cam._camera
            # maintain_square_pixels=False: we set both apertures explicitly and
            # do not want the first call to rewrite the second.
            prim.set_horizontal_aperture(
                HORIZONTAL_APERTURE_MM, maintain_square_pixels=False)
            prim.set_vertical_aperture(
                VERTICAL_APERTURE_MM, maintain_square_pixels=False)
            prim.set_focal_length(FOCAL_LENGTH_MM)
            # Write fx/fy/cx/cy into the OmniLensDistortionOpenCvPinholeAPI
            # attributes as well. read_camera_info() reads *these* -- not the
            # focal length -- so this is what lands in
            # {ns}{id}/{name}/color/camera_info.k.
            prim.set_opencv_pinhole_properties(
                cx=CX, cy=CY, fx=FX, fy=FY, pinhole=ZERO_DISTORTION)

        carb.log_warn(
            "[skybeta] camera intrinsics applied: "
            f"{IMAGE_WIDTH}x{IMAGE_HEIGHT} fx={FX:.2f} fy={FY:.2f} "
            f"cx={CX:.1f} cy={CY:.1f} hfov={HFOV_DEG:.1f}deg "
            f"baseline={BASELINE:.4f}m")
        return True

    # -----------------------------------------------------------------------
    def run(self):
        carb.log_warn(
            "[skybeta] PX4 MAVLink: tcpin:localhost:"
            f"{4560 + MAV_VEHICLE_ID}  autolaunch={PX4_AUTOLAUNCH}")
        carb.log_warn(
            "[skybeta] ROS 2 topics under /"
            f"{ROS2_NAMESPACE}{ROS2_VEHICLE_ID}/  "
            f"(cameras: {LEFT_CAMERA_NAME}, {RIGHT_CAMERA_NAME})")

        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            if not self._intrinsics_applied:
                self._intrinsics_applied = self._apply_camera_intrinsics()
            self.world.step(render=True)

        carb.log_warn("[skybeta] closing Isaac Sim.")
        self.timeline.stop()
        simulation_app.close()


def main():
    SkybetaSimApp().run()


if __name__ == "__main__":
    main()
