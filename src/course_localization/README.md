# Course localization and logging

The organizer initial 10cm test uses bounded text/JPEG/PNG/PLY logging, with no NPZ or rosbag.
See `docs/odom_10cm_test.md`. The full-course logger described below is separate.

A team-workspace package that runs beside the existing OAK/cuVSLAM stack. The default localization launch publishes localization and diagnostic outputs only. An explicitly enabled, separate course follower is also available; it is bounded velocity feedback, not MPC. The localization node does not arm, command flight, publish to `/fmu/in/*`, replace cuVSLAM, or modify the flight interface. No map-override feature is included.

For setup, startup, self-test, RViz and optional controller commands, see `docs/course_setup.md` in the repository.

## Run

Build this package in the team's overlay, using the existing `team_build.sh` procedure. The camera/cuVSLAM stack must already be running. Dependencies are declared in `package.xml`: ROS messaging/TF, cv_bridge, message_filters, px4_msgs, rosbag2, NumPy, SciPy, OpenCV and trimesh. Install missing declared dependencies with the team's normal rosdep/package workflow; do not overwrite the fleet underlay.

```bash
# Inside the existing skybeta container, with this package in /root/team_ws/src:
team_build.sh --packages-select course_localization
source /root/team_ws/install/setup.bash
ros2 launch course_localization localization.launch.py
```

`config/hardware.yaml` holds the topic names, known launch pose and range mounting offset. The default launch pose comes from the simulated course start, not automatic global localization: set it to the actual marked start and heading. Camera calibration and stereo baseline come from CameraInfo plus the existing TF tree; no simulated camera baseline or pitch is assumed. The downward mount offset is explicitly unmeasured (zero by default), and must be replaced with its measured FLU offset. The approximately 1-degree beam setting follows this repository's hardware notes; it is not the simulator's 27-degree cone.

The supplied map is the photo-reconstructed nominal structural geometry. The filter uses measured stereo depth, cuVSLAM motion, measured IMU attitude/yaw assistance and available downward range. It keeps the selected 800-particle tune. Stereo is rectified on the host and processed at 640x400, yielding 160x100 right-camera axial depth, at a target of 5 updates/s.

If `/fmu/out/distance_sensor` is not exported by the installed PX4 DDS configuration, range fusion is explicitly marked missing. A standard `sensor_msgs/Range` publisher on `/course/downward_range` is also supported. No PX4 parameter or DDS change is made by this package. Standard Range's TF +X must point down and match the configured sensor offset. Invalid, stale, poor-quality and incompatible-clock range data are not treated as valid height. A measured clock offset can be set for PX4 timestamps; the adapter never fabricates an offset from arrival time.

## Quick stationary self-test

Run from the repository root in a ROS-sourced shell while localization is running:

```bash
python3 tools/course_selftest.py --height 0.80
```

Hold the drone level and stationary above a flat floor. `0.80` means a physically measured **sensor-to-floor** distance in metres, not the body height. The default is PX4's downward DistanceSensor; use `--range-source range` for a standard ROS Range publisher. The script observes for ten seconds and prints checks for fresh, ordered streams, source clocks, nonblank stereo images, calibration sizes, paired exposures, usable depth, odometry frames/quaternions, stationary IMU and range accuracy. The default range tolerance is 5 cm (`--tolerance`). Without `--height`, accuracy is explicitly **NOT TESTED**. This is a bench sanity check, not flight certification or a replacement for measured mount calibration.

## Logs: what the sensors and filter saw

Every launch creates a unique run under `~/team_ws/logs/course/`, which is inside the existing persisted team-workspace mount when run in the container. Copy this directory off the drone after each run. Stop normally with Ctrl-C so the bag and writer flush.

- `manifest.json`: resolved parameters, exact copies/hashes of nominal map files and configuration, alignment conventions and logging bounds.
- `rosbag/` and `rosbag.log`: full-rate camera, calibration, IMU, odometry, range, TF and localization outputs. Enabled by default with `record_raw_topics: true`. Bags split at 256 MiB; this splits files but does **not** cap total disk use. Stereo alone can produce several GB per minute uncompressed. No logs are automatically deleted.
- `frames/*.npz`: processed exposures with raw/rectified stereo, measured depth, calibration, camera transform, odometry, IMU/range history, PF pose/covariance, particles/weights and timestamps. Invalid numeric data remain NaN in NPZ; JSON records use null.
- `diagnostics.jsonl`: measurement validity, rejected motion, synchronization waits, depth coverage, processing time, dropped logs and writer errors.
- `points/*.npz`: raw measured body-frame point chunks and explicit transforms/coordinates for both reconstructions.
- `snapshots/*_vo.{ply,npz,json}`: accumulated measured points aligned by **odometry plus the fixed launch prior**, independent of subsequent PF correction.
- `snapshots/*_pf.{ply,npz,json}`: the same measurements aligned by **PF poses**. These are map-conditioned and potentially circular: a good-looking PF cloud does not prove the original map is correct.
- `final_health.json`: final counts of accepted, written, dropped and failed records. Cloud snapshots are voxel-bounded; raw point chunks are retained. Queue/disk failures are visible rather than silently discarded.

The two reconstructed clouds are observations, not a clean mesh or an automatically built replacement map. View the PLY files in CloudCompare or another point-cloud viewer. Preserve the raw observations and calibration to rebuild or realign a map later. The package never overwrites the nominal map.

Live inspection topics: `/course/stereo_depth`, `/course/particle_pose`, `/course/particles`, `/course/observed_points_vo`, `/course/observed_points_pf` and `/course/status`. The point-cloud and pose fixed frame is `course_map`. The depth image is right-rectified optical Z; its exact body transform is archived per frame.

## Code and verification

The selected filter core is under `course_localization/core/`; no simulator or experimental filter variants are imported. `filter_session.py` handles measured filter updates, `sensor_geometry.py` handles calibration/time interpolation, `inputs.py` owns ROS subscriptions, `node.py` publishes/logs, and `recording.py` owns bounded asynchronous storage. Raw full-rate bag recording is isolated in `raw_bag.py`.

Local tests cover core parity, geometry/timing, logging, and an optional exact 155-frame recorded-input regression. They do not establish Jetson throughput, live topic availability, correct physical mounts, or hardware convergence. Tests were exercised in the available ROS Jazzy/Python3.12 container; the target Zephyrus stack is ROS Humble, so the stationary self-test and target build are still required.
