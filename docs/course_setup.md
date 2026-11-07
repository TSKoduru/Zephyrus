> **Initial test:** the organizer default now runs the [odometry-only 10 cm test](odom_10cm_test.md). The full-course instructions below are separate.

# Course setup, startup, self-test and RViz

This change adds a separate `course_localization` team package. It does not replace the OAK/cuVSLAM stack or change PX4 settings. You deploy the repository yourself. Start with localization/logging only; the optional course controller is disabled by default.

## 1. Install the team package

On your drone, copy only the new package into the persisted team workspace. Do not overlay the entire Zephyrus `src/` tree and shadow the fleet packages.

```bash
mkdir -p ~/team_ws/src/course_localization
rsync -a ~/Zephyrus/src/course_localization/ ~/team_ws/src/course_localization/
```

Use the actual checkout path if it is not `~/Zephyrus`. The existing fleet stack should be running. Enter its container (default name):

```bash
docker exec -it skybeta-stack bash
source skybeta_env.sh
```

Install any missing dependencies from `course_localization/package.xml` using the team's normal rosdep/package procedure. The new Python dependencies are SciPy, OpenCV and trimesh, alongside NumPy. The target already provides PX4 messages and ROS; do not install a different `px4_msgs` branch. RViz requires the `rviz2` package and a graphical display. Then:

```bash
team_build.sh --packages-select course_localization
source /root/team_ws/install/setup.bash
```

The package has been built and tested locally in the available ROS Jazzy container. The drone uses Humble; its build and bench checks are still required.

## 2. Check the launch assumptions

Edit `/root/team_ws/src/course_localization/config/hardware.yaml`:

- Set `initial_mean4` to the real marked start: map x/y, body height and yaw in radians. This is a known-start filter, not automatic global map alignment.
- Set `range_position_body_m` to the measured sensor offset in body FLU coordinates. The default zero is explicitly an unmeasured approximation.
- Confirm topics, frame names and timestamps. Camera mounting and stereo calibration come from the existing TF/CameraInfo, not simulator constants.
- PX4 range is read from `/fmu/out/distance_sensor`. If this topic is absent, range is logged as missing; this package does not change PX4 DDS configuration. A standard `/course/downward_range` Range publisher is also supported.
- Keep the drone stationary while the IMU initializes. Clock-mismatched sensor data are rejected rather than automatically shifted.

## 3. Start localization and logging

In the sourced container shell:

```bash
ros2 launch course_localization localization.launch.py
```

For an edited source configuration, pass it explicitly:

```bash
ros2 launch course_localization localization.launch.py \
  config:=/root/team_ws/src/course_localization/config/hardware.yaml
```

This launch publishes no flight commands. Inspect health in another sourced shell:

```bash
ros2 topic echo /course/status
```

Look for increasing `frames`, a recent frame, empty `last_error`, valid range/IMU, a running raw bag, and no recorder failures/drops. Logs go to a unique directory inside `/root/team_ws/logs/course/`, persisted on the host as `~/team_ws/logs/course/`.

## 4. Ten-second stationary self-test

In a ROS-sourced shell with access to this checkout:

```bash
python3 /path/to/Zephyrus/tools/course_selftest.py --height 0.80
```

Measure the **downward sensor itself** 0.80 m above a flat floor; hold the drone level and still. Substitute the actual measured height. The script checks fresh/ordered streams, compatible clocks, nonblank stereo, calibration dimensions, paired exposures, usable depth, odometry frames/quaternions, stationary gravity and range accuracy (default tolerance 5 cm).

If the tool is outside the container, copy just the script into the team workspace and run `/root/team_ws/course_selftest.py`. Use `--range-source range` for a standard ROS Range publisher. Without `--height`, range accuracy explicitly reports **NOT TESTED**. A pass does not prove global localization or flight performance.

## 5. RViz

In a graphical ROS shell on the same machine/DDS domain:

```bash
rviz2 -d "$(ros2 pkg prefix --share course_localization)/config/course.rviz"
```

Or start it with localization:

```bash
ros2 launch course_localization localization.launch.py rviz:=true
```

The existing `host/run.sh` forwards X11 when started from a graphical session with `DISPLAY` set. A headless container may not have display access. For laptop RViz, follow the existing [DDS instructions](competitor_guide.md#your-drones-dds-domain); do not guess or change the drone's domain ID. The default fleet transport is localhost-only, so an ordinary laptop session will not discover it.

The supplied layout uses fixed frame `course_map`:

- **Grey:** nominal structural map (`/course/nominal_map`).
- **Cyan:** measured points placed using odometry and the fixed initial alignment.
- **Pink, initially disabled:** the same observations placed using PF estimates. This view is map-conditioned, not independent proof of the map.
- **Yellow arrows / pose:** particles and PF estimate.
- **Depth panel:** measured right-rectified stereo depth; black/invalid areas are not filled with perfect geometry.

If a display is blank, check its topic and `/course/status` first. A missing calibration/TF or stale odometry prevents processing deliberately.

## 6. Optional closed-loop course follower

This is bounded velocity feedback, **not MPC**. It combines reference velocity with map-relative position error, freezes reference progress when tracking lags, and limits speed (0.3 m/s), acceleration (0.8 m/s²) and yaw rate (0.6 rad/s). The original simulation's stationary warmup is removed; the actual path is slowed uniformly, without changing its positions. There is no new online obstacle avoidance or flight validation.

The follower uses synchronized PF and PX4 attitude to express commands in PX4 local ENU. It separately checks visual odometry; it does not assume VO and PX4 yaw origins match. PX4 executes the existing inner control loop. Wrong localization still produces wrong feedback, even with a well-behaved controller.

First inspect it disabled:

```bash
ros2 launch course_localization controller.launch.py
ros2 topic echo /course/controller_status
```

It will report all readiness gates and publish no commands. Before an explicit trial, stop the example square mission or any other setpoint publisher while retaining one flight interface. If stopping the example launch also stops its interface, restart the existing launch **without** its mission:

```bash
ros2 launch competitor_example competitor.launch.py competitor:=false
```

Do not run a second interface alongside the original. Then, for an operator-controlled trial:

```bash
ros2 launch course_localization controller.launch.py enable_control:=true
```

The enabled controller primes with zero velocity only after its gates pass. It does not arm or request Offboard; the existing pilot/flight-interface handover remains in charge. Fresh localization, acceptable covariance, valid IMU/range, synchronized PX4 attitude and no competing publisher are required. Faults stop twist publication and request `/skybeta/hold`; the interface's existing velocity timeout remains a fallback. Offboard loss latches the controller. A frozen interface is never automatically unfrozen: disarm and explicitly reset/restart the interface before another trial. Test the handover on the bench before flight, as required by the repository's existing rules.

## 7. Stop and inspect the records

Stop the controller first, then localization with Ctrl-C; allow the recorder and bag to flush. Keep the entire run directory:

- `manifest.json`: parameter, source, trajectory and nominal-map copies/hashes.
- `rosbag/`: full-rate sensor, TF, localization and controller/setpoint streams.
- `frames/*.npz`: images, depth, calibration, motion, range/IMU, PF state and weights.
- `points/*.npz`: measured point chunks with both explicit alignment transforms.
- `snapshots/*_vo.ply` / `*_pf.ply`: accumulated clouds for CloudCompare or similar tools.
- `diagnostics.jsonl` / `final_health.json`: processing, missing data, queue drops and disk errors.

Raw bags split at 256 MiB but total storage is not capped; uncompressed stereo can use several GB/minute. Cloud previews have a voxel cap, while raw chunks remain. No map is automatically replaced and no logs are automatically deleted. See the package [README](../src/course_localization/README.md) for format details.
