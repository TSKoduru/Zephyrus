# CURRENT DEFAULT — initial odometry-only 10 cm flight test

```bash
ros2 launch competitor_example competitor.launch.py
```

This is also the default used by `run_competitor.sh` and `run_full_stack.sh`.
It launches the flight interface, stereo map recorder, and **only** the initial test.
**UP 35 cm, hold, FORWARD 10 cm, hold, DOWN at 8 cm/s until PX4 reports landed.**
Forward follows the drone's starting heading; start on the ground facing clear space.
Uses existing cuVSLAM `/visual_slam/tracking/odometry`. No particle filter, course route,
or MPC runs. The pilot arms and selects the existing Offboard workflow; this mission
never requests arming or mode changes. Pilot disarms after landing. No automatic retry.

Build and source the updated packages before using the default command:

```bash
cd ~/team_ws
colcon build --packages-select course_localization skybeta_flight_interface competitor_example --symlink-install
source install/setup.bash
```

Deploy all THREE updated packages (including the launch and interface fix), not just
course_localization. The camera/cuVSLAM stack must already be running. Stop other
mission/setpoint publishers. For a stationary logging check use `enable_motion:=false`.
The mission waits for fresh odometry, flight state and recorded stereo mapping.
A tracking/logging fault stops the mission and requests HOLD; pilot takes over.

Logs: `~/team_ws/logs/odom_10cm/<run>/`, including raw rosbag, stereo/calibration,
measured depth, and accumulated `snapshots/*_vo.ply` maps in the odometry frame.
Empty `_pf.ply` files are recorder schema artifacts; no PF is running.
Use Ctrl-C to flush logging after the test.
RViz: fixed frame `odom`, PointCloud2 `/odom_test/observed_points`,
Image `/odom_test/stereo_depth`. Status: `/odom_test/mission_status` and
`/odom_test/map_status`.

Software smoke checks are not a hardware flight validation.
