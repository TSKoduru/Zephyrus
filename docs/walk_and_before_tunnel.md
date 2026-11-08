# First: walk test, live in the existing SSH logs

Build/source the updated overlay. Both `competitor_example` and
`course_localization` must resolve to the new overlay, not the old underlay.

```bash
cd ~/team_ws
colcon build --packages-select course_localization competitor_example skybeta_flight_interface --symlink-install
source install/setup.bash
ros2 launch competitor_example competitor.launch.py
```

Expected banner: **WALK TEST ONLY**. This default starts only localization and
compact logging: no flight interface, no mission, no controller, no arming.
Stop any previously running competitor/controller first. Keep the aircraft disarmed
and props off while carrying it; this launch does not disable other running nodes.
The existing OAK/cuVSLAM stack must already be running.

The same SSH launch output prints a small map at 2 Hz:

- `#`: estimated drone center.
- `.`: particle positions projected onto the floor plane (overlapping particles share a cell).
- `|`, `-`, `/`, `\`: nominal map walls sliced at 0.8 m, not live detected walls.
- Above the map: XYZ, yaw, particle count, freshness; map +Y is up and +X is right.
- `STALE / WAITING`: no fresh estimate; never interpret the old panel as live tracking.

Start near the map's marked S and align the drone with the configured initial heading.
`config/hardware.yaml` contains the known launch prior; this is not global relocalization
from any arbitrary starting position. Walk slowly through identifiable corners and
compare the `#` with your actual location. A tight cloud can still be confidently wrong.
Check stationary drift, direction of movement, height response and tracking through turns.
No extra SSH session or display command is required. An optional full-screen read-only
viewer remains available as `ros2 run course_localization ascii_view`.

Logs remain bounded per run: no raw bag/NPZ output, rolling text, 60 preview sets and
two map snapshots. For failures send `mission_transitions.jsonl`, `events.jsonl`
(and `.1` backups), `diagnostics.jsonl` and `manifest.json`. Old runs are retained.

# Separate before-tunnel flight implementation

```bash
ros2 launch course_localization before_tunnel.launch.py
```

This starts the flight interface, PF localization and feedback follower, but control
is **disabled by default**. `enable_control:=true` explicitly enables the pilot-triggered
flight. Do not start it concurrently with the walk launch, 10cm mission or another
setpoint publisher. The walk test is the first test; this is not a declaration of flight readiness.

The route is `trajectory/before_tunnel.npz`: about 8.51 m, about 45 s after trimming
its initial hold, with endpoint (2.00, 1.70, 0.116) m in the course map. It follows the
first section, moves slightly away from a plywood brace, then descends and lands
outside the tunnel. It does not enter the tunnel. The reference speed is at most
0.25 m/s; both controller and interface cap the 3D commanded speed at **0.3048 m/s
(1 ft/s)**. This is a command limit, not a guarantee on actual speed under disturbance.
The controller is bounded velocity feedback, not MPC. It uses PF/map position and
PX4 heading to express velocity commands in PX4's local axes, avoiding the initial
test's direct absolute cuVSLAM-position-to-PX4-position conversion.

At the final low approach, vertical velocity is 0.08 m/s downward until PX4 reports
landed; it then requests HOLD and does not restart. The pilot disarms. Landing timeout,
excess lateral drift, stale sensors, missing range/IMU, or lost Offboard request HOLD;
the pilot must land on faults. Missing range/IMU is not bypassed to force motion.
The previously uploaded NPZ frames had no valid range; verify that input first.

## Verification limits

See `before_tunnel/geometry_report.json` and `before_tunnel/route.png`.
Nominal mesh/box checks found no overlap for a level 32 cm-radius drone plus small
sampling margin. Landing remains outside the modeled entrance. Software tests cover
velocity limits, endpoint sequencing and touchdown in three ideal kinematic cases.
These are not drone dynamics or hardware flight tests. An additional 10 cm position-error
clearance check failed in narrow sections; see `clearance_stress_report.json`.
The model is photo-reconstructed, and floor/cage net and detailed articulated body
collisions are not included in that geometry check. The route is **not verified safe
under 10 cm localization/tracking error**. Walking the cage must establish localization
quality before flight. The previous 1.43 m target discrepancy is still unexplained.

The original short flight remains explicitly available as
`ros2 launch competitor_example odom_10cm.launch.py`; it is no longer the default.
