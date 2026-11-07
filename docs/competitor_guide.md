# skybeta competitor guide

**Read the [rules](../README.md#rules) first.** SSH into another team's drone and you will be
disqualified. How to log in to yours: [connect to your drone](../README.md#connect-to-your-drone).

You have a drone that already knows where it is. Your job is to decide where it should go.

The stack below you — OAK-D-S2 → cuVSLAM → PX4 EKF2 over uXRCE-DDS — publishes a
GPS-denied pose at ~30 Hz. Everything between your setpoints and the flight controller is
owned by `skybeta_flight_interface`, which you should not need to modify.

There is **always a safety pilot on RC**. They arm the vehicle, and arming is what starts your
code flying. They can take it back at any instant. Nothing you write may assume otherwise.

> **What has and has not been proven.** Tested on the bench: the full VIO stack, position
> estimates reaching the flight controller, and Position mode becoming available on drone 16.
> **Not yet flown on any airframe:** a hover on VIO, takeoff under Offboard, and the
> arm-then-autonomy handover in §4. Your first runs are the test. Do them with the props off,
> then low and tethered, and assume something will need adjusting.

---

## 1. The contract

Four topics. That is the whole interface.

| Topic | Type | Direction | Frame / units |
|---|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | **you subscribe** | ENU position + orientation in `odom`; body-frame FLU twist; metres, m/s, rad |
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | **you publish** | ENU position in `odom`, metres; orientation = commanded yaw |
| `/skybeta/setpoint/twist` | `geometry_msgs/TwistStamped` | **you publish** | ENU world velocity, m/s; `angular.z` = yaw rate about ENU up, rad/s |
| `/skybeta/state` | `std_msgs/String` (JSON) | **you subscribe** | what PX4 is doing — see §3 |

Two optional services:

| Service | Type | What it does |
|---|---|---|
| `/skybeta/hold` | `std_srvs/Trigger` | Freeze at the last position the interface considered safe. Always available. |
| `/skybeta/request_offboard` | `std_srvs/Trigger` | Ask PX4 for Offboard. **Refused unless `allow_mode_request:=true`. SITL only.** |

Do not publish to `/fmu/in/*` yourself. Two publishers on `offboard_control_mode` fight
over the control flags and PX4 sees whichever arrived last.

### Frames — read this twice

`odom` is **ENU**: `+x East, +y North, +z Up`.

...except cuVSLAM has no compass and no gravity vector (`enable_imu_fusion` is off, so this
is pure stereo VO). "East" and "North" actually mean *whatever direction the vehicle's x and
y axes pointed when the SLAM node started*. The origin is the rig's startup pose, so at
power-on the vehicle is at `(0, 0, 0)` and `z` is height above that point. **Do not assume
magnetic north anywhere in your code.**

PX4 is **NED**: `+x North, +y East, +z Down`. The flight interface does that conversion:

```
position:   (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)     # world: permute x/y
yaw:        yaw_ned = 90 deg - yaw_enu                          # its own inverse
yaw rate:   yawspeed_ned = -yawspeed_enu                        # z flips
```

You never write those. If you find yourself negating a `z`, you are working in the wrong
frame.

**Yaw is optional.** Leave the setpoint's orientation quaternion all-zero and the interface
sends `yaw = NaN`, which tells PX4 to hold whatever heading it has. Set a real (normalised)
quaternion to command one.

**Pose setpoints persist; twist setpoints expire.** A position setpoint *is* a hold, so it
stays in effect until you replace it. A velocity setpoint is a rate command that is only
safe while someone is actively refreshing it, so it expires after `setpoint_timeout_s`
(default 1.0 s) and the interface falls back to holding the last safe position. If your node
dies mid-manoeuvre the vehicle hovers; it does not keep flying.

### The one frame caveat nobody can fix for you

PX4's local NED frame is EKF2's, whose origin and heading are **not** the same as the vision
`odom` frame's. With `pose_frame=FRD` on the vision message, EKF2 estimates that rotation
itself, and your ENU setpoint is rotated by the plain REP-103 map with no correction. In
practice the residual is a constant yaw offset and a constant translation.

Two parameters exist for this once you have measured it on the bench:

- `anchor_setpoints` (default `false`) — express setpoints as a displacement from the vision
  origin, re-anchored on the PX4 local position captured at the same instant. Fixes the
  translation.
- `odom_to_ned_yaw_offset_rad` (default `0.0`) — fixes the rotation.

Measure before you set either:

```bash
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.orientation
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort \
                                          --qos-durability transient_local
```

If the two yaws differ by a constant, that constant is your offset. Fly a hover with both at
zero first — you may not need them at all.

---

## 2. The safety envelope

Every setpoint is **clamped, never rejected**. Rejecting would leave PX4 holding whatever it
had last, which for a velocity setpoint means a vehicle still flying at the command that took
it out of bounds. Clamping keeps the stream continuous and pins the commanded state to the
boundary.

| Parameter | Default | Meaning |
|---|---|---|
| `max_altitude_m` | `3.0` | Ceiling above the origin captured at the **first odometry sample** |
| `min_altitude_m` | `0.3` | Floor above that origin — stops you commanding into the ground |
| `max_speed_mps` | `1.5` | Cap on the **norm** of a twist setpoint, not per-axis. **Pose setpoints are not speed-limited** |

**There is no horizontal geofence** (removed in 0.3.0). A position setpoint's x and y go to
PX4 exactly as you wrote them, however far out they are: keeping the vehicle inside the
room or the netting is your code's job and the safety pilot's. Only the altitude is clipped.
Each clamp logs a throttled WARN and shows up in `/skybeta/state`.

**A far-away pose is a fast flight.** PX4 flies a position setpoint at its own limit, and on
these airframes that is `MPC_XY_VEL_MAX` = 12 m/s, more than a room and more than VIO tracks
without motion blur. Send waypoints a short step ahead of the vehicle, or use twists, which the
speed cap does cover.

**A clamped setpoint you never notice looks exactly like a vehicle that will not reach its
waypoint.** If your mission hangs at a corner, check `clamped` in `/skybeta/state` before you
check anything else. The example node logs this for you.

The 5 m square in `competitor_example` reaches √2 × 5 = 7.07 m from the start at its far
corner, and nothing stops a bigger one: size it to the space you fly in.

Tighten the envelope from the launch file; do not loosen it without asking:

```bash
ros2 launch competitor_example competitor.launch.py \
    max_altitude_m:=2.0 square_side_m:=3.0
```

---

## 3. `/skybeta/state`

A JSON string at 5 Hz. Parse it with `json.loads(msg.data)`.

```json
{
  "stamp": 1234.567,
  "px4_link": true,
  "armed": true,
  "nav_state": 14,
  "nav_state_name": "OFFBOARD",
  "offboard_active": true,
  "odom_enu":   [0.12, -0.04, 1.50],
  "origin_enu": [0.00,  0.00, 0.00],
  "local_enu":  [0.11, -0.05, 1.49],
  "envelope_ok": true,
  "setpoint_mode": "position",
  "clamped": [],
  "frozen": false,
  "envelope": {"min_altitude_m": 0.3, "max_altitude_m": 3.0,
               "max_speed_mps": 1.5}
}
```

- `offboard_active` — **wait for this before you command anything.** It goes true once the
  flight interface has entered Offboard (it asks when the pilot arms and then switches to Position, §4),
  and false the instant the pilot takes over.
- `odom_enu` is cuVSLAM's position; `local_enu` is EKF2's own estimate reported back in ENU.
  They should track each other. If they diverge, the vision link is degrading.
- `envelope_ok` describes the **vehicle**, not the setpoint: false while the vehicle is above
  `max_altitude_m` (carried there by momentum or by the pilot). A vehicle on the pad, or being
  landed, is legitimately below `min_altitude_m` and is not reported as a breach. (Before
  0.3.0 this field was `geofence_ok` and also covered a 10 m radius.)
- `clamped` lists the reasons the last setpoint was modified: `max_altitude`,
  `min_altitude`, `max_speed`.

---

## 4. RC, arming and autonomy — the procedure

**The flight interface never arms.** There is no arm service. The pilot arming, **then switching to
Position** on the ground, is the go signal: `flight_interface` then asks PX4 for Offboard, once, and
your node flies the vehicle, **including the takeoff**. There is no manual takeoff and no RC switch for Offboard.

Offboard is PX4's name for "take commands from the Jetson". In Position, PX4 follows the sticks and
ignores your setpoints.

Before every flight:

1. Props off for the first run of any new code. Bench it.
2. Safety pilot has the transmitter, is standing clear, and has confirmed the kill switch.
3. The stack starts by itself at power-up (§5). **Keep the drone still for the first minute**:
   position and heading are measured from where it was when it started.
4. Confirm on the ground station:
   - `/skybeta/state` shows `px4_link: true` and a non-null `local_enu`,
   - `/visual_slam/tracking/odometry` is publishing and `odom_enu` moves sensibly when you
     carry the vehicle,
   - your node is publishing at least once a second.
5. **Pilot arms** with the mode switch on Altitude, throttle low.
6. **Pilot switches to Position**, within a few seconds, throttle still low.
7. `offboard_active` goes true, and your node takes off.

**It leaves the ground as soon as Offboard starts.** Every pose is floored at `min_altitude_m`, so
any pose lifts the vehicle to at least 0.3 m, even one that says "stay where you are". A twist
takes off only if it has an upward component. If the vehicle has not taken off 10 s after arming, PX4
disarms it (`COM_DISARM_PRFLT`), and that clock includes the time before the switch to Position.

Offboard is requested only when all of these hold at the moment of the switch to Position. If one
does not, the vehicle stays in Position under the pilot, and `flight_interface` logs
`auto offboard: staying out of Offboard: <reason>`.

| Condition | Why |
|---|---|
| Armed **first**, then switched to Position | Arming with the switch already on Position is how a pilot flies with no autonomy |
| PX4 reports the vehicle **landed** | Flipping to Position in the air is a pilot asking for position hold |
| PX4 has a valid local position | With no VIO there is nothing to fly on |
| A setpoint from your node in the last second | No team code, no autonomy |
| `/skybeta/hold` not in force | |
| The interface saw the vehicle disarmed first | A restart in flight never seizes the vehicle |

Each arming gets **one attempt**. Once Offboard has ended, for any reason, it does not come back
until the vehicle is disarmed and armed again. Your node must notice and stop advancing its mission
(the example does).

### Taking the vehicle back

The pilot has three ways, and should practise them with the props off.

| Action | Result |
|---|---|
| **Raise the throttle towards centre**, or move any stick quickly | Position (Altitude if position is lost). **The normal way**: it also leaves the throttle where Position holds height |
| Flip the mode switch to Altitude or Manual | That mode. PX4 only reacts when the switch **moves**, so it is already "on Position" during autonomy and must go somewhere else |
| Kill switch (channel 5) | Motors stop, and the vehicle falls |

**Watch the throttle.** It sits low during autonomy, and in Position or Altitude low throttle means
descend. Flipping the mode switch without raising it brings the vehicle down. The same happens
if your setpoint stream stalls: 1 s later (`COM_OF_LOSS_T`) PX4 drops to Position
(`COM_OBL_RC_ACT`) and sinks unless the pilot acts. The pilot lands the vehicle; the interface
has no land command.

Losing the RC link triggers PX4's failsafe, in Offboard as well (`COM_RCL_EXCEPT` = 0).

To fly with no autonomy at all, arm with the switch already on Position, or launch with
`auto_offboard_on_arm:=false`. Flying off in Altitude and flipping to Position in the air is also safe.

`/skybeta/request_offboard` exists for SITL, where there is no pilot. It is refused unless
`allow_mode_request:=true`, and even then it only asks — it never arms.

PX4 will not enter Offboard unless the `OffboardControlMode` stream is already running, which the
flight interface does at 10 Hz from the moment it starts, before any setpoint of yours exists.

### What you are flying on

**VIO is the only horizontal position source.** No airframe has GPS or optical flow; the one
downward laser rangefinder gives height. If cuVSLAM loses tracking (dark, a blank floor or wall, a
covered lens, fast motion) PX4 has no horizontal position, and the vehicle falls back to Altitude
behaviour and drifts. Fly over texture, in good light, slowly. Height comes from the rangefinder,
with the barometer as backup; the sensor's limit is 4 m, less in bright light.

---

## 5. Running it

**The stack starts by itself when the drone powers up** (`skybeta-stack.service`), because the
flight controller needs VIO for Position mode. You normally never start it. It has four tmux windows:
`agent`, `slam`, `bridge` and `competitor`.

```bash
docker exec -it skybeta-stack tmux attach -t skybeta   # look at it; Ctrl-b d to leave
docker rm -f skybeta-stack                             # restart it; it is back in about 10 s
sudo systemctl stop skybeta-stack                      # stop it until the next boot
```

`host/run.sh stack` refuses while the service runs. Use it only after
`sudo host/autostart.sh disable`, or on a drone without autostart.

**Restart only on the ground and keep the drone still**: a restart re-zeroes the VIO origin. If
`agent`, `slam` or `bridge` dies, the service restarts the whole stack. If your `competitor` window
dies, nothing restarts: that is deliberate, so team code cannot take position hold down with it.

The `competitor` window builds your team workspace (below), then runs `ros2 launch
competitor_example competitor.launch.py`, which starts `flight_interface` **and** the example node.
Point it at your own package by setting `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` in
`/etc/skybeta/drone.env`, then restart the stack. **Your launch file must start `flight_interface`
with `auto_offboard_on_arm:=true`**, or the vehicle stays in Position; copying the example's does.

To work by hand, first stop what the `competitor` window is running (attach, select it, Ctrl-C).
**Two flight interfaces fight over the control flags.** Then, in a shell on the running container
(`host/run.sh shell` attaches to the running stack):

```bash
host/run.sh shell
ros2 launch competitor_example competitor.launch.py competitor:=false   # interface only
ros2 run my_team_nav my_node
```

### Writing your own

**Your code lives in `~/team_ws/src` on the drone** — the team workspace. `host/run.sh`
bind-mounts it at `/root/team_ws` in every container, so it survives the container exiting
(every container here is `--rm`) and survives the organisers updating the skybeta image. Code
anywhere else inside a container — including `/root/skybeta_ws/src` — is gone the moment the
container stops.

Start from the example:

```bash
# on the drone (host shell)
cp -r ~/skybeta/src/competitor_example ~/team_ws/src/my_team_nav
# ...rename the package in package.xml / setup.py / setup.cfg / resource/, then:
```

Fork `square_mission.py` inside it. It is heavily commented and does nothing you will not
need: wait for Offboard, capture a start pose, publish ENU setpoints on a timer, check arrival
with `math.dist`, advance a state machine, notice when Offboard is lost.

Keep `flight_interface` in your launch file exactly as `competitor.launch.py` has it.

Build inside the container — the stack's `competitor` window does this automatically on
start:

```bash
team_build.sh                                  # whole workspace
team_build.sh --packages-select my_team_nav    # any colcon arguments
```

It builds `/root/team_ws` as an overlay on the skybeta workspace and hands the files back to
your host user. A Python package built with `--symlink-install` (the default here) picks up
edits on the next `ros2 run` with no rebuild. New shells source the overlay automatically.

**Do not put `px4_msgs` in your workspace.** PX4's own tutorials tell you to clone it; here
the image already carries the generation that matches the flight controller (`release/1.16`
for v1.16.2), and a different one connects, lists every topic, and moves no data.
`team_build.sh` warns about any package that shadows one the image already has. The VIO → FC
pipeline (agent, cuVSLAM, bridge) never loads your overlay, so a mistake here can break your
code but not the drone's state estimate.

#### Extra system or Python packages

Never `apt install` or `pip install` in a running container — it vanishes on exit. Put them
in a team image instead, layered on the pinned skybeta base:

```bash
host/run.sh build --init     # writes ~/team_ws/Dockerfile + .dockerignore
# edit ~/team_ws/Dockerfile: RUN apt-get install ... / RUN pip3 install ...
host/run.sh build            # builds skybeta-team:<VERSION>
docker rm -f skybeta-stack   # restart the stack: it uses the team image from now on
```

When the organisers roll out a new skybeta version, run `host/run.sh build` again: the team
image is tagged per base version, and the stack falls back to the plain base (with a warning)
until you rebuild.

#### Your drone's DDS domain

Each drone has its own `ROS_DOMAIN_ID`, set in `/etc/skybeta/drone.env`, and by default its DDS
traffic never leaves the Jetson. Everything started through `host/run.sh` gets both
automatically. The flight controller's `UXRCE_DDS_DOM_ID` must match it; the organisers set
that, so do not change `ROS_DOMAIN_ID` by yourself. To use rviz2 or `ros2 topic echo` from a laptop, set
`SKYBETA_DDS_SCOPE=network` in `drone.env`, restart the stack, and export the same
`ROS_DOMAIN_ID` on the laptop. Switch it back before flying: in `network` scope, the camera
streams go over the WiFi.

#### Camera position

The camera's position and angle on the airframe live in `drone.env` as `CAMERA_X/Y/Z` and
`CAMERA_ROLL/PITCH/YAW` (metres, radians), and are measured by the organisers. Those values are what
turn camera motion into vehicle motion: a wrong mount angle makes the vehicle orbit its target. If
you remount the camera, measure it again and restart the stack.

### Recording a bag

Do this for every flight. It is the only way to explain a bad one afterwards.

```bash
host/run.sh bag                        # writes mcap into ./bags on the host
```

Or by hand from a container shell:

```bash
ros2 bag record -s mcap -o /bags/$(date +%Y%m%d-%H%M%S) \
  /oak/left/image_raw /oak/right/image_raw /oak/imu/data \
  /visual_slam/tracking/odometry /visual_slam/status \
  /skybeta/setpoint/pose /skybeta/setpoint/twist /skybeta/state \
  /fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint \
  /fmu/in/offboard_control_mode /fmu/out/vehicle_local_position
```

Record from a *second* container (`host/run.sh bag` does), not from the stack container —
recording competes for CPU with cuVSLAM, and the camera topics are the expensive ones.

---

## 6. Debugging

| Symptom | First thing to check |
|---|---|
| Nothing moves after the pilot arms | `flight_interface`'s log: `auto offboard: staying out of Offboard: ...` gives the reason (§4). Then `/skybeta/state` → `px4_link`. If false, the uXRCE agent is not talking to the FC |
| Armed, reason is clear, still no Offboard | Is `flight_interface` running? `ros2 topic hz /fmu/in/offboard_control_mode` must show ~10 Hz. |
| Mission hangs at a waypoint | `clamped` in `/skybeta/state`. You are asking for something outside the envelope. |
| Position drifts / orbits | cuVSLAM tracking. `ros2 topic echo /visual_slam/status`; a `vo_state` of 2 is lost tracking. |
| `odom_enu` and `local_enu` diverge | The EV link into EKF2 is dropping. Check the TELEM2 cable seating before anything else. |
| Disarms by itself about 10 s after arming | It never took off: Offboard never started, or you sent only horizontal twists (`COM_DISARM_PRFLT`) |
| Dropped to Position or Altitude mid-flight | Your setpoint stream stalled, the pilot's sticks moved, or VIO lost tracking. It will not re-enter Offboard until you disarm |
| Vision stopped reaching the FC soon after power-up | The Jetson has no clock battery, and NTP stepped the clock. The service restarts the stack when that happens; a stack you started by hand needs a restart, once `timedatectl` says synchronized. Needs WiFi with internet |
| Setpoints ignored after your node pauses | You were sending twists. They expire after 1 s by design. Send poses, or keep the twist stream up. |

Useful one-liners:

```bash
ros2 topic hz /fmu/in/offboard_control_mode          # must be ~10 Hz
ros2 topic hz /visual_slam/tracking/odometry         # must be ~30 Hz
ros2 topic echo /skybeta/state --once
ros2 topic echo /fmu/in/trajectory_setpoint          # what PX4 actually got
ros2 param get /flight_interface max_altitude_m
ros2 service call /skybeta/hold std_srvs/srv/Trigger # panic button
```

The `/fmu/out/vehicle_status` topic is spelled `/fmu/out/vehicle_status_v1` on PX4 v1.16 —
message versioning added a `_v<N>` suffix. `flight_interface` resolves this from the
installed `px4_msgs` at startup and logs the map it chose; do not hard-code either spelling.

---

## 7. The advanced track — `competitor_advanced_cpp`

`skybeta_flight_interface` drives PX4's built-in Offboard mode. The alternative is to
register **your own flight mode** with PX4 using `px4_ros2_cpp`, which is what
`src/competitor_advanced_cpp` demonstrates: a `px4_ros2::ModeBase` subclass called "Skybeta
Goto" that flies to a parameterised NED target using `GotoSetpointType`.

Once registered it appears in QGroundControl's mode list, and PX4's commander runs its arming
checks against it. **Nothing selects it as shipped:** the arm-then-autonomy flow in §4 enters
Offboard, not your mode, and the transmitter's mode switch holds Manual, Altitude and Position.
Getting a mode onto the switch is a parameter change on the FC, which is [at your own risk](../README.md#change-px4-parameters-at-your-own-risk).

```bash
ros2 run competitor_advanced_cpp skybeta_goto_mode --ros-args \
    -p target_north_m:=5.0 -p target_east_m:=0.0 -p target_down_m:=-1.5
```

**Take the extra risk seriously:**

- **No safety envelope.** The Python flight interface is not in this loop at all. There is no
  altitude clamp and no speed cap — whatever `updateSetpoint()` publishes goes
  to the controller. You own the envelope yourself.
- **NED, not ENU. `+z` is DOWN.** `target_down_m: -1.5` means 1.5 m *up*. A sign error here
  puts the vehicle in the ground.
- **It reads EKF2's estimate, not cuVSLAM's odometry.** GPS-denied that is only as good as
  the vision link feeding it, and it degrades silently when the link drops.
- **Registration is a live handshake.** The node throws at construction if the agent is not
  running or `px4_msgs` does not match the firmware. It is C++, so a mistake is a segfault
  rather than a traceback.

`relative_to_activation` (default true) anchors the target to wherever the pilot handed the
vehicle over, rather than to the EKF2 origin, which may be anywhere. Leave it true.

Fly the Python track first. Only move to this one if you actually need a named mode on the
switch.

---

## 8. A tour of the code

What happens between your node and the motors, in order. Paths are relative to the repo root.

**Startup.** `skybeta-stack.service` runs `host/stack_service.sh`, which runs `host/run.sh stack`.
That starts the container, and `container_scripts/run_full_stack.sh` opens the tmux windows:

| Window | Script | What it runs |
|---|---|---|
| `agent` | `run_agent.sh` | MicroXRCEAgent: `/fmu/*` topics over the UART to PX4 |
| `slam` | `run_slam.sh` | `oakd_vslam.launch.py`: camera driver, GPU mono conversion, cuVSLAM |
| `bridge` | `run_bridge.sh` | `px4_odom_bridge`: cuVSLAM odometry to `/fmu/in/vehicle_visual_odometry` |
| `competitor` | `run_competitor.sh` | builds `~/team_ws`, then launches `COMPETITOR_PKG` |

**The way in: where the vehicle thinks it is.** OAK-D stereo images go to cuVSLAM, which publishes
`/visual_slam/tracking/odometry` at 30 Hz in ENU. `px4_odom_bridge` rotates that to NED and sends it
to the flight controller, where EKF2 fuses it with the IMU and rangefinder. You read the same
odometry topic; EKF2's estimate comes back in `/skybeta/state` as `local_enu`.

**The way out: where you want it to go.** In `skybeta_flight_interface/flight_interface.py`:

1. `pose_setpoint_cb` / `twist_setpoint_cb` take your message.
2. `envelope.py` clamps it (§2), and `frames.py` converts ENU to PX4's NED.
3. The result becomes the current target. Nothing is sent yet.
4. `tick()` runs at 10 Hz: `build_setpoint()` turns the target into a `TrajectorySetpoint`, published
   after an `OffboardControlMode` heartbeat. This never stops, even with no target: a stream
   that stalls drops PX4 out of Offboard.
5. `_step_auto_offboard()`, also each tick, feeds `auto_offboard.py`, which decides whether this
   switch to Position may enter Offboard. All the §4 rules are in that one small file.
6. `publish_state()` publishes `/skybeta/state` at 5 Hz.

**Your node** is `competitor_example/square_mission.py` until you replace it. It is a state machine
(`WAIT_FOR_OFFBOARD`, `HOVER`, `SQUARE`, `HOLD`) driven by a timer, reading odometry and
`/skybeta/state` and publishing poses. Three things in it are worth copying: it publishes its current
position while waiting, it starts the mission only once `offboard_active` is true, and it stops
advancing when Offboard is lost.

**Where your code goes.** `~/team_ws/src` on the drone, built as an overlay on the image's workspace
by `team_build.sh`. The VIO pipeline never sources that overlay, so a broken team build cannot
stop the drone knowing where it is.

### Package map

```
src/
├── skybeta_flight_interface/          do not modify
│   ├── flight_interface.py            the 10 Hz heartbeat + the node
│   ├── auto_offboard.py               when arming may enter Offboard (§4)
│   ├── frames.py                      ENU<->NED / FLU<->FRD math
│   ├── envelope.py                    the altitude / speed clamp
│   ├── topics.py                      /fmu topic names vs px4_msgs MESSAGE_VERSION
│   └── test/                          pytest; integration_offboard.py needs a DDS peer
├── competitor_example/                COPY THIS
│   ├── square_mission.py              hover -> 5 m square -> hold
│   └── launch/competitor.launch.py    flight_interface + your node
├── competitor_advanced_cpp/           the advanced track (§7)
└── oakd_vslam_bringup/                camera + cuVSLAM + the PX4 odometry bridge
```

Run the unit tests any time you touch the interface:

```bash
cd /root/skybeta_ws/src/skybeta_flight_interface && python3 -m pytest -q test/
```
