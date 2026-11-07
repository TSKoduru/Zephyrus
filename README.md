# skybeta

**CURRENT DEFAULT: UP 35 cm → FORWARD 10 cm → DOWN, odometry only, stereo map recorded.**
Run `ros2 launch competitor_example competitor.launch.py`.
See [initial test setup and logs](docs/odom_10cm_test.md) before running.


A GPS-denied autonomy stack for a Jetson Orin Nano paired with a Pixhawk 6C, shipped as
one container image. **Write your navigation code in `~/team_ws/src` on your drone**:
everything below your setpoints is already built, measured and documented.

- **Competitors:** read the [rules](#rules), [connect to your drone](#connect-to-your-drone),
  then follow the [quick start](#quick-start--competitor) and
  [`docs/competitor_guide.md`](docs/competitor_guide.md).
- **Fleet operators** bringing up ten-plus identical drones:
  [quick start](#quick-start--fleet-operator), then
  [`docs/fleet_guide.md`](docs/fleet_guide.md).

**Need help?** Message Kelvin on Slack (`kelvinkn@stanford.edu`).

---

## Course localization additions

See [course setup, self-test and RViz](docs/course_setup.md) for the optional team-workspace particle filter, detailed logging and explicitly enabled feedback follower. The separate `course.launch.py` launch is localization/logging only; the follower is not MPC.

## Rules

You have root on your drone. That is so you can work freely, not permission to do everything
below.

### Do not touch another team's drone

**Do not SSH into any drone other than your own, or you will be disqualified.** This holds
whether or not the login works, and whether or not the drone is in use. Run `hostname` when you
log in and check that it shows your drone number. If you reach the wrong drone by mistake, log
out and tell the organisers.

### Do not override the doctor or the monitoring

Do not edit, replace, disable, stop, wrap or fake the output of any of these:

| What | Where |
|---|---|
| Doctor (health check) | `host/doctor.sh` |
| Status agent | `skybeta-status.service`, `/opt/skybeta-status/` |
| ROS status sidecar | `skybeta-ros-status.service`, `/run/skybeta-status/` |
| The organisers' link to the drone | Tailscale (`tailscaled`) |

Running `host/doctor.sh` is always fine, and so is reading these files. If one of them gets in
your way, ask the organisers.

### Do not change the sudo password

Do not change the password of the `dronehacks` account: it is the login and the sudo password,
and the organisers need it to maintain your drone.

### Fork the repo if you want to

You are welcome to fork this repository and work from your fork. Keep your code in your fork or
in `~/team_ws`, and leave the `~/skybeta` checkout on the drone as it is: the organisers update
it, and an update is refused if its tracked files have been changed.

```bash
# on the drone: your fork or your own repo goes in the team workspace
cd ~/team_ws/src
git clone https://github.com/<you>/<your-repo>.git
```

A GitHub fork of a public repository is public, so other teams can read it. If you want your
code private, create a new private repository and push your packages there.

### Change PX4 parameters at your own risk

You may change the PX4 parameters on your drone, but **it is at your own risk, and anything that
happens as a result is your responsibility.** The drone ships with parameters set for this
airframe. Changing `UXRCE_DDS_*`, `MAV_1_CONFIG`, `SER_TEL2_BAUD` or `EKF2_*` can cut the link to
the Jetson or stop the drone using vision for position. Write down what you change and its old
value.

### Keep the drone flying safely

- **A safety pilot on RC is always there**, and can take the vehicle back at any time. Nothing
  you write may assume otherwise.
- **Run new code with the props off first**, then low and tethered. A hover on visual odometry
  has not flown on any airframe yet, so your first runs are the test.
- **Do not publish to `/fmu/in/*`** and do not modify `skybeta_flight_interface`. Send setpoints
  through `/skybeta/setpoint/*`.
- **Do not change `DRONE_ID`, `ROS_DOMAIN_ID` or `TEAM`** in `/etc/skybeta/drone.env`, or the
  drone's hostname. They keep your drone's traffic separate from every other drone on the shared
  network. `COMPETITOR_PKG` and `COMPETITOR_LAUNCH` in that file are yours to set.

---

## How your drone is tested

1. We put your drone in the cage.
2. The safety pilot **arms** the drone, on the ground, with the mode switch on Altitude.
3. The safety pilot switches to **Position**, within a few seconds.
4. **We expect your drone to take over from there.** Your code takes off and flies with nothing
   more from us.

So all of this has to be true before the pilot arms:

- **Your code starts by itself.** Set `COMPETITOR_PKG` and `COMPETITOR_LAUNCH`, so that your node
  runs in the stack's `competitor` window with nobody logged in.
- **Your launch file starts `flight_interface` with `auto_offboard_on_arm:=true`.** Copying the
  example's launch file does this.
- **Your node is already publishing setpoints**, at least once a second, before arming.
- **Vision is tracking**, so PX4 has a valid position. Leave the drone still for the first minute.

**Takeover happens when the pilot switches to Position after arming, with the drone on the
ground.** It happens once per arming. Arming with the switch already on Position does not start your
code, and neither does switching to Position in the air. If anything above is missing, the drone
stays in Position under the pilot and `flight_interface` logs
`auto offboard: staying out of Offboard: <reason>`. Your code also has to take off within 10
seconds of arming, or PX4 disarms the drone.

Rehearse this whole sequence with the props off before your test.

---

## Connect to your drone

You will get your **drone number**, its **login** and the **Wi-Fi details** from the organisers.
The login user is `dronehacks`, and the drone's hostname is `skybeta-NN`, where `NN` is your
drone number (drone 7 is `skybeta-07`). Your laptop and the drone must be on the same Wi-Fi network.

```bash
ssh dronehacks@<your-drone's-wifi-ip>
```

**If you don't know the Wi-Fi address, or Wi-Fi is down,** connect a USB cable from your laptop
to the Jetson's USB-C port. Every Jetson answers on the same address over USB:

```bash
ssh dronehacks@192.168.55.1
hostname                        # check this is your drone
ip -4 -br addr show wlP1p1s0    # the drone's Wi-Fi address
```

Tailscale on the drones is for the organisers; teams do not use it.

**What is on the drone:** this repository,
[`Artash-N/skybeta`](https://github.com/Artash-N/skybeta), branch `fleet-ready`, cloned to
`~/skybeta`. Your own code goes in `~/team_ws`, which updates never touch.

---

## How the drone works

The drone has two computers. The **Pixhawk 6C** flight controller runs PX4: it keeps the vehicle
stable and drives the motors. The **Jetson Orin Nano** works out where the vehicle is and runs
your code, which decides where it goes.

An OAK-D-S2 stereo camera feeds NVIDIA's **cuVSLAM** on the Jetson's GPU. The resulting
visual odometry is converted to PX4's frame conventions and pushed to the flight
controller over **uXRCE-DDS** on `/dev/ttyTHS1`, where EKF2 fuses it as external vision.
The vehicle then knows where it is with no GPS and no optical flow.

Height comes from a downward laser rangefinder, with the barometer as backup. **Horizontal position
comes from vision and nothing else.** If cuVSLAM loses tracking in the air (darkness, a blank
floor or wall, a covered lens), PX4 has no horizontal position and falls back to Altitude
behaviour.

### A flight, start to finish

1. **Power up and keep the drone still for the first minute.** The stack starts by itself, and
   position and heading are measured from wherever the drone was when it started.
2. **Your node starts publishing setpoints** in the `competitor` window of the stack.
3. **The safety pilot arms, then switches to Position.** The Jetson has no way to arm the vehicle.
4. **The Jetson enters Offboard once, and your code takes off and flies.** The flight interface
   clamps altitude and twist speed and sends your setpoints to PX4. It does not limit how far
   you go horizontally: staying inside the flying area is your code's job and the pilot's.
5. **The pilot can take over at any moment** with the sticks, the mode switch or the kill switch.
   Offboard does not come back until the vehicle is disarmed and armed again.

[`docs/competitor_guide.md`](docs/competitor_guide.md) §4 has the conditions and the take-over
procedure in full.

### The data path

```
OAK-D-S2  ──USB3──▶  depthai-ros v3 driver     /oak/{left,right}/image_raw   bgr8, 1024x640 @30 Hz
                              │
                              ▼
                     isaac_ros_image_proc      /oak/{left,right}/image_mono  mono8, GPU convert
                     ImageFormatConverterNode
                              │
                              ▼
                     cuVSLAM (isaac_ros_visual_slam 3.2.6)
                              │
                              ▼               /visual_slam/tracking/odometry  ENU, ~30 Hz
                     px4_odom_bridge          ENU->NED, FLU->FRD
                              │
                              ▼               /fmu/in/vehicle_visual_odometry
                     MicroXRCEAgent v2.4.3
                              │
                        UART 921600
                              ▼
                     PX4 v1.16.2  EKF2
```

### Your code sits here

```
/visual_slam/tracking/odometry  ──▶  your node  ──▶  /skybeta/setpoint/{pose,twist}
                                                              │
                                                              ▼
                                                  skybeta_flight_interface
                                        altitude + speed clamp (no geofence),
                                        10 Hz OffboardControlMode heartbeat,
                                        ENU<->NED
                                                              │
                                                              ▼
                                        /fmu/in/trajectory_setpoint  ──▶  PX4
```

**There is always a safety pilot on RC.** The flight interface has **no arm path**. The pilot
arms and switches to Position; the interface then enters Offboard, and the pilot can take the vehicle
back at any instant with the sticks or the mode switch. Nothing you write may assume otherwise.

---

## Hardware

| | |
|---|---|
| Companion computer | **Jetson Orin Nano Super**, JetPack 6.x / L4T R36.5.0, Ubuntu 22.04 |
| Camera | **OAK-D-S2** — `OV9282` monochrome stereo pair, `IMX378` colour, BNO086 IMU |
| Flight controller | **Pixhawk 6C**, PX4 **v1.16.2** (vendored: [`px4/firmware/`](px4/firmware/)) |
| Link | TELEM2 ↔ Jetson 40-pin header pins 8/10, `/dev/ttyTHS1` @ 921600, uXRCE-DDS |
| Radio | Safety pilot's RC, with a tested kill switch |

Physical build and pre-flight checks, each tied to the failure it prevents:
[`docs/hardware_checklist.md`](docs/hardware_checklist.md).

Two hardware facts worth knowing before you measure anything:

- **The camera runs at 1024×640 @ 30 Hz, not the sensor's native resolution.** The
  MyriadX's XLink transport ceiling is ~143 MB/s, and at native resolution both eyes drop
  *different* frames — 23.3 Hz with left/right stamps a whole frame apart, and cuVSLAM
  odometry collapsing to 16.4 Hz. 1024×640 measures a clean 30.0 Hz at 118 MB/s and 2.56×
  the pixels of the old accidental 640×400 default.
- **The OAK-D enumerates twice.** Unbooted it is `03e7:2485` at 480 Mbps; once depthai
  pushes firmware it re-appears as `03e7:f63b` at 5000 Mbps. Any bandwidth reasoning based
  on the pre-boot `lsusb` reading is wrong.

---

## Quick start — competitor

Your drone comes preloaded: the skybeta image, this repo, and its identity file
`/etc/skybeta/drone.env` (drone number, DDS domain, camera mount pose), all set up by the
fleet operator ([fleet guide](docs/fleet_guide.md) step 3). Plug the OAK-D in **before**
you start the container.

```bash
cd skybeta

# 1. Check the machine is healthy.
host/doctor.sh

# 2. The stack starts by itself at power-up on a fleet drone. Look at it:
docker exec -it skybeta-stack tmux attach -t skybeta   # Ctrl-b d to leave

# Only on a drone without autostart (run.sh refuses a second stack). Positional arguments
# override drone.env's camera pose: x y z (m) roll pitch yaw (rad), FLU, Pixhawk IMU -> left lens.
host/run.sh stack
```

(Setting up a drone from scratch instead? `host/pull_image.sh $(cat VERSION)` with
`REGISTRY` set fetches the image; the fleet guide covers the rest.)

That opens a tmux session with four windows — `agent`, `slam`, `bridge`, `competitor`.
`Ctrl-b 0..3` switches, `Ctrl-b d` detaches. The banner shows your drone number, DDS domain
and the team workspace being used.

### Where to write your code

**In `~/team_ws/src` on the drone, nowhere else.** That directory is mounted into every
container at `/root/team_ws`, so it survives the container exiting and image updates; code
written anywhere inside a container is gone when it stops. Start from the example:

```
src/competitor_example/competitor_example/square_mission.py    <- copy this package
```

It is a heavily commented Python node that waits for Offboard, captures a start pose,
publishes ENU setpoints on a timer, checks arrival, and notices when the pilot takes over.
Copy it into `~/team_ws/src/my_team_nav`, rename the package, and point the launcher at it:
in `/etc/skybeta/drone.env` (`COMPETITOR_PKG=`, `COMPETITOR_LAUNCH=`), then restart the stack:

```bash
docker rm -f skybeta-stack    # the service starts it again in about 10 s; on the ground, keep it still
```

The `competitor` window builds the team workspace (`team_build.sh`) before launching. Extra
apt/pip packages go in `~/team_ws/Dockerfile` (`host/run.sh build --init`, then
`host/run.sh build`); never `apt install` in a running container. Don't put `px4_msgs` in
your workspace: the image already carries the generation that matches the flight controller.

Three topics are the entire interface:

| Topic | Type | Direction |
|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | you **subscribe** — ENU pose in `odom` |
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | you **publish** — ENU position + yaw |
| `/skybeta/state` | `std_msgs/String` (JSON) | you **subscribe** — what PX4 is doing |

**Do not publish to `/fmu/in/*` yourself.** Two publishers on `offboard_control_mode`
fight over the control flags and PX4 sees whichever arrived last.

**→ Read [`docs/competitor_guide.md`](docs/competitor_guide.md) next.** It covers the
frame conventions (including the one caveat nobody can fix for you), the safety envelope
and what a clamped setpoint looks like, `/skybeta/state`, the RC/arming procedure, and the
advanced C++ track that registers a custom PX4 flight mode.

No drone in hand? See [simulation](#simulation).

---

## Quick start — fleet operator

Bringing up an airframe from a bare board, in order. Each step has a verification command
whose output must match before you move on.

```
1  Flash PX4 v1.16.2                    px4/firmware/         (erases all parameters)
2  Load 00_baseline.params + 04..07 (RC only,
   arm ch 7 / kill ch 5, board roll 180,
   RadioMaster sticks, frame geometry),
   then calibrate THIS airframe in QGC                      px4/params/           (never clone calibration)
3  host/provision.sh                    Jetson                (one reboot, then resume)
   + /etc/skybeta/drone.env             unique DRONE_ID / ROS_DOMAIN_ID   doctor: drone-identity
4  Load 01_telem2_mavlink.params        prove the wire        mavproxy HEARTBEAT
5  Load 02_telem2_uxrce_dds.params      prove the data        check_topics.sh echoes
   + UXRCE_DDS_DOM_ID = drone number    (02 sets UXRCE_DDS_PTCFG=2)
6  Load 03a_height_reference.params     bench hover           altitude holds
7  Load 03b (EV_DELAY 70, measured)    EV fusion             cs_ev_pos / cs_ev_yaw true
8  Save px4/params/fleet/<serial>.params + tools/param_diff.py -> zero drift
```

**→ [`docs/fleet_guide.md`](docs/fleet_guide.md)** has each step in full, with the
expected output, the failure modes, and a fleet table template.

The two things most often got wrong:

- **Step 2.** Accelerometer, magnetometer, level-horizon and RC calibration are properties
  of *one* board, in *one* orientation, wired to *one* receiver. Clone them between drones
  and you get a vehicle that believes it is level when it is not — a silent failure with no
  error message anywhere. `00_baseline.params` excludes them by construction; its header
  lists exactly what was stripped and why.
- **Step 5, the DDS settings.** Every drone shares one WiFi network, so each gets its own
  DDS domain (`ROS_DOMAIN_ID` in `drone.env` = `UXRCE_DDS_DOM_ID` on its FC), and by default
  its DDS never leaves the Jetson. That needs **`UXRCE_DDS_PTCFG = 2`** on the FC (in the 02
  file): with the default 0 the agent's PX4 participant sits on every interface, the
  loopback-only stack never discovers it, and `/fmu/*` silently never arrives. Measured, not
  theorised: see [`host/README.md`](host/README.md#per-drone-identity-and-dds-isolation).
- **Step 5, the pass condition.** A topic list is not a pass. `ros2 topic list` showing fifty `/fmu/*` topics
  only proves the agent built a session and created the datawriters. **The pass condition
  is a `--once` echo returning a sample**, which is what `check_topics.sh` does.

Drift control, any time:

```bash
tools/param_diff.py --dump px4/params/fleet/SN0001.params   # exit 0 == no drift
tools/param_diff.py --master /dev/ttyTHS1,57600             # against a live FC
tools/param_diff.py --selftest                              # no hardware needed
```

---

## Repo layout

```
skybeta/
├── VERSION                          semver for the image tag
├── CLAUDE.md                        the record: measurements, bug list, what is unverified
├── jetson_pixhawk_uxrce_ros2_setup.md   the original hardware/PX4 procedure
├── container_scripts/               baked onto $PATH inside the image
│   ├── check_topics.sh              the real bug-#8 test: echo, not topic count
│   ├── record_bag.sh                mcap recording of the flight-debug topic set
│   ├── run_agent.sh                 MicroXRCEAgent, serial or udp4, -r participant refs
│   ├── run_bridge.sh                px4_odom_bridge
│   ├── run_competitor.sh            the competitor window: team_build, then launch
│   ├── run_full_stack.sh            tmux: agent, slam, bridge, competitor
│   ├── run_slam.sh                  camera + converters + cuVSLAM
│   ├── skybeta_env.sh               sourced: ROS + underlay + team overlay
│   └── team_build.sh                build ~/team_ws as an overlay, chown it back
├── docker/
│   ├── Dockerfile.jetson            aarch64: NGC base + apt + agent + workspace
│   ├── Dockerfile.sim               x86_64 twin for the Isaac Sim profile
│   ├── apt-pins.txt                 audit record of the exact versions in the image
│   └── ros-snapshots.key.asc        signing key for snapshots.ros.org
├── docs/
│   ├── competitor_guide.md          the API, the envelope, RC procedure, C++ track
│   ├── fleet_guide.md               per-drone bring-up, 8 verified steps
│   ├── distribution.md              the image: contents, registries, pulling, updating a fleet
│   ├── hardware_checklist.md        build-time + pre-flight physical checks
│   └── debugging.md                 symptom -> cause -> fix
├── host/                            things a container cannot do for itself
│   ├── README.md
│   ├── provision.sh                 one idempotent, resumable entry point
│   ├── doctor.sh                    twelve unprivileged health checks; non-zero on any FAIL
│   ├── run.sh                       stack | shell | bag | build | sim
│   ├── drone.env.example            template for /etc/skybeta/drone.env (per-drone identity)
│   ├── drone_env.sh                 library: parses drone.env (never sources it)
│   ├── dds/                         fastdds_localhost.xml (ROS nodes),
│   │                                agent_refs_{local,network}.xml (the agent, PTCFG=2)
│   ├── build_image.sh
│   ├── pull_image.sh                base-first, so each drone transfers the delta
│   ├── push_image.sh
│   ├── jetson_uart_dma_fix.sh       the R36.5 UARTA DMA/IOMMU device-tree patch
│   ├── uart_loopback_test.py        jumper pins 8+10 and prove the Jetson side
│   └── tests/                       test_extlinux_fix.sh, test_drone_env.sh
├── px4/
│   ├── firmware/                    px4_fmu-v6c_default-v1.16.2.px4 + .sha256 + README
│   └── params/
│       ├── README.md                the staged files and the order they go in
│       ├── 00_baseline.params       907 fleet-common parameters, no calibration
│       ├── 01_telem2_mavlink.params      TELEM2 -> MAVLink, proves the wire
│       ├── 02_telem2_uxrce_dds.params    TELEM2 -> uXRCE-DDS
│       ├── 03a_height_reference.params   range-finder height + baro backup (the runaway-climb fix)
│       ├── 03b_ekf2_vision.params        EV fusion
│       ├── 04_rc_only_fd_roll_off.params COM_RC_IN_MODE=0 (no joystick), FD_FAIL_R=0
│       ├── 05_rc_switches_board_rot.params RC_MAP_ARM_SW=7, RC_MAP_KILL_SW=5, SENS_BOARD_ROT=8 (roll 180)
│       ├── 06_rc_radiomaster.params      AETR stick order + RC1..18 calibration of the fleet RadioMaster
│       ├── 07_airframe_geometry.params   CA_ROTOR<n>_PX/PY rotor positions, EKF2_RNG_POS_Z
│       └── fleet/                   per-drone saved dumps: <serial>.params, plus the
│                                    SN0001 reference dump the baseline was generated from
├── sim/                             Isaac Sim 5.1.0 + Pegasus v5.1.0 + PX4 SITL, x86 only
│   ├── README.md                    ordered spike checklist
│   ├── pegasus_skybeta.py           the Isaac Sim app (host)
│   ├── px4_sitl.env                 SITL environment + param block (host)
│   └── skybeta_sim.launch.py        the stack, sim remaps (container)
├── src/
│   ├── competitor_example/          COPY THIS -- Python, hover -> 5 m square
│   ├── competitor_advanced_cpp/     advanced -- px4_ros2_cpp custom flight mode
│   ├── skybeta_flight_interface/    do not modify -- heartbeat, frames, safety envelope
│   └── oakd_vslam_bringup/          camera + cuVSLAM launch + the PX4 odometry bridge
└── tools/
    ├── latency_probe.py             measure the real EKF2_EV_DELAY
    └── param_diff.py                fleet drift check; --selftest needs no hardware
```

---

## Inside the image

Base: `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c` —
the same NGC Isaac ROS Humble image `isaac_ros_common`'s `run_dev.sh` pulls, pinned by
content hash so every Jetson in a fleet builds against identical bits. The bare
`aarch64-ros2_humble` tag does not exist on NGC, only the content-addressed variants.
**aarch64-only** — there is no amd64 manifest, by design, since cuVSLAM and NITROS ship
precompiled for Jetson.

| Layer | Step | Contents |
|---|---|---|
| 0 | apt source | `packages.ros.org/ros2` (rolling) replaced by `snapshots.ros.org/humble/2026-08-07`, so a rebuild months later resolves the same versions. `ros2-testing` is **not** installed |
| 1 | apt | `depthai-ros-driver-v3` **3.3.0**, `isaac-ros-visual-slam` **3.2.6**, NITROS imu/odometry types, `diagnostic-updater` (see below), eigen, build deps, tmux. One `RUN`, so the ~3 GB layer caches as a unit |
| 2 | pip | `depthai` runtime, `MAVProxy` for the stage-01 UART test |
| 3 | udev | the `03e7` OAK-D rule |
| 4 | git+cmake | **Micro-XRCE-DDS-Agent v2.4.3**, built from source |
| 5 | colcon | `/root/skybeta_ws`: `px4_msgs @ release/1.16`, `px4-ros2-interface-lib @ release/1.16`, and the four first-party packages from `src/` |
| 6 | COPY | `container_scripts/` onto `/usr/local/bin`, callable by name |
| 7 | bashrc | auto-source `/opt/ros/humble` + the workspace overlay |
| 8 | LABEL | provenance: ROS snapshot date, PX4 firmware, git refs, agent tag |

### Sizes, and why the pull script exists

| | |
|---|---|
| | Unpacked on disk | Transferred (compressed) |
|---|---|---|
| Full image | **42.6 GB** | 13.2 GB |
| Shared public NGC base | **40.3 GB** | 12.6 GB |
| **skybeta delta** | **~2.3 GB** | **~0.6 GB** |

The delta is the number that matters for fleet distribution. `host/pull_image.sh` fetches
the NGC base from `nvcr.io` **first**, so the registry pull then finds every base layer
already present and transfers only the delta. Pull skybeta first and you drag 13 GB
across the venue network, once per drone. Registry options, credentials, offline venues
and fleet updates: **[`docs/distribution.md`](docs/distribution.md)**.

### Pins, and the reasons for them

| Pin | Value | Why |
|---|---|---|
| PX4 firmware | **v1.16.2** | what the flight controller runs; vendored with its sha256 |
| `px4_msgs` | `release/1.16` | uXRCE-DDS matches on message definition. The installed `VehicleOdometry.msg` carries `MESSAGE_VERSION`, which is the quick proof of the generation |
| `px4-ros2-interface-lib` | `release/1.16` | the advanced C++ track (`px4_ros2_cpp`) |
| Micro-XRCE-DDS-Agent | **v2.4.3** | the tag PX4's own v1.16 ROS 2 user guide specifies. Do not bump it to v3.x |
| ROS apt snapshot | `2026-08-07` | `packages.ros.org/ros2` is a rolling repo; two builds of a "pinned" Dockerfile weeks apart genuinely resolve different candidates |
| Isaac ROS apt | NVIDIA `release-3` | already a pinned channel, in `/etc/apt/sources.list` in the base — the snapshot swap leaves it alone |
| pip | `depthai==3.10.0`, `MAVProxy==1.8.74`, `pymavlink==2.4.49`, `numpy<2` | exactly what 0.1.0 resolved. Unpinned, a 2026-10 rebuild pulled numpy 2.2.6 over the base's 1.26.4 (ROS Humble's compiled modules are built against 1.x), opencv-python 5.0 and vtk |

`ros-humble-diagnostic-updater` is named explicitly in the apt list on purpose. The NGC
base ships 4.0.6, which contains **only the Python module** — no `libdiagnostic_updater.so`
— so apt considers the depthai driver's dependency satisfied, never upgrades it, and the
driver's composable node dies at load with `cannot open shared object file`. Naming the
package pulls 4.0.7, which ships the library. Expect more of this shape: the base carries
pre-installed ROS packages apt will not upgrade just because something depends on them.

`px4_msgs` and `px4-ros2-interface-lib` are cloned by *branch*, so two builds can still
drift. Pass `--build-arg PX4_MSGS_REF=<sha>` to freeze them; the SHAs actually used are
written to `/root/skybeta_ws/px4_msgs.sha` and `px4_ros2_lib.sha` inside every image.

### Building and distributing

```bash
host/build_image.sh                 # context = repo root, -f docker/Dockerfile.jetson
host/build_image.sh --help          # env overrides: snapshot date, git refs, agent tag
```

Three tags, all the same image id: `skybeta:$(cat VERSION)`, `skybeta:<git-sha>` (suffixed
`-dirty` if the tree was not committed), and a readable compatibility tag
`skybeta:0.1.0-px4_1.16.2-isaacros_3.2`.

**`latest` is never pushed and never pulled.** `pull_image.sh` refuses it outright — a
moving tag is how a ten-drone fleet silently splits mid-competition. Record the digest,
not the tag.

```bash
echo "$TOKEN" | docker login <your-registry> -u <user> --password-stdin
REGISTRY=<your-registry>/<your-org> host/push_image.sh       # --dry-run first
```

Rebuild cost after an edit: `container_scripts/` → ~1 s (layer 6 only); `src/` → ~11 min
(colcon re-runs); an apt line or the agent tag → ~20 min; `SNAPSHOT_DATE` → the whole apt
layer, and regenerate `docker/apt-pins.txt` and diff it before trusting the result.

---

## Topics and frames contract

### Frames

| Frame | Convention | Notes |
|---|---|---|
| `odom` | **ENU** — +x East, +y North, +z Up | cuVSLAM's origin is the rig's **startup pose**, not north. "East" and "North" mean whichever way the vehicle's x and y pointed when the SLAM node came up |
| `base_link` | **FLU** — +x forward, +y left, +z up | origin at the **Pixhawk's primary IMU**, not the CoG or frame centre |
| `oak_parent_frame` | static child of `base_link` | the camera mount pose, from `host/run.sh stack <x> <y> <z> <r> <p> <y>` |
| PX4 local | **NED** — +x North, +y East, +z Down | EKF2's own frame; its origin and heading are *not* `odom`'s |

TF tree: `map → odom → base_link → oak_parent_frame → oak_{left,right}_camera_optical_frame`.

**cuVSLAM has no compass and no gravity vector** — `enable_imu_fusion` is off, so this is
pure stereo VO. A frame with neither cannot be NED, which is why the bridge declares
`POSE_FRAME_FRD` (value **2**) and lets EKF2 estimate the rotation itself. Keep
`EKF2_EV_POS_X/Y/Z` at zero on the flight controller: the static TF already carries the
camera lever arm, and setting both applies it twice.

### Topics

| Direction | Topic | Type / QoS | Source |
|---|---|---|---|
| pub | `/oak/{left,right}/image_raw` | `sensor_msgs/Image` `bgr8`, 1024×640 @30 Hz | depthai-ros v3 |
| pub | `/oak/{left,right}/camera_info` | `sensor_msgs/CameraInfo` | depthai-ros v3 |
| pub | `/oak/{left,right}/image_mono` | `sensor_msgs/Image` `mono8` | `ImageFormatConverterNode` |
| pub | `/oak/imu/data` | `sensor_msgs/Imu` @ ~199 Hz | depthai-ros v3 (BNO086) |
| pub | `/visual_slam/tracking/odometry` | `nav_msgs/Odometry`, RELIABLE @ ~30 Hz | cuVSLAM |
| pub | `/visual_slam/status` | `VisualSlamStatus` — `vo_state` 0 unknown / 1 tracking / 2 lost | cuVSLAM |
| pub | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry`, **BEST_EFFORT / VOLATILE** | `px4_odom_bridge` |
| sub | `/skybeta/setpoint/{pose,twist}` | ENU, from **your** node | `skybeta_flight_interface` |
| pub | `/skybeta/state` | `std_msgs/String`, JSON @ 5 Hz | `skybeta_flight_interface` |
| pub | `/fmu/in/{trajectory_setpoint,offboard_control_mode}` | @ 10 Hz | `skybeta_flight_interface` |
| sub | `/fmu/out/*` | BEST_EFFORT / TRANSIENT_LOCAL | uXRCE-DDS agent ← PX4 |

PX4 v1.16 **versions its messages**: a versioned topic gains a `_v<N>` suffix, so
`/fmu/out/vehicle_status` is spelled `/fmu/out/vehicle_status_v1`. Nothing in the stack
hard-codes either spelling — `check_topics.sh` and
`skybeta_flight_interface/topics.py` both resolve it against the live topic list.

### Camera mount pose

`CAMERA_X/Y/Z` and `CAMERA_ROLL/PITCH/YAW` in `/etc/skybeta/drone.env` become the `base_link →
oak_parent_frame` static TF. That is the only place the boot-time service reads them; `host/run.sh
stack <x> <y> <z> <roll> <pitch> <yaw>` overrides them for a hand-started stack. Translation in metres, rotation in radians, FLU body axes,
measured from the **Pixhawk's primary IMU** to the **left** stereo lens focal point
(cuVSLAM uses the left camera as the optical centre).

```bash
host/run.sh stack 0.10 0 -0.02 0 0.35 0    # 10 cm forward, 2 cm down, 20 deg down-pitch
```

Measure the rotation, do not assume zero. A down-pitched VIO camera is the norm on a
drone, and an unmodelled mount rotation couples vehicle rotation into apparent
translation — which orbits.

---

## Simulation

The whole stack — cuVSLAM, the odometry bridge, the flight interface, your node —
against a simulated OAK-D and a simulated Pixhawk, so a team can develop without a drone.

> **This needs an x86_64 workstation with an RTX GPU. It will not run on the Jetson.**
> NVIDIA's stated floor for Isaac Sim 5.1.0 is an **RTX 4080 16 GB, 32 GB RAM, Ubuntu
> 22.04, driver ≥ 595**. There is no aarch64 Isaac Sim build — only DGX Spark is supported
> on aarch64 — so this is a separate machine, not the drone's.

Pinned: **Isaac Sim 5.1.0 + Pegasus v5.1.0 + PX4 SITL v1.16.2**. Pegasus v5.1.0 requires
Isaac Sim 5.1.0 and is **not** compatible with 6.0.

**Status: a spike, not a supported path.** Every file in `sim/` has been checked against
the upstream sources it calls and byte-compiles, but nothing has been executed — the
development machine is a Jetson. [`sim/README.md`](sim/README.md) is an **ordered
checklist** with the expected output and the meaning of each failure, from PX4 SITL alone
through to flying the square against Pegasus's ground-truth pose. Run it in order and
record what actually happened. Gazebo Harmonic is the documented fallback.

---

## Debugging

Full version: **[`docs/debugging.md`](docs/debugging.md)**. The entries that come up most:

| Symptom | Cause | Fix |
|---|---|---|
| A rate measurement that looks bad | `ros2 topic hz` drops ~10 % in its own Python queue — it reported 27.1 Hz and a 150 ms max gap on a stream that was 30.016 Hz with a 33.37 ms max gap by header stamp | Measure with `record_bag.sh` / `host/run.sh bag` and analyse `header.stamp`. Never quote `ros2 topic hz` |
| Zero bytes on `/dev/ttyTHS1` after the stack was stopped or restarted (the link worked earlier) | PX4 v1.16.2 firmware: the FC's TELEM2 TX stays dead after the agent goes away ([`CLAUDE.md`](CLAUDE.md) bug #15) | **Power-cycle the FC.** A reseat does not help. Do this after every stack stop until the patched firmware is flashed |
| Zero bytes on `/dev/ttyTHS1` | The TELEM2 JST-GH connector, until proven otherwise | Reseat or replace the cable **before** touching any parameter. A negative UART result on a suspect cable is not a result |
| `/fmu/*` topics listed but no data | `UXRCE_DDS_PTCFG` ≠ 2 on the FC (local DDS scope), `ROS_DOMAIN_ID` ≠ `UXRCE_DDS_DOM_ID`, or `px4_msgs` generation vs firmware | `check_topics.sh` — the pass is an echo returning a sample, not a topic count. The topics you see may be only the stack's own endpoints |
| A laptop can't see the drone's topics | By design: `SKYBETA_DDS_SCOPE=local` keeps DDS on the Jetson | `SKYBETA_DDS_SCOPE=network` in `drone.env` + the same `ROS_DOMAIN_ID` on the laptop; switch back before flying |
| `libdiagnostic_updater.so: cannot open shared object file` | The base image's pre-installed 4.0.6 ships only the Python module; apt never upgrades it | Already fixed in the Dockerfile. When another node fails to `dlopen`, check `dpkg -L <pkg> \| grep '\.so'` before assuming the package is absent |
| `uxrce_dds_client status` shows 0 subscribers | Parameters or baud | `UXRCE_DDS_CFG=102`, `MAV_1_CONFIG=0`, `SER_TEL2_BAUD=921600`, **reboot the FC** |
| `X_LINK_UNBOOTED` | OAK-D enumerated after container start | Plug the camera in **before** `host/run.sh stack` |
| `Delta between current and previous frame ... above threshold` | Cosmetic. In the shipped cuVSLAM 3.2 binary this is a bare `RCLCPP_WARN` with no `return` | Nothing. Raising the threshold recovers nothing; the launch sets 40 ms only to silence the driver's stamp wobble |
| Toilet-bowling in position hold | `pose_frame`, EV yaw, `EKF2_EV_DELAY`, or an unmodelled camera tilt | [`debugging.md`](docs/debugging.md) §4.1 — check `pose_frame` is **2** first |
| Altitude climbs and keeps climbing | The PX4 height reference, not the vision code | `EKF2_HGT_REF` / `EKF2_BARO_CTRL` / `EKF2_RNG_CTRL`. Load `px4/params/03a_height_reference.params` |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` plus a ROS 2 setup script | Wrap the `source` in `set +u` / `set -u`, as every script in `container_scripts/` does |
| RViz from the container is blank | Host has not permitted X | `xhost +local:root` |

```bash
docker logs -f skybeta-stack                              # combined stdout (tmux noise)
host/run.sh shell                                         # new shell, or exec into the running one
docker exec -it skybeta-stack tmux attach -t skybeta      # reattach after Ctrl-b d
```

`host/run.sh stack` names its container `skybeta-stack` and its tmux session `skybeta`;
`host/run.sh shell` uses `skybeta`, and `host/run.sh bag` uses `skybeta-bag`. Override any
of them with `NAME=`.

---

## Known limitations

What changed between image versions, and what to do when upgrading a drone:
[`CHANGELOG.md`](CHANGELOG.md).

Read [`CLAUDE.md`](CLAUDE.md) before doing anything non-obvious. It is the record of what
has been measured, what has been fixed, and — importantly — what is *inference* rather
than fact. The short list:

- **The reference airframe's TELEM2 connector is physically damaged.** Every UART symptom
  in this repo's history traces to it, except silence after a stack restart (next item).
  Replace the cable before trusting any UART measurement.
- **On PX4 v1.16.2, every stack stop loses the FC until it is power-cycled.** When the agent
  stops answering (stack stop, `docker rm -f`, Jetson reboot, agent crash) the FC's TELEM2
  transmitter stays dead until the FC reboots. Power-cycle the FC after any stack stop until
  the patched firmware is flashed. [`CLAUDE.md`](CLAUDE.md) bug #15.
- **The flight symptoms may not have the causes the analysis suggests.** The saved May-07
  parameter dump has `EKF2_EV_CTRL = 0` — external vision fusion was **off** during the
  flights that toilet-bowled and climbed. Re-dump the parameters that were actually live
  before attributing a bad flight to the vision stack.
- **NITROS does not negotiate.** cuVSLAM 3.2 uses a *Managed*NitrosSubscriber, so type
  negotiation never completes. The `bgr8 → mono8` conversion genuinely runs on the GPU
  (p50 5.18 ms, 0 frames lost in 1743), but the hand-off to cuVSLAM is a plain in-container
  `sensor_msgs/Image`. It is not zero-copy; do not report it as such.
- **`enable_imu_fusion` is off**, so this is pure stereo VO with no inertial or gravity
  constraint. Vertical is its weakest axis. IMU rate is no longer the blocker it was —
  199.1 Hz after fixing the frequency defaults — so VIO is now worth trying.
- **The `stereo:` config block is inert.** The driver never instantiates a stereo/depth
  node, so nothing that block sets takes effect — including
  `i_reverse_stereo_socket_order`. If the stereo baseline sign ever needs flipping, the
  lever is the **TF**, not the YAML: cuVSLAM reads the baseline from TF, never from
  `CameraInfo` `P[3]`.
- **`enable_localization_n_mapping` defaults to False** in the launch — odometry only, no
  loop closure. Loop closure re-localises by teleporting the pose, which EKF2 has no way
  to interpret.
- **The image is aarch64-only.** The sim profile is the x86 counterpart.
- **The sim profile has never been executed.** See [above](#simulation).

---

## Reference

- [`CLAUDE.md`](CLAUDE.md) — measurements, the bug list, and what remains unverified
- [`jetson_pixhawk_uxrce_ros2_setup.md`](jetson_pixhawk_uxrce_ros2_setup.md) — the
  original hardware and PX4 procedure this stack was built from
- [`host/README.md`](host/README.md) — every host script, phase by phase
- [`px4/params/README.md`](px4/params/README.md) — the staged parameter files
- [`px4/firmware/README.md`](px4/firmware/README.md) — the pinned firmware and how to
  verify it
