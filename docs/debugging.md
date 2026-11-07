# Debugging skybeta

Symptom → cause → fix. Everything here was measured on the development unit; where a
number appears it is a measurement, not an estimate. The long-form reasoning behind each
entry lives in [`../CLAUDE.md`](../CLAUDE.md).

Two rules dominate everything below.

> **1. Measure with `ros2 bag record -s mcap` and header stamps, never `ros2 topic hz`.**
>
> **2. A negative UART result on a suspect cable is not a result.** Reseat the connector
> before you change a single parameter.

---

## 0. `ros2 topic hz` lies on this stack

It is the first tool everybody reaches for and it is wrong here by a wide margin. It
measures **arrival** time in a Python CLI subscriber, and that subscriber drops roughly
10 % of image messages in its own queue before it ever timestamps them.

Same stream, same 60 s, three ways of counting:

| Method | Rate | Max gap |
|---|---|---|
| `ros2 topic hz /oak/left/image_raw` (1026 samples, under load) | **27.1 Hz** | **150 ms** |
| C++ `ros2 bag record`, receive time | 30.18 Hz | — |
| Same bag, **header stamps** | **30.016 Hz** | **33.37 ms** |

An earlier session recorded "27.9 Hz average, 98 ms max gap" from `ros2 topic hz` and
concluded the camera was dropping frames. It was not. Two clean 60 s driver-only bags
analysed from header stamps gave 30.02 Hz with **zero** intervals above 34 ms, on both
runs. Header→arrival latency is where the jitter lives: p50 ≈ 22 ms, max 73 ms. That is
host delivery jitter, and cuVSLAM never sees it — its own frame-gap check uses header
stamps (`latest_ts - last_track_ts` in `visual_slam_impl.cpp`).

**Do this instead:**

```bash
host/run.sh bag                       # or, in the container:
record_bag.sh --duration 60
```

then analyse `header.stamp` deltas offline. `ros2 topic hz` is fine as a yes/no "is
anything publishing at all" probe. It is not fine as a number you put in a report.

---

## 1. UART / the uXRCE-DDS link

### Zero bytes on `/dev/ttyTHS1`

**If the link worked earlier in this FC power cycle, rule out the firmware first.** On PX4
v1.16.2 the FC's TELEM2 transmitter dies when the agent stops answering (stack stop,
`docker rm -f`, Jetson reboot, agent crash) and stays dead until the FC is power-cycled; see
[the next subsection](#the-stack-was-restarted-and-the-fc-never-came-back-0-bytes-uxrce_dds_client-status-says-disconnected).
No reseat fixes that. Otherwise:

**Check the connector before any parameter.** The TELEM2 JST-GH 1.25 mm 6-pin connector
on the reference airframe is physically damaged and does not seat fully. It produced
hours of silence, then one clean run in which the agent built the entire bridge (51
topics, 24 datawriters, 27 datareaders, real payloads on
`/fmu/in/vehicle_visual_odometry`), then silence again — 0 bytes across a continuous 60 s
watch.

That one good run is the whole diagnosis. It proves the wiring pinout, 921600 baud,
`UXRCE_DDS_CFG=102` and the agent were all already correct. Everything else was the
connector losing contact.

**Consequences.** Any *negative* UART measurement taken with a suspect cable in place is
uninformative — it tells you nothing about configuration. Positive results are still
valid. Replace the cable, then re-run everything that previously "failed".

Sniff the wire directly (no sudo needed if you are in `dialout`):

```bash
python3 -c "
import serial,time
s=serial.Serial('/dev/ttyTHS1',921600,timeout=0.2); s.reset_input_buffer()
b=bytearray(); t=time.time()
while time.time()-t<4: b+=s.read(4096)
print(len(b),'bytes'); print(bytes(b[:64]).hex(' '))"
```

Then, in order:

1. **Is the Jetson's own TX/RX path alive?** Jumper pins 8 and 10 and run
   `host/uart_loopback_test.py`. If loopback fails, the fault is on the Jetson side and
   nothing about the FC matters yet.
2. **Is the device tree patched?** `host/doctor.sh` checks
   `/proc/device-tree/bus@0/serial@3100000/` for `iommus` and `dmas`, and that
   `3100000.serial` is bound to `serial-tegra` owning `ttyTHS1`. No sudo, no `dmesg`.
3. **Is anything else holding the port?** Two MicroXRCEAgents on one serial port corrupt
   each other's framing. `host/run.sh` refuses to start when another running container
   already has `/dev/ttyTHS1` passed through, but a stray host-side `mavproxy.py` will
   not be caught.
4. **Only now, parameters.** `MAV_1_CONFIG` and `UXRCE_DDS_CFG` must never both point at
   `102`. One serial port, one owner; whichever you are not using goes to `0`.

### The stack was restarted and the FC never came back (0 bytes, `uxrce_dds_client status` says disconnected)

**Symptom.** The first run after the FC boots is fine. After the stack is stopped or restarted
(or the Jetson reboots, or the agent crashes), the next run gets nothing: 0 bytes on
`/dev/ttyTHS1`, 0 `/fmu/out` topics, `check_topics.sh` echoes nothing. In the fleet batch on
2026-10-20 (drones 5, 6, 16) this was a usbprov stack test that passed, then a second one
1-2 min later that failed with "vehicle_status never reached 0.5 Hz (got none)".

**Cause.** PX4 v1.16.2 firmware, [bug #15](../CLAUDE.md). After ~3 s without an answer from
the agent, the FC's `uxrce_dds_client` disconnects and closes and reopens `/dev/ttyS3`; a NuttX
STM32H7 UART TX DMA bug then leaves TELEM2's transmitter dead for every opener until the FC
reboots. The cable is fine.

**Confirm it in a minute.** On the Jetson, the sniff above: 0 bytes in 4 s. In a healthy
disconnected state the client sends a 0x7E ping frame every second, so 0 bytes is not "the
client is idle". On the FC's MAVLink console (QGC → Analyze Tools → MAVLink Console, FC on
USB):

```
uxrce_dds_client status     # "Running, disconnected"
dmesg                       # "No ping response, disconnecting" then
                            # "init serial /dev/ttyS3 @ 921600 baud"
```

**Fix.** Power-cycle the FC (or `reboot` on the MAVLink console). Within 20 s the sniff shows
the 1 Hz ping frames again, and the stack connects. `uxrce_dds_client stop` / `start` does not
help, nor does a reseat, nor any parameter. The lasting fix is firmware: PX4 v1.16.2 with the
NuttX fix cherry-picked. Until your FC runs it, power-cycle the FC after every stack stop.

### The agent connects, topics appear, nothing carries data

This is [bug #8](../CLAUDE.md) in its original form and the reason `check_topics.sh`
exists. A topic *name* in `ros2 topic list` only proves the agent built a session and
created the datawriters. It says nothing about whether PX4 is filling them.

```bash
run_agent.sh          # window 1
check_topics.sh       # window 2 -- the real test
```

`check_topics.sh` resolves each topic against the live list (PX4 v1.16 versions its
messages, so `/fmu/out/vehicle_status` is spelled `/fmu/out/vehicle_status_v1`) and then
does a timed `--once` echo. **A returned sample is the pass condition.** The count is
decoration.

If the echo hangs while the list looks healthy:

- `UXRCE_DDS_DOM_ID` on the FC must equal `ROS_DOMAIN_ID` in the container. That is this
  drone's own domain from `/etc/skybeta/drone.env`, not 0; `host/run.sh stack` prints it. The
  agent creates its participant in whatever domain the FC asks for, so a mismatch connects
  cleanly and puts every `/fmu` topic where nothing is listening.
- `UXRCE_DDS_PTCFG` on the FC must be `2`. With `0` the agent builds PX4's participant on every
  interface while the stack's ROS nodes sit on loopback; they never discover each other, and it
  looks exactly like a dead link. `run_agent.sh` prints the refs file it was given.
- Anything outside the container (a laptop, a second container started by hand) only sees the
  drone with `SKYBETA_DDS_SCOPE=network` **and** the same `ROS_DOMAIN_ID`. The default `local`
  scope keeps DDS on loopback + shared memory.
- `px4_msgs` must match the firmware generation. The image pins `release/1.16` and the
  installed `VehicleOdometry.msg` carries `MESSAGE_VERSION`; that field's presence is the
  quick proof you are not on a 1.14 build.
- Check the FC side: `uxrce_dds_client status` on the MAVLink console.

### `uxrce_dds_client status` shows 0 subscribers

Parameters, or baud. `UXRCE_DDS_CFG = 102`, `MAV_1_CONFIG = 0`, `SER_TEL2_BAUD = 921600`,
then **reboot the FC** — every one of those is reboot-gated. `px4/params/02_telem2_uxrce_dds.params`
sets all of them at once.

---

## 2. The image and the container

### `libdiagnostic_updater.so: cannot open shared object file`

Fixed in `docker/Dockerfile.jetson`, but the *shape* of this failure will recur, so know
it:

```
[oak_container] Failed to load library: Could not load library dlopen error:
libdiagnostic_updater.so: cannot open shared object file: No such file or directory
```

It is not a missing dependency declaration. `ros-humble-depthai-ros-driver-v3` correctly
`Depends:` on `ros-humble-diagnostic-updater`. The trap is that the **NGC base image
already has `ros-humble-diagnostic-updater 4.0.6` installed**, and that particular build
ships only the Python module — `dpkg -L` lists 68 files and **zero** `.so`. apt sees the
dependency as satisfied and never upgrades it. The composable node then `dlopen`s the C++
library at runtime and dies.

Fix: name the package explicitly in the Dockerfile's apt list, which pulls the candidate
(4.0.7) and provides `/opt/ros/humble/lib/libdiagnostic_updater.so`.

**Generalise it.** The base carries a set of pre-installed ROS packages that apt will not
upgrade just because something depends on them. When a node fails to `dlopen` a library,
compare `dpkg -l` against `apt-cache policy` *before* concluding the package is absent:

```bash
dpkg -l | grep <pkg>
apt-cache policy ros-humble-<pkg>
dpkg -L ros-humble-<pkg> | grep '\.so'      # the actual question
```

### `X_LINK_UNBOOTED` / `Skipping X_LINK_UNBOOTED device`

The OAK-D enumerated after the container started and the hotplug was not seen. Plug the
camera in **before** `host/run.sh stack`, then confirm with `lsusb | grep 03e7`.

### `AMENT_TRACE_SETUP_FILES: unbound variable`, pane dies immediately

`set -u` plus a ROS 2 setup script that reads unset variables. Every script in
`container_scripts/` already wraps the `source` in `set +u` / `set -u`. If you wrote a new
one, do the same.

### RViz from inside the container is blank

`xhost +local:root` on the host.

### The NGC base won't pull at build time

```bash
docker pull nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c
```

The bare `aarch64-ros2_humble` tag does not exist on NGC — only content-addressed
variants.

---

## 3. Camera and cuVSLAM

### Frame rate below 30 Hz, or L/R frames not pairing

First: are you measuring with `ros2 topic hz`? See §0.

If it is real, the likely cause is **transport throughput, not USB**. The MyriadX's XLink
transport tops out near 120–140 MB/s regardless of the USB link rate, and the OAK-D-S2
*is* on USB 3.0 SuperSpeed. Measured, driver only, 40 s per config, `camera_info` only:

| Config | Rate | Note |
|---|---|---|
| 1280×800 ×2 @30 | **23.3 Hz** | 143 MB/s — the ceiling; 23 % of frames lost |
| 1280×800 left only @30 | 30.0 Hz | proves it is throughput, not the sensor or the host |
| 1280×800 ×2 @20 | 20.0 Hz | clean, but costs a third of the rate |
| 1280×800 ×2 @30 + `i_low_bandwidth` | 23.3 Hz | no help |
| **1024×640 ×2 @30** | **30.0 Hz** | 118 MB/s — **what ships** |
| 960×600 ×2 @30 | 30.0 Hz | 104 MB/s, more margin |

At the native resolution the two eyes drop *different* frames: L/R header-stamp sync p95
rises to 33.3 ms — a whole frame — few stereo pairs match, and cuVSLAM odometry collapses
to 16.4 Hz with 1154 jitter warnings. **Do not "fix" the config back to 1280×800 without
re-measuring.** The rationale is in a comment block in
`src/oakd_vslam_bringup/config/oakd_vslam.yaml`.

`jetson_clocks` / `nvpmodel` are not a lever here: the unit is already on `MAXN_SUPER`
(`pmode 0002`), and `host/provision.sh`'s `power` phase keeps it there across reboots.

### `Delta between current and previous frame [...] is above threshold`

Cosmetic. In the shipped isaac_ros_visual_slam **3.2** binary this branch is a bare
`RCLCPP_WARN` with no `return` (`visual_slam_impl.cpp:671-678`); `Track()` runs on the
very next line. The same value goes to the library as
`configuration.max_frame_delta_ms`, which the installed `cuvslam.h` documents as being
compared "to warn user and prevent mistakes".

**Raising the threshold recovers nothing** — there is nothing being discarded. The launch
sets 40 ms only to stop the driver's 32.2–33.6 ms stamp wobble tripping the 34.0 default
about once a minute. Steady-state frame loss under full load was **0 in 1954**.

Beware: this is version-specific. In release-4.x the same parameter is
`max_frame_delta_s` and the surrounding code differs. The image installs 3.2.6; read the
`release-3.2` sources, not `latest`.

### `vo_state` is 2 (lost), or tracking drifts absurdly

```bash
ros2 topic echo /visual_slam/status --field vo_state     # 0 unknown, 1 tracking, 2 lost
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
```

- **cuVSLAM takes the stereo baseline from TF, not from `CameraInfo` `P[3]`.** The static
  transforms in the launch file *are* the rig geometry as far as the tracker is
  concerned. If the baseline is wrong, that is where it is wrong. In particular
  `i_reverse_stereo_socket_order` in the YAML is **not** the lever — the whole `stereo:`
  block is inert because the driver never instantiates a stereo/depth node.
- Low-texture scene. A blank wall or a bare floor gives a stereo front end nothing to
  hold.
- Check `/oak/left/image_mono` is actually `mono8`, at the resolution you expect:
  `ros2 topic echo /oak/left/image_mono --field encoding --once`.

### Images are `bgr8` and cuVSLAM wants `mono8`

Expected, and handled. The v3 driver publishes `/oak/left/image_raw` as `bgr8` even though
`CAM_B`/`CAM_C` are `OV9282` **monochrome** sensors, and the v3 parameter reference exposes
no encoding option for those sockets — so a host-side conversion is unavoidable.

Two `isaac_ros_image_proc` `ImageFormatConverterNode`s (`encoding_desired: mono8`) run in
the same composable container as cuVSLAM and publish `/oak/{left,right}/image_mono`.
Measured against the Python `bgr_to_mono` nodes they replaced: conversion latency p50
10.12 → **5.18 ms**, frames lost 2/1714 → **0/1743**.

**It is not zero-copy.** cuVSLAM 3.2 uses a *Managed*NitrosSubscriber, so type negotiation
never completes and `type_negotiation_duration_s` changes nothing. The conversion runs on
the GPU; the hand-off to cuVSLAM is a plain in-container `sensor_msgs/Image`. Do not
report it as NITROS zero-copy.

### `/oak/imu/data` at 26 Hz

The frequency defaults, not batching. Leaving `i_acc_freq` / `i_gyro_freq` at their
documented 480/400 yields **26.3 Hz** on the BNO086; asking for 200 explicitly gives
199.5. `i_sync_method: LINEAR_INTERPOLATE_ACCEL` beats `COPY` on regularity (p95 5.5 vs
8.8 ms). The shipped config asks for 200 and measures 199.1 Hz.

---

## 4. Flight symptoms

Read the caveat first.

> **The May-07 parameter dump has `EKF2_EV_CTRL = 0`.** External vision fusion was
> **off** on the airframe when those flights happened. Every suspect below assumes EV was
> being fused. If it was not, these flights had a different cause entirely — most likely
> the height-reference problem in §4.2, which is independent of vision. **Re-dump the
> parameters that were actually live during a bad flight before attributing it to
> anything in the vision stack.**

Also: an intermittent EV datalink is a plausible cause of both symptoms in its own right.
If the TELEM2 link drops mid-flight while `EKF2_EV_CTRL` is fusing vision, EKF2 loses
position aiding and the estimator degrades — which looks exactly like the drift being
chased. Fix the cable first.

### 4.1 Toilet-bowling in position hold

| Rank | Suspect | Check | Fix |
|---|---|---|---|
| 1 | `pose_frame` declared NED when the vision frame is not NED | `ros2 topic echo /fmu/in/vehicle_visual_odometry --field pose_frame` | Must be **2** (`POSE_FRAME_FRD`). The bridge now defaults to FRD; if you see 1, something overrode the `pose_frame` parameter |
| 2 | `EKF2_EV_CTRL` fusing position but not yaw | `EKF2_EV_CTRL` on the FC | `9` = bit 0 (horizontal position) + bit 3 (yaw). Fusing EV position while heading still comes from the magnetometer is the same frame mismatch from the PX4 side |
| 3 | `EKF2_EV_DELAY = 0` | the FC's value | **Measure it**: `tools/latency_probe.py`. Do not ship the 50 ms placeholder |
| 4 | Camera mount rotation unmodelled | `ros2 run tf2_ros tf2_echo base_link oak_parent_frame` | `host/run.sh stack <x> <y> <z> <roll> <pitch> <yaw>` — a down-pitched VIO camera is normal and must be declared, or vehicle rotation couples into apparent translation |

**Why #1 orbits.** cuVSLAM initialises its `odom` frame at the rig's startup pose — its x
axis points wherever `base_link` pointed when the node came up, not north. The bridge's
ENU→NED rotation is correct, so PX4 receives a valid *local* frame with an **arbitrary
heading reference**, which is the literal definition of `POSE_FRAME_FRD`:

```
uint8 POSE_FRAME_NED = 1 # NED earth-fixed frame
uint8 POSE_FRAME_FRD = 2 # FRD world-fixed frame, arbitrary heading reference
```

Tell EKF2 the measurements are north-referenced when they are offset by a constant, and
the position controller pushes at a fixed angle to the actual error. The vehicle orbits
instead of converging. Declare FRD and let EKF2 estimate the rotation, **or** fuse EV yaw
so the estimator adopts the vision heading. Do not do a partial version of both.

Confirm the frame is startup-aligned before acting:

```bash
ros2 run tf2_ros tf2_echo odom base_link                      # vehicle still
# yaw the airframe ~90 deg by hand, then compare:
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.orientation
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort \
                                          --qos-durability transient_local
```

A constant offset equal to the startup heading means startup-aligned, i.e. FRD.

### 4.2 Altitude drifts up and keeps going up

| Rank | Suspect | Check |
|---|---|---|
| 1 | **The PX4 height reference, not the vision code** | `EKF2_HGT_REF`, `EKF2_BARO_CTRL`, `EKF2_RNG_CTRL` |
| 2 | EV vertical fusion on top of pure stereo VO | `EKF2_EV_CTRL` bit 1 |
| 3 | No reset handling on tracking loss | `/visual_slam/status` vs `reset_counter` |

**#1 is the cheapest fix on the list and the one to try first.** The May dump has
`EKF2_HGT_REF = 2` (Range), `EKF2_BARO_CTRL = 0` (baro fusion **off**), and GPS off.
Whenever the range finder stops delivering (above its 4 m maximum, over an obstacle, a
rejected sample) there is then no height source left at all. The altitude estimate
free-runs, the controller reads it as sinking, and commands a climb that never
terminates — including the detail that it *starts* fine. (`EKF2_RNG_A_HMAX` only matters in
conditional mode; that airframe was on `EKF2_RNG_CTRL = 2`.)

Caveat, and it matters: outdoors with `EKF2_GPS_CTRL = 7` the GPS still provides height,
so this failure mode is specific to the GPS-denied case. `px4/params/03a_height_reference.params`
keeps the range finder as the reference and puts the barometer back behind it (PX4's
"Indoor (non-flat terrain)" configuration); it depends on nothing else in the stack.

**#2**: vertical is the weakest axis for pure stereo VO, and `enable_imu_fusion` is off,
so there is no inertial or gravity constraint at all. Keep `EKF2_EV_CTRL` bit 1 off until
vision altitude is trusted. The bridge's z arithmetic itself is correct (`-p.z` for
ENU→NED).

**#3**: when cuVSLAM loses tracking and re-initialises, its pose jumps. The bridge now
increments `reset_counter` when `vo_state` leaves and re-enters tracking, so EKF2 is told
to expect the discontinuity instead of integrating it as real motion. If you replaced the
bridge, you must do the same.

### Verified *not* a cause

cuVSLAM's `twist` **is** body-frame, so the bridge's `VELOCITY_FRAME_BODY_FRD` with
`(x, −y, −z)` is right. Confirmed in `pose_cache.cpp`: `GetVelocity` differentiates
`dp = poses.front().inverse() * poses.back()`, i.e. `T_body(t0)_body(t1)`, whose
translation is in the body frame.

Two caveats that are cosmetic today: that velocity is a finite difference across the whole
pose cache rather than the last two samples, so it is smoothed and lags; and
`angular_velocity` is an Euler **rate**, not the body angular-velocity vector — they
diverge at non-zero roll/pitch. EKF2 does not fuse `angular_velocity`.

### Covariance looks wrong

It probably is, and it probably does not matter yet. cuVSLAM publishes real covariance
(0 identity fallbacks in 1962 samples) but tiny — median σx ≈ 78 µm, five to seven orders
below `EKF2_EVP_NOISE = 0.1`. With `EKF2_EV_NOISE_MD = 0` PX4 treats the parameters as a
lower bound, so EKF2 clamps all six components and the message's own numbers never reach
the estimator. Set `EKF2_EV_NOISE_MD = 1` only if you want the parameters used directly.

---

## 5. Offboard control

See [`competitor_guide.md`](competitor_guide.md) §6 for the competitor-facing version.
The essentials:

| Symptom | First check |
|---|---|
| Nothing happens after the pilot arms and switches to Position | `flight_interface`'s `auto offboard:` log line (`armed already in Position` means the order was wrong: arm first), then `/skybeta/state` → `px4_link`. False means the agent is not talking to the FC |
| PX4 refuses to enter Offboard at all | The `OffboardControlMode` stream must already be running above ~2 Hz *before* the pilot arms, because the mode request goes out at that instant. `ros2 topic hz /fmu/in/offboard_control_mode` ≈ 10 Hz |
| Drops out of Offboard by itself | The stream stalled. `COM_OF_LOSS_T`, then `COM_OBL_RC_ACT` |
| Mission hangs at a waypoint | `clamped` in `/skybeta/state`. The safety envelope modifies setpoints silently apart from a throttled WARN |
| Setpoints ignored after your node pauses | Twist setpoints expire after `setpoint_timeout_s` (1 s) by design. Pose setpoints persist |

---

## 6. Where to look next

- [`../CLAUDE.md`](../CLAUDE.md) — the full record: measured numbers, the bug list, and
  what has and has not been verified.
- [`hardware_checklist.md`](hardware_checklist.md) — physical checks, each tied to the
  failure signature it prevents.
- [`fleet_guide.md`](fleet_guide.md) — per-drone bring-up with a verification command at
  every step.
- [`../px4/params/README.md`](../px4/params/README.md) — the staged parameter files and
  the order they go in.
