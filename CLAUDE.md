# skybeta — repo guide

Containerized Jetson stack pairing an **OAK-D-S2** (cuVSLAM visual odometry) with a
**Pixhawk 6C** (PX4 **v1.16.2**) over uXRCE-DDS on `/dev/ttyTHS1` @ 921600.

Read [`jetson_pixhawk_uxrce_ros2_setup.md`](jetson_pixhawk_uxrce_ros2_setup.md) for the
canonical hardware/PX4 procedure; [`README.md`](README.md) for the container workflow.

## Layout at a glance

- **`host/`** does what a container can't: `provision.sh` (idempotent, resumable: prereqs →
  UART DMA patch → power mode → image pull), `doctor.sh` (unprivileged health check),
  `run.sh stack|shell|bag|build|sim`, `build_image.sh` / `pull_image.sh` / `push_image.sh`,
  `jetson_uart_dma_fix.sh`, `uart_loopback_test.py`, `autostart.sh` (per-drone boot-time stack:
  `skybeta-stack.service` → `stack_service.sh`). The old numbered scripts are gone.
  Per-drone identity: `/etc/skybeta/drone.env` (template `drone.env.example`, parsed by
  `drone_env.sh`). DDS isolation: `dds/fastdds_localhost.xml` (ROS nodes) and
  `dds/agent_refs_{local,network}.xml` (the agent's PX4 participant, `UXRCE_DDS_PTCFG=2`).
- **`docker/Dockerfile.jetson`** builds `skybeta:<VERSION>` on the NGC Isaac ROS Humble base
  (content-hash pinned, aarch64). `docker/Dockerfile.sim` is the x86 sim twin.
- **`container_scripts/*.sh`** land on `$PATH` inside the image (`run_agent.sh`, `run_slam.sh`,
  `run_bridge.sh`, `run_full_stack.sh`, `run_competitor.sh`, `check_topics.sh`, `record_bag.sh`,
  `team_build.sh`, and `skybeta_env.sh`, which is sourced, not run).
- **Team code lives outside the image**, in `$TEAM_WS` (default `~/team_ws`) on the host,
  bind-mounted at `/root/team_ws` and built as an overlay on `/root/skybeta_ws`. The VIO -> FC
  pipeline (agent, cuVSLAM, bridge) never sources that overlay.
- **`src/`** — `oakd_vslam_bringup` (cuVSLAM launch + the PX4 odometry bridge),
  `skybeta_flight_interface` (the competitor-facing offboard wrapper: 10 Hz heartbeat, ENU↔NED,
  altitude + speed clamp, **no geofence** since 0.3.0, **no arm path**, enters Offboard once when
  the pilot arms, then switches to Position on the ground: `auto_offboard.py`), `competitor_example` (Python reference mission),
  `competitor_advanced_cpp` (px4_ros2_cpp custom mode). Competitors add their own packages
  in the team workspace, not here.
- **`px4/`** — pinned v1.16.2 firmware + staged parameter files; **`tools/`** — `param_diff.py`,
  `latency_probe.py`; **`sim/`** — Isaac Sim + Pegasus profile; **`docs/`** — competitor, fleet,
  hardware and debugging guides.

Data path: `OAK-D → depthai driver (bgr8) → isaac_ros_image_proc ImageFormatConverterNode
(GPU, mono8) → cuVSLAM → /visual_slam/tracking/odometry → px4_odom_bridge (ENU→NED, FLU→FRD)
→ /fmu/in/vehicle_visual_odometry → MicroXRCEAgent → UART → PX4 EKF2`. A competitor's node
consumes the odometry and publishes to `/skybeta/setpoint/*`.

## Verified facts about this Jetson

Measured 2026-09-02 on the dev unit, not assumed:

- **Jetson Orin Nano Engineering Reference Developer Kit Super**, L4T **R36.5.0**,
  Ubuntu 22.04.5, kernel 5.15.185-tegra.
- **UART DMA/IOMMU patch is live.** `/proc/device-tree/bus@0/serial@3100000/` carries
  `iommus`, `dmas`, `dma-names`; the platform device is bound into
  `/sys/kernel/iommu_groups/1` via `smmu.0x0000000012000000`, driver `serial-tegra`,
  compatible `nvidia,tegra194-hsuart`, status `okay`.
- **UART TX is healthy.** 4096 bytes written at 921600 8N1 completed in 55.8 ms against a
  44.4 ms theoretical floor — the DMA path is not stalling.
- `dmesg` is restricted (`kernel.dmesg_restrict=1`) and **sudo requires a password**, so
  `02b_uart_fix_verify.sh` / `jetson_uart_dma_fix.sh status` can't be run unattended. Use
  the `/proc/device-tree` + `/sys/bus/platform/devices/3100000.serial/iommu_group` checks
  above instead — they need no privileges.
- User is in `dialout` **and** `docker`, so the UART and docker need no sudo. The
  `sudo chmod 666 $DEV` / `sudo systemctl stop nvgetty` lines in `04_run_container.sh` and
  `run_pipeline.sh` are already no-ops here (`nvgetty.service` does not exist on this image).
- `/etc/docker/daemon.json` registers the `nvidia` runtime but does **not** set
  `default-runtime: nvidia` (contrary to what `01_prereqs.sh` would install). Harmless —
  every `docker run` in this repo passes `--runtime nvidia` explicitly, and the build needs
  no GPU.

## Hardware status

- **No skybeta airframe has an optical-flow sensor** (fleet owner, 2026-11-03; applies to every
  drone). The only ground-facing sensor is **one downward laser rangefinder with a ~1° beam**,
  run by PX4's VL53L1X driver (`SENS_EN_VL53L1X=1`; drone 16's FC reports `DISTANCE_SENSOR`
  laser, downward, 0–4 m). Earlier text here and in 03a called it a "27 deg FOV" VL53L1X; the
  beam is narrow, and the exact part is not recorded yet. The `EKF2_OF_CTRL=1`,
  `SENS_EN_PMW3901=1`, `SENS_TFLOW_CFG=103` in the May-07 dump (and so in `00_baseline`) are a
  leftover sensor set; `08_no_optical_flow.params` turns them off. **Horizontal position comes
  from VIO and nothing else** (no GPS, no flow): without the stack, Position mode cannot engage,
  and if VIO loses tracking in flight PX4 falls back to Altitude behaviour.

- **OAK-D-S2 is on USB 3.0 SuperSpeed — do not be fooled by `lsusb` before it boots.**
  The MyriadX enumerates twice. Unbooted it appears as `03e7:2485` at **480 Mbps** on bus 1;
  once depthai loads firmware it re-enumerates as `03e7:f63b` at **5000 Mbps** on `2-1.1`
  (bus 2, the USB3 hub). Verified: `dai.Device().getUsbSpeed()` → `UsbSpeed.SUPER`. Any
  bandwidth or latency reasoning based on the pre-boot `lsusb` reading is wrong.
- **Camera identity, read off the device** (`getConnectedCameraFeatures()`):
  `CAM_A = IMX378` 4056×3040 COLOR; `CAM_B`/`CAM_C` = **`OV9282` 1280×800**, supported types
  `[MONO, COLOR]`. IMU is **BNO086** fw 3.9.9 (the 9-axis part, not the BMI270 fallback).
- **Pixhawk link is intermittent — the TELEM2 JST-GH connector is physically damaged and
  does not seat fully.** Confirmed by the user 2026-09-02. This is the root cause of the
  intermittent first-contact silence in this repo's history (0 bytes until a reseat); for that
  symptom, do not go looking for a software explanation. Silence that starts **after the stack
  was stopped or restarted** is a different, firmware fault: bug #15 (found 2026-10-20).

  Observed sequence in one session: silent for hours → one clean run in which the agent
  established a session and built the **entire** bridge (51 topics, 24 datawriters,
  27 datareaders, `DataWriter::write` carrying real payloads, including
  `/fmu/in/vehicle_visual_odometry`) → silent again → 0 bytes across a continuous 60 s watch.
  That one good run is proof the wiring, 921600 baud, `UXRCE_DDS_CFG=102` and the agent are
  all correct. Everything else was the connector losing contact.

  **Same pattern again 2026-10-12** (cable still not replaced): 0 bytes at every baud rate ->
  after a reseat, uXRCE framing (`7e 01 00 ...`) at 921600 and a full session with the 0.2.0
  stack (24 datawriters, 27 datareaders, a replier) -> 0 bytes again after the stack was
  restarted, PX4 never reconnected. That session also showed the FC is on
  `UXRCE_DDS_DOM_ID=0`, `UXRCE_DDS_PTCFG=0` (agent participant multicasting on WiFi to
  `239.255.0.1:7400`).

  **Correction 2026-10-20:** the last step of that sequence (0 bytes after the stack was
  restarted, PX4 never reconnected) was not the connector. It is bug #15: on PX4 v1.16.2 the
  FC's TELEM2 transmitter dies when the agent stops answering and stays dead until the FC is
  power-cycled or rebooted. Reproduced on drone 16 with a good cable; a reseat does not clear
  it, an FC `reboot` does.

  **Consequences for debugging:** any *negative* UART result recorded while this connector is
  in place is uninformative — it tells you nothing about configuration. Positive results are
  still valid. Replace the JST-GH 1.25 mm 6-pin cable before trusting any further UART
  measurement, and re-run anything that previously "failed".

  **Consequences for flight:** an intermittent EV datalink is a plausible contributor to the
  reported toilet-bowl and altitude-drift symptoms in its own right. If the link drops mid-
  flight while `EKF2_EV_CTRL` is fusing vision, EKF2 loses position aiding and the estimator
  degrades — which looks exactly like the drift being chased. **Fix the connector before
  attributing those symptoms to `pose_frame`, `EKF2_EV_DELAY` or anything else in this
  document.** Do not bodge it with tape or glue on a flight vehicle.

Quick re-test (no sudo needed, user is in `dialout`):

```bash
python3 -c "
import serial,time
s=serial.Serial('/dev/ttyTHS1',921600,timeout=0.2); s.reset_input_buffer()
b=bytearray(); t=time.time()
while time.time()-t<4: b+=s.read(4096)
print(len(b),'bytes'); print(bytes(b[:64]).hex(' '))"
```

## Flight controller — what the saved logs actually show

**2026-10-15 dump** (`px4/params/fleet/SN0001-2026-10-15.params`): the FC runs **v1.16.1**
(QGC header "1.16.1 rc", git `94cb2012` = tag `v1.16.1`), not v1.16.2 as stated elsewhere in
this file. Same 1.16 message generation, so `px4_msgs @ release/1.16` still matches, but the
fleet pins v1.16.2. Reflashing erases every parameter, so dump first. It also confirms the
inference made from the agent's discovery traffic: `UXRCE_DDS_DOM_ID=0`, `UXRCE_DDS_PTCFG=0`.
See `px4/params/README.md` for the rest of what it shows.

`~/Downloads/mav.parm` (1129 params) and `~/Downloads/mav.tlog` (1.38 MB) are a MAVProxy
capture from **2026-05-07 22:25:49–22:26:44** (55 s, 19918 messages: `ATTITUDE`,
`HIGHRES_IMU`, `LOCAL_POSITION_NED`, `ODOMETRY`, `TIMESYNC`, plus a param download). That
capture is the proof the UART link *has* worked. Read it before re-debugging the wiring.

Two things it establishes that contradict the repo's premises:

**The FC runs PX4 v1.16+, not v1.14.** The dump contains `EKF2_RNG_FOG`, which is absent
from the v1.14 *and* v1.15 parameter references and first appears in v1.16 (checked against
`docs.px4.io/{v1.14,v1.15,v1.16}/en/advanced_config/parameter_reference.html`). See bug #8.

**The DDS path was never enabled on the airframe.** As of that capture:

| Param | Value | What the setup doc wants |
|---|---|---|
| `UXRCE_DDS_CFG` | **0** (disabled) | `TELEM2` |
| `MAV_1_CONFIG` | **102** (= TELEM2, MAVLink) | `0` |
| `MAV_1_MODE` | 2 (Onboard) | — |
| `SER_TEL2_BAUD` | 921600 | 921600 ✓ |
| `EKF2_EV_CTRL` | **0** (vision fusion off) | bits 0+3 = 9 |
| `EKF2_EV_DELAY` | **0** | 50 ms |
| `EKF2_HGT_REF` | **2** (Range) | Baro |
| `EKF2_BARO_CTRL` | **0** (baro off) | — |
| `EKF2_EVP/EVV/EVA_NOISE` | 0.1 / 0.1 / 0.1 | ✓ (already set) |

So TELEM2 was carrying **MAVLink**, not uXRCE-DDS — i.e. the working state on disk is §3 of
the setup doc (the MAVProxy sanity test), not §7 (the DDS bridge). The estimator was also
still on the *old* sensor set the container is meant to replace: `EKF2_OF_CTRL=1` (optical
flow on — but no current airframe has a flow sensor, see Hardware status), `EKF2_RNG_CTRL=2` with a measured `EKF2_RNG_POS_Y=-0.027432`, `EKF2_GPS_CTRL=7`.

Caveat: the dump may be truncated (1129 lines vs 1174 `PARAM_VALUE` messages in the tlog), so
treat a *missing* param as unknown; a *present* one is reliable. And these are May-7 values —
re-dump before trusting them.

## DepthAI grounding (checked against Luxonis docs, v3 driver)

**This container runs DepthAI ROS v3, not v2** — worth stating plainly because it is easy to
assume otherwise. Both lines are published to `packages.ros.org` and the base image can see
both:

```
ros-humble-depthai-ros-driver      Candidate: 2.12.2-1jammy   <- v2 line
ros-humble-depthai-ros-driver-v3   Candidate: 3.3.0-1jammy    <- v3 line, what we install
```

The Dockerfile installs the **`-v3`** packages, `oakd_vslam.launch.py` resolves
`get_package_share_directory('depthai_ros_driver_v3')`, and it includes `driver.launch.py`
(v2's entry point is `camera.launch.py`, deprecated in v3). The `driver:` / `pipeline_gen:`
namespaces in `config/oakd_vslam.yaml` are also v3 spelling — v2 nests the same settings under
`camera:`. The three are consistent, so the repo is coherently v3 throughout.

If a previous working setup was on v2, its YAML is **not** portable here: `camera:` →
`driver:`/`pipeline_gen:`, and `i_mx_id` → `i_device_id`.

Documented defaults that matter here:

| Parameter | Upstream default | Repo sets | Note |
|---|---|---|---|
| `left/right.i_width` × `i_height` | **640 × 400** | 1280 × 800 | **silently discarded — see #3** |
| `left/right.i_fps` | 30.0 | 30.0 | same as default |
| `left/right.i_enable_lazy_publisher` | `true` | `false` | correct: cuVSLAM subscribes late |
| `stereo.i_publish_left_rect` / `_right_rect` | **`false`** | `true` | unused by the launch — see #5 |
| `stereo.i_publish_synced_rect_pair` | *not in v3 docs* | `true` | likely a dead v2-era key |
| `stereo.i_depth_preset` | `FAST_ACCURACY` | `DEFAULT` | `DEFAULT` is a valid enum |
| `stereo.i_reverse_stereo_socket_order` | `false` | `true` | flips which camera is "first" in `CameraInfo` — verify the sign of `P[3]` on `/oak/right/camera_info` |
| `pipeline_gen.i_nn_type` | `spatial` | `none` | correct — avoids loading a blob |
| `pipeline_gen.i_pipeline_type` | — | `stereo` | valid set is `RGB, RGBD, Stereo, RGBStereo, Depth, CamArray, Tof, DepthTof, StereoToF, RGBToF, Thermal` |
| `driver.i_enable_ir` | `true` | `false` | no-op: OAK-D **S2 has no IR emitter or dot projector** |
| `imu.i_batch_report_threshold` | 5 | 1 | lower latency. **Not the IMU-rate lever** — `i_acc_freq`/`i_gyro_freq` are (see 3b): left at their documented 480/400 defaults the BNO086 delivers 26 Hz; set to 200 it delivers 199 |

**Hardware spec correction:** the OAK-D **S2 stereo pair is OV9282 _monochrome_**, 1280×800
native — *not* the OV9782 colour part the old launch-file docstring claimed. Confirmed on this
device: `CAM_B`/`CAM_C = OV9282`, IMU = **BNO086** fw 3.9.9. Note the driver publishes those
sockets as **bgr8 anyway**, so a host-side mono8 conversion is still required (bug #7); the
sensor part number was never the reason it was needed. Re-check on any new unit with:

```bash
docker exec -it skybeta python3 -c "
import depthai as dai
with dai.Device() as d:
    for f in d.getConnectedCameraFeatures(): print(f)"
ros2 topic echo /oak/left/image_raw --field encoding --once
```

Sources: [driver](https://docs.luxonis.com/software-v3/depthai/ros/driver/) ·
[parameters](https://docs.luxonis.com/software-v3/depthai/ros/parameters/) ·
[OAK-D S2 hardware](https://docs.luxonis.com/hardware/products/OAK-D%20S2) ·
[v3 release notes](https://discuss.luxonis.com/blog/6392-depthai-ros-release-30)

## cuVSLAM grounding (isaac_ros_visual_slam **3.2**, which is what ships here)

**Read the version-matched docs.** `nvidia-isaac-ros.github.io` defaults to `latest`
(release-4.6), and the parameter set changed between 3.x and 4.x. The image installs
**3.2.6**. Checking `latest` will tell you `enable_imu_fusion` does not exist and that IMU
fusion is controlled by `tracking_mode` (`Multi-camera (0), Visual Inertial (1), RGBD (2)`) —
true for 4.x, **wrong for us**. The release-3.2 source is authoritative:

```
https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/release-3.2/
    isaac_ros_visual_slam/src/visual_slam_node.cpp     # the declare_parameter<> block
    isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
    isaac_ros_visual_slam/src/impl/pose_cache.cpp
```

Verified defaults in **3.2** vs what `oakd_vslam.launch.py` sets:

| Parameter | 3.2 default | Launch sets | Note |
|---|---|---|---|
| `enable_imu_fusion` | `false` | `False` | **valid in 3.2** (renamed `tracking_mode` in 4.x); redundant but correct |
| `gyro_*` / `accel_*` noise | 0.000244 / 0.000019393 / 0.001862 / 0.003 | same values | verbatim copies of the defaults — no-ops, and only used when IMU fusion is on |
| `rectified_images` | `true` | `False` | correct — the launch feeds `image_raw` |
| `camera_optical_frames` | **`{}` (empty)** | *not set* | verify the base_link→optical-frame extrinsics actually resolve |
| `imu_frame` | `""` | `oak_imu_frame` | inert while IMU fusion is off |
| `image_jitter_threshold_ms` | **34.0** | *not set* | 30 fps = 33.3 ms nominal — 0.7 ms of headroom |
| `sync_matching_threshold_ms` | 5.0 | *not set* | left/right stamps must agree within 5 ms |
| `enable_localization_n_mapping` | **`true`** | *not set* | full SLAM + loop closure is running, not just odometry |
| `enable_slam_visualization` | `false` | now `False` | **the 13 `visual_slam/vis/*` topics are advertised regardless** — these flags gate *publishing*, not advertisement. 11 of 13 carry zero messages when off; `vis/slam_odometry` and `vis/velocity` are ungated in 3.2 |
| `enable_observations_view` | `false` | **`True`** | ditto |
| `enable_landmarks_view` | `false` | **`True`** | ditto |
| `path_max_size` | 1024 | 2048 | ditto |
| `image_qos` | `SENSOR_DATA` (BEST_EFFORT) | *not set* | the GPU converter publishes default RELIABLE — compatible; NITROS negotiation does not complete either way (see bug #7) |

**`image_jitter_threshold_ms` is warn-only — verified in the shipped binary.** In release-3.2
`visual_slam_impl.cpp:671-678` the over-threshold branch is a bare `RCLCPP_WARN` with no
`return`; `Track()` runs on the next line. The same value is passed to the library as
`configuration.max_frame_delta_ms` (3.2 uses ms; the `max_frame_delta_s = …/1000` form is
release-main/4.x), which the installed `cuvslam.h` documents as comparing deltas *"to warn user
and prevent mistakes"*. Its third use, `epsilon_stream2_` in `message_stream_sequencer.hpp`, is
a timeout to stop waiting for IMU; the only discard there is an out-of-order stamp. **Raising
the threshold recovers nothing.** Set it to ~40 ms only to silence a ~1/min cosmetic warning
caused by the driver's stamp wobble (32.2–33.6 ms) tipping over 34.0.

### What the docs do **not** say

NVIDIA documents `odom_frame` only as "the frame corresponding to the origin of the odometry
that cuVSLAM estimates", with the odometry pose being `odom_frame -> base_frame`. **Nothing
states its initial orientation.** So the claim underpinning the toilet-bowl diagnosis — that
`odom` is aligned to the rig's startup pose rather than to north or gravity — is *inference*,
not a documented fact. What supports it:

- `enable_imu_fusion` defaults to `false` and the launch keeps it off, so there is no inertial
  input and gravity is not observable.
- `visual_slam/vis/gravity` is documented as "the currently **estimated** gravity vector", and
  in `visual_slam_impl.cpp` `PublishGravity` early-returns unless `tracking_mode_ == VIO`.
- There is no north/mag input to cuVSLAM at all.

A frame with no gravity and no heading reference cannot be NED. But **measure it before acting**:

```bash
# 1. Start the stack, leave the vehicle still, then read the startup orientation:
ros2 run tf2_ros tf2_echo odom base_link
# 2. Yaw the airframe ~90 deg by hand and compare cuVSLAM yaw against the FC's:
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.orientation
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort --qos-durability transient_local
```

If cuVSLAM's yaw is offset from the FC's by a constant that equals the heading at startup, the
frame is startup-aligned and `POSE_FRAME_NED` is the wrong declaration.

## Target PX4 parameter sets

Enum values below are quoted from the **v1.16** parameter reference (the firmware this FC
actually runs — see bug #8). `*_CONFIG` params share one serial-port enum:
`0 = Disabled, 6 = UART 6, 101 = TELEM 1, **102 = TELEM 2**, 103 = TELEM 3,
104 = TELEM/SERIAL 4, 201/202/203 = GPS 1/2/3, 300 = Radio Controller, 301 = Wifi Port,
401 = EXT2, 1000 = Ethernet`.

**The one rule that matters: `MAV_1_CONFIG` and `UXRCE_DDS_CFG` must never both point at 102.**
One serial port, one owner. Whichever you are not using goes to `0`.

### State A — MAVLink, for proving the wire (§3 of the setup doc)

| Param | Value | Meaning |
|---|---|---|
| `MAV_1_CONFIG` | `102` | TELEM 2 |
| `MAV_1_MODE` | `2` | Onboard |
| `MAV_1_FLOW_CTRL` | `0` | Force off — CTS/RTS are not wired |
| `MAV_1_FORWARD` | `0` | — |
| `SER_TEL2_BAUD` | `57600` | drop to 57600 while debugging; 921600 also fine |
| `UXRCE_DDS_CFG` | `0` | Disabled — must not hold TELEM2 |

Then `mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors`.

**Note: the May 7 dump already satisfies this** (`MAV_1_CONFIG=102`, `MAV_1_MODE=2`,
`MAV_1_FLOW_CTRL=0`, `SER_TEL2_BAUD=921600`, `UXRCE_DDS_CFG=0`). So a powered, correctly wired
6C would already be streaming. Silence on the wire is **not** a parameter problem.

### State B — uXRCE-DDS, what the container needs (§4/§7)

| Param | Value | Meaning |
|---|---|---|
| `MAV_1_CONFIG` | `0` | Disabled — frees TELEM 2 |
| `UXRCE_DDS_CFG` | `102` | TELEM 2 |
| `SER_TEL2_BAUD` | `921600` | must match `run_agent.sh` / `XRCE_BAUD` |
| `UXRCE_DDS_DOM_ID` | this drone's ID | **must equal `ROS_DOMAIN_ID`** in `/etc/skybeta/drone.env`. Per airframe; set by hand, not in the 02 file |
| `UXRCE_DDS_KEY` | non-zero | unique per client; default is fine |
| `UXRCE_DDS_PTCFG` | `2` | custom participant: the agent builds it from `host/dds/agent_refs_<scope>.xml`. **With `0`, the loopback-only stack never sees `/fmu/*`** (bench-measured) |

Reboot the FC after either change — every param above is reboot-gated.

### Fleet policy — `04_rc_only_fd_roll_off.params` (2026-10-22)

| Param | Value | Meaning |
|---|---|---|
| `COM_RC_IN_MODE` | `0` | RC Transmitter only. The May dump (and so the baseline until 2026-10-22) had **1 = Joystick only**, which disables RC input and its checks: FCs provisioned before that date ignore the RC transmitter until re-flashed |
| `FD_FAIL_R` | `0` | roll failure detector off (0 = check disabled; default 60 deg / 0.3 s → takeoff lockdown). `FD_FAIL_P` stays 60 |

Kept out of `00_baseline.params` so `param_diff --make-baseline` still reproduces the baseline
from the May-07 dump; `~/pixhawk_params/flash.sh` loads 04 by default.

### Fleet policy — `05_rc_switches_board_rot.params` (2026-11-01)

| Param | Value | Meaning |
|---|---|---|
| `RC_MAP_ARM_SW` | `7` | arm/disarm switch on RC channel 7. The May dump (baseline until 2026-11-01) had arm on **5** and kill on **7**, i.e. swapped. Stick arming is off anyway (`MAN_ARM_GESTURE=0`) |
| `RC_MAP_KILL_SW` | `5` | emergency kill switch on RC channel 5 |
| `SENS_BOARD_ROT` | `8` | FMU board **Roll 180°** (v1.16 enum: 0 none, 4 yaw 180, 8 roll 180, 12 pitch 180): the 6C hangs upside down on these airframes, read off drone 20's QGC setup. Not 4 (yaw 180 = right way up, facing the tail). Reboot-gated. `SENS_BOARD_X/Y/Z_OFF` (level horizon) stays per-unit and must be calibrated **after** this is loaded |

Same staged-file reasoning as 04; `flash.sh` loads 05 by default. `param_diff` and
`~/pixhawk_autoparams.py` now exclude only `SENS_BOARD_[XYZ]_OFF` as per-unit, not the whole
`SENS_BOARD_` family, so `SENS_BOARD_ROT` reaches the FC. Every FC flashed before 2026-11-01 has
the swapped switches and no rotation until it is re-run through the flasher.

### Fleet policy — `06_rc_radiomaster.params` and `07_airframe_geometry.params` (2026-11-01)

Both read off **drone 20's FC** (`px4/params/fleet/drone20-radiomaster-2026-11-01.params`), the
one airframe configured by hand in QGC. 06: `RC_MAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4` (AETR; the
May dump's TAER 4/3/2/1 scrambles the sticks on the RadioMaster) and `RC1..RC18`
`_MIN/_TRIM/_MAX/_DZ/_REV` (sticks 982/1494/2005, switches 1000/1500/2000, deadzone 10). The
`RC<n>_*` family is **no longer per-unit** in `param_diff` or `~/pixhawk_autoparams.py`: one
transmitter model fleet-wide. 07: `CA_ROTOR0..3_PX/PY` (front x +0.08, rear x −0.15, y ±0.16 /
±0.14) and `EKF2_RNG_POS_Z = 0.03445`; the baseline had PX4's ±1 m placeholders. Baseline is
now 904 rows (907 before 08). Drone 20 diffs clean against baseline+staged except the deliberately unloaded 03b.

### Fleet policy — `08_no_optical_flow.params` (2026-11-03)

`EKF2_OF_CTRL=0`, `SENS_EN_PMW3901=0`, `SENS_TFLOW_CFG=0` (frees TELEM3). No airframe has a flow
sensor (Hardware status). In `param_diff` STAGED and `flash.sh`'s default set; once loaded the
`SENS_FLOW_*` params vanish from the FC, so `param_diff --dump` lists them as MISSING. Expected.

### Fleet policy — `09_autonomy_takeover.params` (2026-11-05)

`COM_RC_OVERRIDE = 3` (default 1): bit 1 lets the sticks take over from Offboard. Needed because of
the competition flow: the pilot arms, then switches to Position, then `flight_interface` requests Offboard
(`auto_offboard_on_arm`, rules in `auto_offboard.py`), so the mode switch is already on Position
during autonomy and PX4 only acts on it when it **changes** (`ManualControl.cpp`,
`mode_slot != _previous_switches.mode_slot`). Checked in the v1.16 source, not assumed: Offboard can
be entered armed on the ground (`UserModeIntention::change` has no landed check, only `canRun`), and
takeoff starts when a position setpoint is above the vehicle or vz is upward
(`MulticopterPositionControl.cpp`). `min_altitude_m` floors every pose, so any pose lifts it to 0.3 m.
Throttle counts as a stick (`sticks_moving`), and low throttle in Position descends, so raising
it is the clean take-over. **None of this has flown.** Default in `param_diff` STAGED and `flash.sh`,
as is 03b since the fleet rollout. Baseline is 903 rows.

### EKF2 for vision

| Param | Set to | Currently (May) | Why |
|---|---|---|---|
| `EKF2_EV_CTRL` | `9` | **0** | bitmask: `0`=horiz pos, `1`=vert pos, `2`=3D velocity, `3`=yaw. 9 = horiz pos + yaw. Add bit 1 (→11) only once vision altitude is trusted |
| `EKF2_EV_DELAY` | `70` | **0** | ms, range 0–300. **Measured** 2026-10-16 on the bench Orin, `tools/latency_probe.py`: image → bridge publish p50 68.7 ms, p95 102 ms, + ~1–2 ms serial. Re-check per airframe. Drone 16 (2026-11-03): p50 60.4 / p95 106 ms → **60** |
| `EKF2_HGT_REF` | `2` (Range) | 2 (Range) | values: `0`=Baro, `1`=GPS, `2`=Range, `3`=Vision. Range is the fleet choice (03a, PX4 "Indoor (non-flat terrain)") |
| `EKF2_BARO_CTRL` | `1` | **0** | default is Enabled. With baro off and GPS off, losing the range finder leaves **no height aiding** |
| `EKF2_RNG_CTRL` | `1` (conditional) | 2 | course may have boxes/tables; `2` is PX4's flat-floor variant |
| `EKF2_EV_NOISE_MD` | `0` | 0 | 0 = trust the variance in the message (params as lower bound); 1 = use params directly. **At 0 the bridge's swapped variances (bug #4) actually reach the estimator** |
| `EKF2_EVP/EVV_NOISE` | `0.1` | 0.1 ✓ | — |
| `EKF2_EVA_NOISE` | `0.05` | 0.1 | README asks for 0.05 |
| `EKF2_EV_POS_X/Y/Z` | `0` | 0 ✓ | keep zero — the static TF carries the lever arm (bug #9) |

There is no optical flow to retire: no airframe has a flow sensor, and `08_no_optical_flow.params`
(fleet default since 2026-11-03) sets `EKF2_OF_CTRL = 0`. The range finder stays as the height
reference (03a).

## Drone 16: VIO position hold on by default (2026-11-03)

Drone 16 (`skybeta-16`, Tailscale 100.94.45.41, Jetson serial 1423825093262, FC sysid 16 on the
patched `v1.16.2-1-g7a39c59`) is the first airframe that flies **Position** mode on VIO with
nobody logged in: power up, leave it still for about a minute, and Position (mode switch position 3,
`RC_MAP_FLTMODE=6`, `COM_FLTMODE1/4/6 = Manual/Altitude/Position`) is available.

- **FC:** fleet default set + `08` + **`03b`** with `EKF2_EV_DELAY=60`, measured on this airframe
  (`tools/latency_probe.py`, 30 s: full chain p50 60.4 ms, p95 106 ms, driver leg 36.5 ms). Loaded
  with `~/pixhawk_params/flash.sh … 08_no_optical_flow.params 03b_ekf2_vision.params --set
  EKF2_EV_DELAY=60 --drone-id 16` (1036 params, 0 missing/mismatched). Read-back:
  `px4/params/fleet/drone16-vio-2026-11-03.params`; `param_diff --dump` differs from
  baseline+staged only in `EKF2_EV_DELAY` (60 vs 70).
- **Jetson:** `sudo host/autostart.sh enable` → `skybeta-stack.service` → `host/stack_service.sh`
  (stack started detached, restarted if agent/slam/bridge dies). Attach with
  `docker exec -it skybeta-stack tmux attach -t skybeta`; `docker rm -f skybeta-stack` restarts it;
  `sudo systemctl stop skybeta-stack` stops it until the next boot; `sudo host/autostart.sh
  disable` removes it. Files were copied onto the drone's 63fec32 checkout (so it shows as dirty)
  until fleet-ready carries them.
- **Bench-verified (props off, stack up, FC rebooted onto the new params):**
  `estimator_status_flags`: `cs_ev_pos` and `cs_ev_yaw` true, `cs_rng_hgt` and `cs_baro_hgt` true,
  `cs_opt_flow`/`cs_mag_hdg` false. `vehicle_local_position`: `xy_valid`/`v_xy_valid` true, eph
  0.05 m. `failsafe_flags.local_position_invalid` false (what Position mode needs).
  `estimator_aid_src_ev_yaw` test ratio 0.0007, `ev_pos` 0.00003, both fused.
  `heading_good_for_control` reads false and `cs_yaw_align` false: with an FRD frame EKF2
  v1.16 resets its yaw to EV yaw without setting `yaw_align` (that path is for NED); it does not
  gate Position mode (`local_position_invalid` is false).
- **Bench-only arming failures (not VIO):** "USB connected" (`health_component system`; PX4 refuses
  to arm with the FC on USB), no RC link (Manual/Alt/Pos need the transmitter), "Strong magnetic
  interference" (`COM_ARM_MAG_STR=2`, warning only; mag not fused), global position (GPS modes only).
  One `pre_flt_fail_innov_heading` (heading ratio 1.46) right after the FC reboot cleared within a minute.
- **Operating notes:** the VIO origin/heading come from the startup pose, so keep it still at power-up.
  Tracking loss (dark, blank floor/wall, lens covered) leaves no horizontal aiding → Altitude
  behaviour. The usbprov stack test `docker rm -f`s `skybeta-stack`, so stop the service before running it.
- **Service enabled 2026-11-03 22:54:** stack up 14 s after `enable`; under it, `cs_ev_pos`/`cs_ev_yaw`
  true, `xy_valid` true (eph 0.04 m), `local_position_invalid` false.
- **Indoor caution:** the baseline carries `MPC_VEL_MANUAL=10` / `MPC_XY_VEL_MAX=12` m/s (full stick in
  Position mode = 10 m/s), far beyond a room and beyond what VIO tracks without motion blur.
  Offboard is not reachable from RC (`RC_MAP_OFFB_SW=0`, no Offboard in `COM_FLTMODE*`).
- **Jetson reboot, 2026-11-03 (FC kept powered on USB):** `skybeta-stack.service` started at boot
  with nobody logged in, "stack up" 15 s after the unit started (~20 s after kernel boot); about a
  minute after boot `vo_state=1`, `cs_ev_pos`/`cs_ev_yaw` true, `xy_valid` true (eph 0.04 m),
  `local_position_invalid` false. The FC, up throughout, reconnected to the new agent with no
  power cycle (patched firmware). An FC-only power cycle under the running stack also recovered
  by itself: fusing again ~15 s after the FC booted.
- **Clock step after boot broke vision (found and fixed 2026-11-03).** The Jetson has no RTC battery:
  it booted at the restored 22:58:21, the stack started, and NTP stepped the clock to 23:04:08. The
  OAK-D driver keeps stamping frames on its start-time clock, so every
  `/fmu/in/vehicle_visual_odometry` reached the FC with `timestamp_sample` 317 s stale (FC-side
  `listener vehicle_visual_odometry`). EKF2 rode it out until its time-sync filter caught up, then
  stopped fusing (`estimator_aid_src_ev_pos` last updated 80 s earlier, `cs_ev_pos` false,
  `xy_valid` false) mid hand test. A stack restart made the samples 57 ms old again. **Fix:**
  `stack_service.sh` waits up to 90 s for `NTPSynchronized=yes` before starting, and restarts the
  stack when wall clock moves against `/proc/uptime` by more than 0.5 s. Both verified on drone 16:
  a forced +3 s step logged "clock stepped by +2.5 s" and restarted, and with timesyncd paused it
  waited, synced after 44 s, then started. **This affects any hand-started stack too:** after a cold
  boot, start the stack only once `timedatectl` says synchronized, or restart it after the sync.
- **Hand test 1 (after the clock fix):** fusion continuous, no resets, `xy_valid` throughout,
  returned to (0.06, −0.06) m; final 90° turn read +83°. Inconclusive on frame alignment: the
  airframe was rotated while carried, max excursion 0.58 m, and the first motion registered left of
  the nose. A controlled repeat (nose fixed, measured 1 m forward / right, then 90° CW) is pending.
- **Not yet verified:** an actual hover, and a battery cold boot of Jetson and FC together (FC off USB).

## Flight symptoms → most likely causes

Two reported behaviours in position hold: **toilet-bowling**, and **altitude drifting up and
continuing up**. Both have specific suspects here. Ranked by how well they explain the
symptom and how cheap they are to check.

### Toilet bowl

**#1 suspect (now addressed): `pose_frame` was declared `POSE_FRAME_NED`, but the vision frame
is not NED.** The bridge used to set `out.pose_frame = VehicleOdometry.POSE_FRAME_NED`; it now
defaults to `POSE_FRAME_FRD` (verified on the bench: `pose_frame=2`). From
`px4_msgs/msg/VehicleOdometry.msg` (identical in `release/1.14` and `release/1.16`):

```
uint8 POSE_FRAME_NED     = 1 # NED earth-fixed frame
uint8 POSE_FRAME_FRD     = 2 # FRD world-fixed frame, arbitrary heading reference
```

cuVSLAM initialises its `odom` frame at the rig's **startup pose** — its x axis points wherever
`base_link` pointed when the node came up, not north. The ENU→NED rotation in the bridge is a
correct rotation, so what PX4 receives is a valid *local* frame with an **arbitrary heading
reference** — which is the literal definition of `POSE_FRAME_FRD`, not `POSE_FRAME_NED`.

Telling EKF2 the measurements are north-referenced when they are offset by (90° − startup
heading) leaves a constant yaw error between the position measurements and the mag/GPS-derived
attitude. The position controller then pushes at a fixed angle to the actual error, and the
vehicle orbits instead of converging. That is the textbook toilet-bowl mechanism.

Fix: use `POSE_FRAME_FRD` and let EKF2 estimate the frame rotation, **or** fuse EV yaw so the
estimator adopts the vision heading. Do not do a partial version of both.

**#2 suspect: `EKF2_EV_CTRL` without the yaw bit.** The README calls for bits 0+3
(horizontal position + yaw) = 9. The May param dump has `EKF2_EV_CTRL = 0`. Fusing EV
*position* while heading still comes from the magnetometer is the same frame-mismatch as
above, arrived at from the PX4 side.

**#3 suspect: `EKF2_EV_DELAY = 0`.** The dump has 0; the README asks for 50 ms. True
end-to-end latency here is plausibly 50–150 ms: USB 2.0 camera → depthai → a **Python**
GPU mono8 conversion → cuVSLAM → DDS → 921600 serial. Fusing stale position as if it were current
turns any oscillation into a growing orbit. Measure it, don't guess — `tools/latency_probe.py`
does exactly this and prints a recommended value.

**#4 suspect (now addressed): camera mount orientation was unmodelled.** The static TF was
`base_link → oak_parent_frame` with hardcoded `--roll 0 --pitch 0 --yaw 0`, and the old
`run_pipeline.sh <x> <y> <z>` exposed **translation only** — a down-pitched VIO camera, the norm
on a drone, could not be expressed at all, and an unmodelled mount rotation couples vehicle
rotation into apparent translation, which orbits. `oakd_vslam.launch.py` now takes
`camera_roll/pitch/yaw`, passed through `host/run.sh stack`. **Measure your actual mount and
pass it** — the default is still zero.

### Upward drift

**#1 suspect: the PX4 height configuration, not the vision code.** The May dump has
`EKF2_HGT_REF = 2` (Range), `EKF2_BARO_CTRL = 0` (baro fusion **off**), GPS off, and
`EKF2_RNG_CTRL = 2`. Once the range finder stops delivering — above its range (the PX4 VL53L1X
driver's maximum is **4 m**, less in bright light), over an obstacle, a rejected sample — there
is **no other height source left**, so the altitude estimate free-runs, the controller reads it
as sinking, and commands a climb that never terminates. That matches "drifts upward and then
continues upward" precisely, including the fact that it *starts* fine.

*Corrected 2026-10-15:* this entry (and `03a`) used to blame `EKF2_RNG_A_HMAX = 5.0`. The
firmware's own parameter docs say that limit "only applies when conditional range aid mode is
activated (EKF2_RNG_CTRL = 1)"; this airframe is on 2 (always). The conclusion stands, the
mechanism was wrong. The root cause is baro being **off**, not the range finder being the
reference: PX4 v1.16's documented "Indoor (flat terrain)" setup *is* range as the reference
(`EKF2_HGT_REF=2`, `EKF2_RNG_CTRL=2`) **with** `EKF2_BARO_CTRL=1`, so the estimator "continues
to estimate its height using the other sensors" when range drops out. That is what
`px4/params/fleet/SN0001-fixes-2026-10-15.params` applies.

Check `EKF2_HGT_REF`, `EKF2_BARO_CTRL` and `EKF2_RNG_CTRL` on the FC **before** touching the
vision code. This is the cheapest fix on the list.

**#2 suspect: EV vertical fusion on top of pure stereo VO.** If `EKF2_EV_CTRL` bit 1 got
enabled, vision z goes straight into altitude — and `enable_imu_fusion: False` in
`oakd_vslam.launch.py` means this is *pure* stereo VO with no inertial or gravity constraint.
Vertical is the weakest axis for pure VO, especially over a low-texture floor with rotor
downwash in frame. The bridge's z math itself is correct (`-p.z` for ENU→NED).

**#3 contributor (now addressed): no reset handling.** `out.reset_counter` was hard-coded to 0,
so when cuVSLAM lost tracking and re-initialised, its pose jumped and PX4 was never told to
expect a discontinuity — EKF2 integrated the jump as real motion. The bridge now subscribes to
`/visual_slam/status` and increments a wrapping `reset_counter` on every non-tracking →
tracking transition (`vo_state`: `0` unknown, `1` success, `2` failed).

### Verified *not* the cause

cuVSLAM's `twist` **is** body-frame, so the bridge's `VELOCITY_FRAME_BODY_FRD` +
`(x, −y, −z)` is right. Confirmed in `pose_cache.cpp`: `GetVelocity` differentiates
`dp = poses.front().inverse() * poses.back()`, i.e. `T_body(t0)_body(t1)`, whose translation is
in the body frame — not the odom frame. (This also confirms bug #4: `velocity_variance` must
**not** take the world-frame x/y swap.)

Caveat: that velocity is a finite difference across the whole pose cache, not the last two
samples, so it is smoothed and lags. And `angular_velocity` is Euler-**rate**, not the body
angular-velocity vector — they diverge at non-zero roll/pitch. EKF2 does not fuse
`angular_velocity`, so it is cosmetic today.

## Simulation grounding (Isaac Sim 5.1.0 + Pegasus v5.1.0, x86 only) — see `sim/`

Isaac Sim cannot run on a Jetson (NVIDIA floor: RTX 4080 16 GB, 32 GB RAM, Ubuntu 22.04,
driver >= 595; only DGX Spark is supported on aarch64). The sim profile is x86-only and lives in
`sim/` + `docker/Dockerfile.sim`. Pegasus **v5.1.0 is pinned to Isaac Sim 5.1.0 and is NOT
compatible with Isaac Sim 6.0**.

**x86 base image:** `nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907`
— the amd64 twin of the aarch64 tag `docker/Dockerfile.jetson` pins (same `isaac_ros_common`
layer history, build timestamps 2025-11-07). Established by enumerating all 291 NGC tags and
diffing image configs, not guessed.

Four findings from building the profile that also matter on hardware:

- **cuVSLAM takes the stereo baseline from TF, not from `CameraInfo` `P[3]`.** The static
  transforms in the launch file *are* the rig geometry. This is why bug #5's
  `i_reverse_stereo_socket_order` is not the lever for a baseline sign error — the TF is.
- **Isaac Sim 5.1 deprecated `set_rational_polynomial_properties`**, so Pegasus's `intrinsics`
  fx/fy and `diagonal_fov` config keys are silently ignored and fx/fy are derived from the prim
  lens instead. Unpatched, the sim camera is ~47 deg HFOV rather than 80. `sim/pegasus_skybeta.py`
  overrides this before the CameraInfo writer snapshots.
- **Zero distortion coefficients are load-bearing in sim:** any non-zero k4/k5/k6 makes
  `read_camera_info` report a `rational_polynomial` model, which contradicts
  `rectified_images: True` and breaks cuVSLAM's assumption.
- **Pegasus `px4_autolaunch` defaults off**, and when it does launch PX4 it uses
  `cwd=TemporaryDirectory()`, so `param save` never persists. Set SITL params every run.
  PX4 SITL is the TCP *client*: the pass condition for step 1 of the spike is PX4 blocking at
  "Waiting for simulator", not a prompt.

The RGB-capture risk flagged during planning is smaller than feared: images do not flow through
the commented-out `get_rgba()` — `ros2_backend.py:398-403` attaches a Replicator writer to the
camera's render product. The spike checklist still verifies it at step 4.

Pegasus was tested upstream against PX4 v1.14.3; we target **v1.16.2** SITL. That gap is the
main open risk, and `sim/README.md` is an ordered spike checklist with the expected output and
failure meaning for each step. Gazebo Harmonic is the documented fallback.

## End-to-end bench verification (2026-09-03, image `skybeta:0.1.0`)

Full stack on the rebuilt image — driver + GPU converter + cuVSLAM + bridge, camera static on
the bench, launched with `camera_x:=0.10 camera_z:=-0.05 camera_pitch:=-0.2618`. 44 s mcap,
header stamps. This is the state a competitor forks.

| topic | n | rate | p99 gap | max gap | note |
|---|---|---|---|---|---|
| `/oak/left/image_raw` | 1312 | 30.000 Hz | 33.33 ms | 66.67 ms | **1024×640 bgr8**, one dropped frame in 44 s |
| `/oak/left/image_mono` | 1312 | 30.000 Hz | 33.33 ms | 66.67 ms | **mono8**, GPU converter |
| `/oak/imu/data` | 8618 | **197.9 Hz** | — | — | p50 5.02 ms |
| `/visual_slam/tracking/odometry` | 1313 | 30.023 Hz | — | — | `vo_state = 1` on **1313/1313** |
| `/fmu/in/vehicle_visual_odometry` | 1301 | 29.952 Hz | — | — | `pose_frame=2` (FRD), `velocity_frame=3` (BODY_FRD), `reset_counter=[0]` |

- Static TF resolves: `base_link → oak_left_camera_optical_frame` = translation
  `[0.099, 0.038, -0.050]` with the −15° pitch composed into the optical rotation, so the new
  `camera_roll/pitch/yaw` launch args do reach the transform.
- Pose covariance identity fallback: **0 of 1313**.
- **The bridge's identity-covariance warning fired anyway, and that was a bug in the warning.**
  cuVSLAM's `PoseCache` returns the 1.0 identity for the first frame or two before it holds two
  poses, so warning on first sight cries wolf on every run. It now requires a full second (30
  consecutive messages) of identity before warning, and any real covariance resets the streak.
  Tested three ways: startup transient silent, sustained failure warns exactly once at the 30th
  message, 29-message bursts silent.

Not covered on the bench: `vo_state` under motion (camera was static), and everything
downstream of the UART — the TELEM2 connector is still broken.

## Fleet bench verification (2026-10-12, image `skybeta:0.2.0`, branch `fleet-ready`)

Context: every team gets one drone, all drones share one WiFi network, and teams write code on
the drone itself. Measured on the dev Orin (`wlP1p1s0`), not assumed.

**DDS isolation**

| Check | Result |
|---|---|
| RMW in the image | `rmw_fastrtps_cpp`, Fast DDS 2.6 |
| MicroXRCEAgent's Fast DDS | its own **2.14.7** (`/usr/local/lib/libfastrtps.so.2.14.7`), separate from ROS's |
| `fastdds_localhost.xml`, ros:humble container, 7 publishers + 2 echoes | all discovered; **0** UDP packets on eth0 (294 without the profile) |
| same, but default `maxInitialPeersRange` (4) | only 4 of 7 topics discovered — loopback has no multicast, so the range is load-bearing |
| full stack, local scope, domain 1 | 29 topics, cuVSLAM odometry 29.99–30.02 Hz, **0** DDS packets on WiFi in 30 s |
| full stack, network scope (control) | 71 packets in 30 s, all to `239.255.0.1:7650` = 7400 + 250 x domain 1 |

**The agent does not take the default profile.** An XRCE client compiled from the agent's own
examples, sending PX4 v1.16.2's exact participant XML (quoted from
`uxrce_dds_client.cpp`) to the real agent in the image, with the ROS side on
`fastdds_localhost.xml`; discovery judged by a long-lived rclpy node (see gotchas):

| FC `UXRCE_DDS_PTCFG` | ROS node discovers the agent's topic | DDS packets on WiFi |
|---|---|---|
| no profile anywhere (old behaviour) | yes | 31 |
| 0, ROS on the localhost profile | **never** (15 s) | 15 |
| 1 (PX4's localhost-only XML) | yes, 1.4 s | 0 |
| 2 + `agent_refs_local.xml` | yes, 0.2 s (1.6 s inside the real `run.sh` stack) | 0 |
| 2 + `agent_refs_network.xml`, ROS default transports | yes, 0.2 s | 20 (expected) |
| 2, agent started without its refs file | fails loudly: `reference file ... does not exist` | — |

The real PX4 then confirmed the `PTCFG=0` row: session up, its participant on WiFi, no `/fmu`
data in domain 1 — see Hardware status.

**Team workspace**: a package written on the host into `~/team_ws/src` built in the
competitor window (7 s), launched, and published (received by an rclpy subscriber); `build/`,
`install/` and `src/` owned by the host user; everything survived the container exiting. A
team `px4_msgs` is flagged by `team_build.sh`; the VIO pipeline stays on the underlay.

**Also run, all passing**: 121/121 unit tests in the image (`skybeta_flight_interface`,
`oakd_vslam_bringup`), `param_diff --selftest` all six steps against the committed reference
dump, `test_drone_env.sh` 30/30, `test_extlinux_fix.sh` 8/8 on the live `extlinux.conf`.

**Not yet verified**: `/fmu` data arriving through the 0.2.0 stack with `UXRCE_DDS_PTCFG=2` on
a real FC (the TELEM2 link dropped first); two drones on one access point.

## Known bugs

**Status: #0–#9 are all FIXED as of 2026-09-03** (branch `competition-base`, verified — see the
end-to-end bench section above). **#10–#14 are FIXED on branch `fleet-ready` as of
2026-10-12** (image `skybeta:0.2.0`, see the fleet bench section above); #13 additionally
needs `UXRCE_DDS_PTCFG=2` on every FC, which has not yet been carried through a real
`/fmu` data run. **#15 is FIXED by firmware (found 2026-10-20, fix verified 2026-10-21)**: on stock PX4
v1.16.2 the FC's TELEM2 stops transmitting after any agent loss until the FC is power-cycled;
FCs must run `px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4`, and until an FC does,
every stack stop needs an FC power cycle. The fixed entries are
kept here because the *diagnosis* is the valuable part:
each one records how the symptom presented and what the evidence was, so a regression is
recognisable. Anything still open is called out explicitly in its own entry.

### 0. FIXED — camera driver could not load at all (`libdiagnostic_updater.so`)

The image built cleanly but `run_slam.sh` was dead on arrival:

```
[oak_container] Failed to load library: Could not load library dlopen error:
libdiagnostic_updater.so: cannot open shared object file: No such file or directory
```

Root cause is a silent apt trap, not a missing dependency declaration:

1. `ros-humble-depthai-ros-driver-v3` correctly declares `Depends: ros-humble-diagnostic-updater`.
2. The **NGC base image already has `ros-humble-diagnostic-updater 4.0.6` installed**, and that
   build ships **only the Python module** — `dpkg -L` lists 68 files and **zero** `.so`.
3. apt sees the dependency as already satisfied and never upgrades it.
4. The composable node `dlopen`s the C++ library at runtime and dies.

Fix applied to the Dockerfile: name `ros-humble-diagnostic-updater` explicitly in the apt list,
which pulls the candidate (4.0.7) and provides `/opt/ros/humble/lib/libdiagnostic_updater.so`.
Verified by upgrading in a throwaway container before committing to a rebuild.

**Watch for more of this.** The base image carries a set of pre-installed ROS packages that apt
will not upgrade just because something depends on them. If another node fails to `dlopen` a
library, check `dpkg -l` against `apt-cache policy` before assuming the package is absent.

**Related smell, also fixed:** the Dockerfile used to install `ros2-testing-apt-source`, which
added `packages.ros.org/ros2-testing`; the candidate for several packages then came from
**testing**, whose versions move without notice, so two builds of this "pinned" image weeks apart
could differ. It is no longer installed, and the ROS source is now a dated snapshot — see the
Gotchas section.

### 1. FIXED — `jetson_uart_dma_fix.sh apply` exited 1 on success

`main()` ends with `[[ $do_reboot -eq 1 ]] && { ...; reboot; }` (`jetson_uart_dma_fix.sh:404`).
Without `--reboot` that list returns 1, and because `main "$@"` is the script's last command
under `set -Eeuo pipefail`, the script exits 1 despite having succeeded. Same pattern in the
`restore` branch.

Cascade: `02_uart_fix.sh` and `setup_and_build.sh` both call `sudo jetson_uart_dma_fix.sh apply`
under their own `set -euo pipefail`, so **both abort immediately after a successful patch** —
`02` never reaches its `sudo reboot`, and `setup_and_build.sh` never reaches step 3 (the image
build).

**Fix:** every branch of `main()` now ends with an explicit `exit 0`. `apply --dry-run` returns 0
and is covered by `host/tests/test_extlinux_fix.sh`.

### 2. FIXED — `extlinux.conf` gained a duplicate `FDT` line on every `apply`

This host currently has the line four times:

```
      FDT /boot/dtb/uart_dma_fix.dtb   ×4
```

The Python helper (`jetson_uart_dma_fix.sh:308-319`) iterates lines in order and reaches
`LINUX /boot/Image` *before* the pre-existing `FDT`, so `not inserted` is still true and it
inserts a fresh line — then separately rewrites the old one further down. Boots fine (all four
paths are identical, U-Boot takes one), but it grows without bound. Almost certainly a
downstream symptom of #1: the bogus exit 1 invites re-running `apply`.

**Fix:** the helper now does a pre-pass that deletes every `FDT` inside `LABEL primary`, then
inserts exactly one after `LINUX`. `apply` also no-ops when the patch is already live (detected
from `/proc/device-tree/bus@0/serial@3100000/iommus`, no sudo needed).
`host/tests/test_extlinux_fix.sh` asserts one FDT line, correct placement, byte-identical
remainder, and idempotency across two passes — run against this host's real 4-line file.

**Note:** this host's `/boot/extlinux/extlinux.conf` still physically has the four lines.
Collapsing them needs root, so `provision.sh` does it; it is harmless (all four paths identical).

**Follow-up 2026-10-12 — that claim was wrong.** `already_applied()` only required *a* matching
`FDT` line, so on a live-patched unit `apply` (and therefore `provision.sh`) reported "nothing to
do" and never collapsed the duplicates; a `sudo ... apply` + reboot on the bench left all four.
`apply` now notices the duplicates and rewrites `extlinux.conf` only (timestamped backup, DTB
untouched, no reboot needed). Verified unprivileged: plain `apply` now stops at "Run as root"
instead of "nothing to do", and `--dry-run` removes exactly the three extra lines.

### 3. FIXED — duplicate YAML keys dropped the resolution; but 1280×800 does **not** fit

`config/oakd_vslam.yaml` declared `left:` and `right:` **twice each**; YAML keeps the last
occurrence, so `i_width`/`i_height`/`i_fps` never reached the driver and it ran at the
documented 640×400 default. Merged into one block per side.

**The obvious fix — asking for the sensor's native 1280×800 — fails.** Measured driver-only,
six configs, 40 s each, `camera_info` only:

| config | rate | note |
|---|---|---|
| 1280×800 ×2 @30 | **23.3 Hz** | 143 MB/s — the ceiling; 23 % of frames lost |
| 1280×800 **left only** @30 | 30.0 Hz | proves it is throughput, not sensor/ISP/host |
| 1280×800 ×2 @20 | 20.0 Hz | clean, but costs a third of the rate |
| 1280×800 ×2 @30 + `i_low_bandwidth` | 23.3 Hz | no help |
| **1024×640 ×2 @30** | **30.0 Hz** | 118 MB/s, 2.56× the pixels — **shipped** |
| 960×600 ×2 @30 | 30.0 Hz | 104 MB/s, more margin |

**This is the XLink ceiling, not USB.** 143 MB/s is ~1.1 Gbps, far below USB 3.0 SuperSpeed's
5 Gbps — so the earlier reasoning here ("USB 3.0, therefore 1280×800 has headroom") was wrong.
The MyriadX's XLink transport tops out near 120–140 MB/s regardless of the USB link rate.

Worse, at 1280×800 the two eyes drop *different* frames: L/R header sync p95 rises to 33.3 ms
(one whole frame), few stereo pairs match, and cuVSLAM odometry collapses to **16.4 Hz** with
1154 jitter warnings. The rationale is recorded in a comment block in the YAML — do not "fix"
it back to 1280×800 without re-measuring.

### 3b. MEASURED (corrected) — camera timing is clean; the earlier jitter alarm was an artefact

Two 60 s `ros2 bag record` runs of the driver alone, analysed from **header stamps** (what
cuVSLAM actually compares), on power mode `MAXN_SUPER` (`pmode 0002`, already the max):

| | run 1 | run 2 |
|---|---|---|
| rate | 30.02 Hz | 30.02 Hz |
| interval p50 / p99 / max | 33.33 / 33.34 / 33.36 ms | 33.33 / 33.34 / 33.35 ms |
| intervals > 34 ms | **0** | **0** |

The device clock is essentially perfect. The earlier "27.9 Hz, max 98 ms" figure came from
`ros2 topic hz`, which measures **arrival** time through a CLI subscriber — and arrival does
jitter (p95 ≈ 39 ms, max 50–83 ms, 14–17 % of gaps > 34 ms; header→arrival latency p50
≈ 22 ms, max 73 ms). That is host delivery jitter, not dropped frames, and cuVSLAM's check
uses header stamps (`latest_ts - last_track_ts` in `visual_slam_impl.cpp`), so it does not
see it.

Under full load (driver + `bgr_to_mono` + cuVSLAM) cuVSLAM logged six over-threshold
warnings in 3.5 min: 933 ms at tracker start (initialisation), four of 34.1–35.1 ms (driver
stamp wobble tipping over a threshold with 0.7 ms of margin), and one 66.7 ms whose header
stamp sits 0.1 s *before* the recorder subscribed — a rosbag2-startup artefact. Steady-state
frame loss: **0 in 1954**. So: not a jitter crisis. Consequences:

- `jetson_clocks`/`nvpmodel` are not a fix here — it is already MAXN_SUPER.
- **`ros2 topic hz` is the wrong tool for this stream.** Re-run under load over 1026 samples it
  still said 27.1 Hz / max 150 ms while the same stream was 30.016 Hz by header stamp and
  30.18 Hz by C++ bag receive. The Python subscriber drops ~10 % in its own queue. Measure
  with `ros2 bag record -s mcap` and analyse header stamps.
- Over-threshold jitter is **warn-only** (see cuVSLAM grounding). The launch now sets 40 ms,
  which is cosmetic: it stops the driver's 32.2–33.6 ms stamp wobble tripping a 34.0 threshold.

**State after the WP2/WP3 fixes** (60 s mcap, full stack, header stamps, 1024×640):

| metric | before | after |
|---|---|---|
| resolution / encoding | 640×400 bgr8 | **1024×640**, `image_mono` is **mono8** |
| image_raw rate, max gap | 30.02 Hz, 33.37 ms | 30.017 Hz, 33.37 ms, 0 % > 34 ms |
| bgr8→mono8 added latency | p50 10.12 / p95 16.42 ms (Python) | **p50 5.18** / p95 17.25 ms (GPU) |
| frames lost in conversion | 2 of 1714 | **0 of 1743** |
| odometry | 30.001 Hz | 30.017 Hz, `vo_state`=1 on **1744/1744** |
| cuVSLAM track time | p50 2.5 / max 20.4 ms | p95 3.88 / max 13.25 ms |
| **IMU** | **26.3 Hz** | **199.1 Hz** (p50 5.02 ms) |
| jitter warnings | ~1/min | 1 in ~80 s |

- **The IMU root cause was the frequency defaults, not batching.** Leaving `i_acc_freq` /
  `i_gyro_freq` at their documented 480/400 yields 26.3 Hz on the BNO086; asking for 200
  explicitly gives 199.5. `i_sync_method: LINEAR_INTERPOLATE_ACCEL` beats `COPY` on regularity
  (p95 5.5 vs 8.8 ms). IMU is no longer a blocker for enabling `enable_imu_fusion`.
- Covariance from cuVSLAM is real (0/1962 identity fallbacks) but tiny — median σx ≈ 78 µm,
  5–7 orders below `EKF2_EVP_NOISE = 0.1`, which `EKF2_EV_NOISE_MD = 0` applies as a lower
  bound. EKF2 clamps all six. Bug #4 is therefore **unobservable as configured**; the fix is
  still correct (the raw values are anisotropic, x ≈ 5× y) but not urgent.

### 4. FIXED — covariance conversion applied a world-frame swap to body-frame quantities

`src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py:130-137`.

`position_variance = [pc[7], pc[0], pc[14]]` is **correct** — ENU→NED permutes x/y. But
`orientation_variance` and `velocity_variance` reuse that same permutation, and both are
body-frame quantities whose FLU→FRD mapping is `(x, -y, -z)`: sign flips only, which square
out of a variance. The node even publishes `velocity = [v.x, -v.y, -v.z]` with no swap while
its variance *is* swapped, so roll↔pitch and vx↔vy variances are transposed relative to the
data they describe. Should be `[pc[21], pc[28], pc[35]]` and `[tc[0], tc[7], tc[14]]`.

**Fix:** now `orientation_variance = [pc[21], pc[28], pc[35]]` and
`velocity_variance = [tc[0], tc[7], tc[14]]`; `position_variance` keeps the x/y swap. Covered by
unit tests with distinct diagonal values.

**Impact was nil as configured** — cuVSLAM's real variances are 5–7 orders below
`EKF2_EVP_NOISE`, which `EKF2_EV_NOISE_MD = 0` applies as a lower bound, so EKF2 clamps all six
regardless (see 3b). It matters the moment that bound drops: the raw values are anisotropic,
x ≈ 5× y.

### 5. FIXED — the `stereo:` config block was dead; nothing it set took effect

`config/oakd_vslam.yaml` sets `stereo.i_publish_left_rect`, `i_publish_right_rect`,
`i_publish_synced_rect_pair`, `i_depth_preset` and `i_reverse_stereo_socket_order`. With the
driver actually running, the published topic list is:

```
/oak/imu/data
/oak/left/camera_info   /oak/left/image_raw   (+ compressed / compressedDepth / theora)
/oak/right/camera_info  /oak/right/image_raw  (+ compressed / compressedDepth / theora)
```

No `*_rect` topics and **no depth topic at all** — the stereo/depth node is never instantiated,
so the whole `stereo:` block was inert. Not a bandwidth waste (nothing was published), but
`i_reverse_stereo_socket_order: true` was **also** doing nothing.

**Fix:** the block is deleted from `config/oakd_vslam.yaml`. If the stereo baseline sign ever
needs flipping, the lever is the **static TF**, not this parameter — cuVSLAM takes the baseline
from TF, not from `CameraInfo` (see the simulation-grounding section).

### 6. FIXED — `remain-on-exit` only applied to the first tmux window

`container_scripts/run_full_stack.sh` runs `tmux set-option -t "$SESSION" remain-on-exit on`
*before* creating the `slam` and `bridge` windows. `remain-on-exit` is a **window** option, so
targeting a session applies it to that session's current window only. The `slam` and `bridge`
panes therefore vanished on crash instead of holding their exit code — the opposite of what the
README promised.

**Fix:** `tmux set-option -g remain-on-exit on`, set in the same invocation that starts the
server (a bare `tmux start-server` does not survive — `exit-empty` defaults on). Verified: all
four windows report `remain-on-exit=on pane_dead=1`; under the old form only the first did.

### 7. FIXED — `bgr_to_mono` replaced by the GPU converter (NITROS does *not* negotiate)

The driver publishes `/oak/left/image_raw` as **`bgr8`** even though `CAM_B`/`CAM_C` are
`OV9282` *mono* sensors (the v3 driver reports `supportedTypes=[MONO, COLOR]` and publishes
these sockets as colour), and the v3 parameter reference exposes no encoding/output-type option
for them. So a host-side conversion to the `mono8` cuVSLAM needs is unavoidable.

The two Python `bgr_to_mono` nodes are gone, replaced by two `isaac_ros_image_proc`
`ImageFormatConverterNode`s (`encoding_desired: mono8`) loaded into the same composable
container as cuVSLAM. Measured: conversion latency **p50 10.12 → 5.18 ms**, frames lost
**2/1714 → 0/1743**.

**But it is not zero-copy.** cuVSLAM 3.2 uses a *Managed*NitrosSubscriber, so type negotiation
never completes and `type_negotiation_duration_s` changes nothing. The conversion itself runs on
the GPU; the hand-off to cuVSLAM is a plain in-container `sensor_msgs/Image`. The docstring says
so rather than claiming NITROS zero-copy. The converter's `nitros_image_rgb8` input label is
harmless here — the mono8 output is bit-exact with the bgr8 channels, since all three channels
are identical on a mono sensor.

The old docstring claim that the S2 has "color stereo sensors (OV9782)" was wrong twice over:
the part is OV9282, and the bgr8 comes from the driver, not the sensor.

### 8. FIXED — `px4_msgs` was pinned to `release/1.14` but the FC runs v1.16.2

The Dockerfile used to clone `px4_msgs` at branch `release/1.14`, and the README, the bridge
docstring and the launch comments all asserted "PX4 v1.14". The evidence above says the flight
controller is **v1.16 or newer**. uXRCE-DDS matches topics by message definition, so a
`px4_msgs` generation that doesn't match the firmware is exactly how you get an agent that
connects, advertises topics, and moves no data.

**Fix:** `docker/Dockerfile.jetson` pins `PX4_MSGS_REF=release/1.16` (recorded SHA
`392e831c…`) and adds `px4-ros2-interface-lib` at `release/1.16`. The installed
`VehicleOdometry.msg` carries `uint32 MESSAGE_VERSION = 0`, which only exists from 1.16.

**Caveat — the "moves no data" claim was never demonstrated.** Humble's rmw matches on type
*name*, not a hash (hashes arrive in Iron), and `VehicleOdometry`'s wire layout is identical
between 1.14 and 1.16. Data may well have flowed. The pin is right regardless; the real test is
the bench one: `check_topics.sh` must show a topic **echoing data**, not merely appearing in
`ros2 topic list`. That test is still blocked on the TELEM2 cable.

### 9. FIXED — `EKF2_EV_POS_*` does exist; the launch comment claimed it didn't

`src/oakd_vslam_bringup/launch/oakd_vslam.launch.py:24` justifies the static
`base_link → oak_parent_frame` TF with "(v1.14 has no `EKF2_EV_POS_*`)". Those parameters are
in the v1.14 parameter reference, in v1.15, in v1.16, **and** in this FC's own param dump
(`EKF2_EV_POS_X/Y/Z`, all `0.000000`).

The static TF is a perfectly good way to do it — but the comment's reasoning was false, and it
hid a real hazard: setting `EKF2_EV_POS_*` on the FC *and* leaving the static TF in place applies
**the camera lever arm twice**.

**Fix:** the comment is corrected. The TF is the single source of truth, driven by
`host/run.sh stack <x> <y> <z> [roll pitch yaw]`; `px4/params/03b_ekf2_vision.params` pins
`EKF2_EV_POS_X/Y/Z` to `0` and says why.

### 10. FIXED — every drone's DDS met every other drone's on a shared network

`host/run.sh` runs every container `--network host` and set no domain or transport, so all
drones' ROS nodes and agents shared domain 0 on the WiFi: drone A's `/fmu/in/*` setpoints
were visible to drone B's agent, and the 30 Hz stereo topics went out as multicast. Fixed by a
per-drone `ROS_DOMAIN_ID` (`/etc/skybeta/drone.env`, matched by `UXRCE_DDS_DOM_ID` on the FC,
now a per-unit exclusion in `param_diff`) and `SKYBETA_DDS_SCOPE=local` by default. See #13
for the part of this that did not work first time.

### 11. FIXED — team code vanished when the container exited

Every container is `--rm`, and the competitor guide said to `colcon build` inside it. Code,
build output and any `apt`/`pip` installs were lost on exit. Team code now lives in
`$TEAM_WS` on the host (mounted at `/root/team_ws`, built by `team_build.sh`, chowned back to
the host user); extra packages go in `$TEAM_WS/Dockerfile` (`host/run.sh build`).

### 12. FIXED — the image's pip packages were unpinned

`pip3 install depthai MAVProxy`. A rebuild for 0.2.0 resolved numpy 2.2.6 over the base's
1.26.4, opencv-python 5.0 and vtk (via MAVProxy 1.8.75), and depthai 3.11 — none of it ever run
on the vehicle. Pinned to what 0.1.0 shipped (`pip3 list` in that image): depthai 3.10.0,
MAVProxy 1.8.74, pymavlink 2.4.49, `numpy<2`.

### 13. FIXED (needs `UXRCE_DDS_PTCFG=2` on each FC) — the agent ignored the localhost profile

The first fix for #10 mounted `fastdds_localhost.xml` as the *default* profile, on the
assumption that it reaches MicroXRCEAgent too. It does not: with `UXRCE_DDS_PTCFG=0` PX4 sends
its own participant XML and the agent builds exactly that, on every interface. Worse, the ROS
nodes, now loopback-only, never discover it — so in local scope `/fmu/*` would silently never
reach the bridge. It only looked healthy on the bench because TELEM2 was dead. Measured
(table above), then seen on the real PX4. Fixed by `UXRCE_DDS_PTCFG=2` (in
`02_telem2_uxrce_dds.params`): PX4 asks for the participant by the reference
`px4_participant`, which the agent builds from `host/dds/agent_refs_<scope>.xml` passed as
`-r`. PTCFG=2 over 1 so that the scope switch in `drone.env` moves the agent too.

### 14. FIXED — `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` never reached the container

The competitor guide documented `COMPETITOR_PKG=my_team_nav host/run.sh stack`, but `run.sh`
never passed either variable into `docker run`, so the stack always launched
`competitor_example`. Now forwarded (and settable in `drone.env`).

### 15. FIXED (firmware `v1.16.2-1-g7a39c59`, verified 2026-10-21) — FC TELEM2 transmitter dies after any agent loss (NuttX STM32H7 TX DMA)

**How it presented** (fleet provisioning batch, drones 5, 6 and 16, 2026-10-20): the usbprov
stack test passed right after the FC booted. The test then tears the stack down with
`docker rm -f skybeta-stack` (MicroXRCEAgent dies with SIGKILL), and a second stack test
1-2 min later failed with "vehicle_status never reached 0.5 Hz (got none)", `fc.link` false and
0 `/fmu/out` topics. Only an FC power cycle or reboot restored the link. So on v1.16.2 **every
stack restart on an airframe loses the FC until it is power-cycled.**

**Bench evidence**, 2026-10-20, drone 16 (Jetson `skybeta-16` + its Pixhawk 6C on PX4 v1.16.2,
FC also on the laptop's USB for the MAVLink shell), in order:

1. Stack stopped: `uxrce_dds_client status` says "Running, disconnected". PX4 `dmesg`: "No ping
   response, disconnecting", then "init serial /dev/ttyS3 @ 921600 baud" (the client closed and
   reopened TELEM2). A pyserial sniff of `/dev/ttyTHS1` @ 921600 for 4 s on the Jetson: **0
   bytes**, although the client in that state sends a ping frame (0x7E HDLC) every second.
2. `uxrce_dds_client stop` + `uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600`: logs
   "init serial", still "disconnected", Jetson still 0 bytes in 14 s.
3. FC `reboot`: within 20 s the Jetson sees the 1 Hz ping frames again (184 bytes / 8 frames in
   8 s). So the Jetson's UART and its reopen path are fine; the fault is inside the FC.
4. Fresh FC, agent-only container on the Jetson (`run_agent.sh`): session established, FC
   "Running, connected", Payload tx 66620 B/s. `docker pause` of that container (the agent stops
   answering but the Jetson's port stays **open**, line electrically idle): after 9 s the FC logs
   "No ping response, disconnecting" + "init serial". `docker unpause`: the FC stays "Running,
   disconnected"; the agent saw exactly one "session established". The trigger is the FC's own
   disconnect-and-reopen, not anything the Jetson does to the line.
5. Client stopped, `echo HELLO-FROM-NSH > /dev/ttyS3` from nsh: the shell blocked forever and
   the Jetson saw 0 bytes. TELEM2's transmitter is dead at the NuttX level, for every opener,
   until reboot.
6. Jetson side checked independently: `serial-tegra.c` on L4T R36 drives TX idle-high while the
   port is closed and never sends a break, and the Jetson's reopened port received data fine
   (step 3).

**Root cause** (confirmed by reading the pinned sources): a NuttX STM32H7 serial-driver bug.
TELEM2 on fmu-v6c is `/dev/ttyS3` = UART5, built with `CONFIG_UART5_TXDMA=y`. On the last
`close()`, `up_dma_shutdown()` stops an in-flight TX DMA transfer but never zeroes
`dev->dmatx.length` / `dev->dmatx.nlength` nor the DMA stream's `NDTR` register. After the
reopen, `up_dma_txavailable()` requires all three to be zero before it hands new data to DMA,
so every later write only fills the 3000-byte ring buffer and nothing reaches the wire. The
v1.16.2 `uxrce_dds_client` trips it deterministically: after 3 missed pings it calls
`deleteSession()` (0 retries, it just sends DELETE_CLIENT), then `init()` → `deinit()` →
`close(_fd)` microseconds after that write, then reopens the port.

**Upstream.** Fixed by apache/nuttx PR #17203 "arch/arm/stm32h7: Fix UART TX DMA getting
stuck" (merged 2025-10-20), carried as PX4/NuttX commit
`d50ffd354495ce5d30814dba41d2eb70a32b8fa3` (PX4/NuttX PR #353) and brought into PX4-Autopilot
`main` by PR #25791 (2025-10-22); shipped in PX4 **v1.17.0**. **Not** in v1.16.0, v1.16.1,
v1.16.2 or `release/1.16` (NuttX submodule `886acbbdb4f061e5c0ce1a76afbcfa7cb7df9849`). The
matching report is PX4-Autopilot #25357 "[Bug] MicroXRCEAgent cannot reconnect after Ctrl+C,
PX4 stops sending data until reboot". Related but secondary: PX4-Autopilot #26022 / PR #26848
(`uxrce_dds_client` session-reconnection hardening, merged to `main` 2026-04-15, also not in
v1.16.2); it does not fix the DMA wedge.

- https://github.com/apache/nuttx/pull/17203
- https://github.com/PX4/NuttX/pull/353
- https://github.com/PX4/PX4-Autopilot/pull/25791
- https://github.com/PX4/PX4-Autopilot/issues/25357
- https://github.com/PX4/PX4-Autopilot/issues/26022
- https://github.com/PX4/PX4-Autopilot/pull/26848

**Trigger:** anything that makes the agent stop answering for ~3 s while the FC is up: a stack
stop, `docker rm -f`, a Jetson reboot, an agent crash.

**What does not help:**

- Reseating or replacing the TELEM2 cable. The line is fine; the FC's transmitter is wedged
  (steps 3 and 5).
- Restarting the client module (`uxrce_dds_client stop` / `start`, step 2).
- Stopping the agent gracefully instead of `docker rm -f` (inferred from step 4, not tested
  separately): either way the agent stops answering, and the FC's own missed-ping disconnect
  is the trigger.
- Parameters. None avoids it; `UXRCE_DDS_TX_TO` / `UXRCE_DDS_RX_TO` only change *when* the
  disconnect happens.

**Operating rule until every FC runs the fixed firmware:** after any stack stop or restart, or
a Jetson reboot, power-cycle (or `reboot`) the FC before expecting `/fmu` data. In the stack
test and in the field, **a passing first run followed by a silent second run is this bug, not
the connector.**

**Fix:** firmware. `px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` is PX4 v1.16.2
with only PX4/NuttX `d50ffd3` cherry-picked (build script, patch and notes in
`px4/firmware/txdma-fix/`). **Verified 2026-10-21 on drone 16:** the usbprov Pixhawk step
reflashed its FC from stock (`54f0455ffc`) to `7a39c59541` at 21:06, then three stack tests
were run back to back at 21:08:11, 21:08:48 and 21:09:15 with **no FC power cycle** between
them; each tore the stack down with `docker rm -f` and the next one passed with all topics at
rate after 20 s (`fc.link` true). On stock firmware the second run failed every time (section
above). The FC's own `dmesg` afterwards shows the healthy cycle for each stop/start: "No ping
response, disconnecting" → "init serial /dev/ttyS3 @ 921600 baud" → "synchronized with time
offset" (reconnected) — the step that never came on stock. `ver all`: PX4 git-hash
`7a39c595…`, OS git-hash `c21890c6…`, "PX4 version: 1.16.2 0" (dev type byte).

Rollout notes: the patched build reports version 1.16.2 with the *dev* type byte and git hash
`7a39c59541`, so the fleet flasher (`~/pixhawk_autoparams.py`) now compares the git hash from
`AUTOPILOT_VERSION.flight_custom_version`, not only the version, or it would skip every board
already on stock v1.16.2. Second gotcha found on the first reflash: `px_uploader.py` asks for
the bootloader with a MAVLink command addressed to sysid 1 and to broadcast 0, and PX4 only
honours commands addressed to its own `MAV_SYS_ID`. A provisioned FC carries `MAV_SYS_ID` =
drone number, so the uploader looped on "Attempting reboot" forever (drone 16, 20:58-21:02). The
flasher now sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` param1 = 3 to the board's real sysid
before launching the uploader. Fresh boards (sysid 1) never showed this.

Until an FC runs the patched build, the repo helps you recognise the wedge: `host/doctor.sh`'s
`uart-traffic` FAIL names this cause first when the link worked earlier in the FC's power cycle,
and the fleet dashboard's FC hint and the usbprov stack-test docs say to power-cycle the FC.

### 16. FIXED (2026-10-20) — patched UART DTB carried the golden board's CPU trim

**How it presented:** drone 16 (a massflash clone of drone 15) ran with `nproc` = 4,
`/sys/devices/system/cpu/possible` = `0-3`, and both `nvpmodel.service` and
`skybeta-jetson-clocks.service` failed at boot ("failed to read PARAM CPU_A78_4 ... NVPM
ERROR: failed to set power mode!"), so no MAXN_SUPER and no max clocks.

**Root cause:** each Orin Nano chip has a different 6 of the T234's 12 A78 cores enabled, and
UEFI trims the DTB it boots to the chip's fuses. `jetson_uart_dma_fix.sh apply` built
`/boot/dtb/uart_dma_fix.dtb` from `/sys/firmware/devicetree/base`, i.e. the tree UEFI had
already trimmed for drone 15 (`cpu@0..300`, `cpu@10200`, `cpu@10300`). On drone 16 UEFI removed
`cpu@10200/10300` (fused off there) and its real cores were not in the file, leaving 4. The
stock `kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb` has all 12 cpu nodes.

**Fix:** `apply` now patches the stock kernel DTB (`BASE_DTB=` overrides), verifies the staged
DTB keeps every stock cpu node, and treats a DTB with fewer cpu nodes as stale (`already_applied`
and the new no-root `current` mode), so `provision.sh`'s `uart_patch` phase re-runs on old
clones. The result differs from stock only by `iommus = <0x04 0x04>` on `serial@3100000`.
Verified on drone 16: one reboot, `nproc` 6, MAXN_SUPER, clocks pinned, UART still in its
IOMMU group. Rebooting the Jetson trips bug #15, so power-cycle the FC afterwards.
Also verified 2026-10-25 on drone 15 (rebuilt from stock: 12 cpu nodes, byte-identical to drone
16's file, still 6 cores after reboot) and on a fresh clone of the corrected image (6 cores,
MAXN_SUPER, no manual step). Older clones still carry the bad DTB: check `nproc`, and if it is
4 run `sudo host/jetson_uart_dma_fix.sh apply`, reboot, then power-cycle the FC.

## Gotchas when working here

- **On PX4 v1.16.2, never expect the FC to come back after a stack stop without an FC power
  cycle** (bug #15). Stopping the stack, `docker rm -f`, a Jetson reboot or an agent crash
  wedges TELEM2 TX on the FC; the next stack run gets 0 bytes until the FC is power-cycled or
  rebooted. Not the connector.
- The image is **aarch64-only** — the NGC Isaac ROS base publishes no amd64 manifest.
- Apt sources in the NGC base, precisely: `/etc/apt/sources.list.d/` holds `cuda`,
  `nvidia-l4t`, `ros2`, `nodesource`, `yarn`, `mosquitto` — **and** `/etc/apt/sources.list`
  itself carries `deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0`.
  So `ros-humble-isaac-ros-visual-slam` (3.2.6) comes from NVIDIA's pinned `release-3`
  channel, while `ros-humble-depthai-ros-driver-v3` (3.3.0) comes from the ROS repo. The old
  `rm -f .../nvidia-isaac-ros*.list` never matched anything (wrong directory) and has been
  removed. `docker/Dockerfile.jetson` now swaps the ROS source for a dated
  `snapshots.ros.org/humble/<SNAPSHOT_DATE>` (build ARG, currently `2026-08-07`, key vendored
  in `docker/ros-snapshots.key.asc`) and leaves the Isaac channel alone — that is what makes
  two builds weeks apart identical.
- `Micro-XRCE-DDS-Agent` is pinned to **v2.4.3** because the **PX4 v1.16 ROS 2 guide specifies exactly that tag** (`git clone -b v2.4.3 …`). The older "v3.x is wire-incompatible with v1.14" rationale in this repo's history was wrong. Don't bump it.
- **Image distribution is registry-agnostic and was verified end to end on 2026-10-16** (see
  [`docs/distribution.md`](docs/distribution.md)): `push_image.sh` into a throwaway `registry:2`, the
  registry serving the same content digest, `pull_image.sh` back out. Compressed transfer: NVIDIA base
  12.6 GB, skybeta layers ~0.6 GB; registry storage ~13 GB per version. `push_image.sh` takes the
  git-sha tag from the image (label `io.skybeta.git-sha`, else the local `<sha>` tag), **never from the
  checkout**, which moves on without rebuilding. Ubuntu's default `awk` is **mawk: no `{n}` regex
  intervals** -- use bash `[[ =~ ]]` in host scripts.
- `px4_msgs` (`release/1.16`) and `px4-ros2-interface-lib` (`release/1.16`) are cloned at build
  time; their resolved SHAs are baked into the image at `/root/skybeta_ws/*.sha` so a drifted
  rebuild is detectable.
- **Test DDS discovery with a long-lived node, not `ros2 topic list --no-daemon`.** With
  loopback-only transports, discovery is unicast and slower than the CLI's brief wait; the CLI
  reported "not found" for an agent participant that a 15 s rclpy node found in 0.2 s.
- **Never put `--` inside an XML comment in `host/dds/`.** It makes the file malformed, and Fast
  DDS then silently ignores the profile and falls back to every interface. Done twice in one
  session; `run.sh` and `test_drone_env.sh` now refuse a profile that does not parse.
- **Prove isolation with a packet capture**, e.g. `tcpdump -ni wlP1p1s0 'udp portrange
  7400-7700'` from `docker run --rm --network host alpine` (no sudo needed), and always run the
  `network`-scope control so you know the capture can see a leak.
- `host/run.sh stack` needs a TTY (`docker run -it`). Headless, e.g. over SSH from a script:
  `sleep infinity | script -qfc "host/run.sh stack" stack.log`. And never `pkill -f` a pattern
  that also appears in your own SSH command line — it kills the session.
- Editing `container_scripts/` rebuilds only the last layers (~1 s); editing anything under
  `src/` re-runs colcon for all four packages (~10 min on Orin Nano). `.dockerignore` keeps
  local `build/`/`install/`/`log/` and recorded bags out of the context so they don't bust that
  layer.
