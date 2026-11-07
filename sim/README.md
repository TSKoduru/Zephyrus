# skybeta sim profile — Isaac Sim 5.1.0 + Pegasus v5.1.0 + PX4 SITL v1.16.2

The whole skybeta stack — cuVSLAM, the PX4 odometry bridge, `skybeta_flight_interface`,
your competitor node — running against a simulated OAK-D and a simulated Pixhawk, so a
team can develop without a drone in hand.

**This is a spike, not a supported path yet.** Nothing here has been executed: the
development Jetson is aarch64 and there is no aarch64 Isaac Sim. Every file in `sim/`
has been checked against the upstream sources it calls (Pegasus v5.1.0,
isaac_ros_visual_slam release-3.2, Isaac Sim v5.1.0, PX4 v1.16.2) and both Python files
byte-compile and resolve their launch substitutions, but the run below is what turns
"should work" into "works". Run it in order and record what actually happened.

## Contents

| File | Runs where | What it is |
|---|---|---|
| `pegasus_skybeta.py` | **host**, under `isaac_run` | The Isaac Sim app: one Iris, PX4 MAVLink backend, ROS 2 backend, two cameras at a 7.5 cm baseline |
| `skybeta_sim.launch.py` | **container** | cuVSLAM + rgb8→mono8 converters + `/oak/*` mirrors + `MicroXRCEAgent udp4` + `px4_odom_bridge` |
| `px4_sitl.env` | **host** | Environment and the `param set` block for a hand-launched SITL |
| `../docker/Dockerfile.sim` | — | x86_64 image, same `src/` as the drone |

Three processes, three terminals: **Isaac Sim** (host), **PX4 SITL** (host), **the
stack** (container). Isaac Sim and Pegasus stay on the host deliberately — see the long
comment at the top of `docker/Dockerfile.sim`.

## Machine floor

NVIDIA's stated minimum for Isaac Sim 5.1.0: **RTX 4080 16 GB**, 32 GB RAM, Ubuntu
22.04, driver **≥ 595**. Pegasus itself was validated on driver 550.163.01. No Jetson —
Isaac Sim publishes no aarch64 build.

Version pinning is not optional: **Pegasus v5.1.0 requires Isaac Sim 5.1.0 and is not
compatible with Isaac Sim 6.0.** Every Pegasus release states this; each one breaks
against the neighbouring Isaac Sim.

---

## Install (host, once)

### 1. Isaac Sim 5.1.0

```bash
cd ~ && mkdir -p isaacsim && cd isaacsim
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip
./post_install.sh
./isaac-sim.selector.sh
rm isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

Then add the environment block from the [Pegasus installation
page](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
to `~/.bashrc`. It defines `ISAACSIM_PATH`, `ISAACSIM_PYTHON`, `ISAACSIM`, and the
**`isaac_run`** shell function that v5.1.0 requires — `isaac_run` unsets
`ROS_DISTRO`/`AMENT_PREFIX_PATH`/`PYTHONPATH`/`CMAKE_PREFIX_PATH` and strips
`/opt/ros/*` out of `LD_LIBRARY_PATH` before starting Isaac Sim, so that Isaac's
bundled Python 3.11 and its own Humble bridge libraries do not collide with a sourced
ROS 2 Humble. Copy it verbatim; do not improvise a wrapper.

Prove it before going further:

```bash
isaac_run --help
ISAACSIM_PYTHON -c "print('Hello World.')"
ISAACSIM_PYTHON ${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py
```

### 2. Pegasus v5.1.0

```bash
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
cd PegasusSimulator && git checkout v5.1.0
cd extensions
ISAACSIM_PYTHON -m pip install --editable pegasus.simulator
```

Also enable it once through the GUI (Window → Extensions → settings → add
`<repo>/extensions` to the search path → enable on the third-party tab); that first
enable is what installs the extension's own Python requirements.

`extensions/pegasus.simulator/config/configs.yaml` holds `px4_dir` (default
`~/PX4-Autopilot`) and `px4_default_airframe` (`gazebo-classic_iris`).
`pegasus_skybeta.py` overrides both from the environment, so you do not have to edit it.

### 3. PX4-Autopilot v1.16.2

Note the deviation: Pegasus documents v1.14.3. We pin **v1.16.2** because that is what
the fleet's Pixhawk 6C runs (see `CLAUDE.md`), and a sim on different firmware from the
vehicle is a sim that lies about parameters.

```bash
sudo apt install git make cmake python3-pip
pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16.2
git submodule update --init --recursive
make px4_sitl_default none      # "none": no Gazebo, MAVLink simulator only
```

`make px4_sitl_default none` produces `build/px4_sitl_default/bin/px4`. There is no
Gazebo anywhere in this setup, despite the airframe being called `gazebo-classic_iris`
— that name only selects `ROMFS/px4fmu_common/init.d-posix/airframes/10015_*`, whose
autostart routes to `px4-rc.mavlinksim`, i.e. `simulator_mavlink start -c 4560`.

### 4. The sim container

```bash
cd <skybeta repo>
docker build -f docker/Dockerfile.sim -t skybeta-sim:$(cat VERSION) .
```

The base is `nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907`
(~40 GB, the amd64 twin of the tag `Dockerfile.jetson` pins). Everything on top is the
same ~1 GB delta the drone image carries, minus depthai.

---

## The spike run

Terminals:

```
T1  Isaac Sim   isaac_run sim/pegasus_skybeta.py
T2  PX4 SITL    source sim/px4_sitl.env; ... bin/px4 ...
T3  the stack   docker run ... skybeta-sim:<tag>
```

Keep `ROS_DOMAIN_ID` identical in all three (0 is the default everywhere; `px4_sitl.env`
exports it, and PX4's `rcS` copies it into `UXRCE_DDS_DOM_ID` automatically).

### Step 1 — PX4 SITL alone

```bash
source sim/px4_sitl.env
mkdir -p "$PX4_RUN_DIR" && cd "$PX4_RUN_DIR"
"$PX4_SITL_BIN" "$PX4_ROMFS" -s "$PX4_RC_SCRIPT" -i 0
```

**Expect:** the airframe is resolved, then PX4 *stops*:

```
INFO  [init] found model autostart file as SYS_AUTOSTART=10015
INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
```

and no `pxh>` prompt. **That is a pass.** PX4 is the TCP *client* here
(`SimulatorMavlink.cpp` calls `connect()` in a retry loop) and Pegasus is the server, and
`simulator_mavlink start` deliberately blocks the rest of `rcS` "until time is
initialized by the HIL_SENSOR messages from the simulator". So nothing after that line —
including `uxrce_dds_client` — has started yet.

**Failure:** `ERROR [init] Unknown model gazebo-classic_iris` means the checkout is not
v1.16.2 or the ROMFS path is wrong. No output at all means the binary was not built.

### Step 2 — Pegasus connects

```bash
cd <skybeta repo>
isaac_run sim/pegasus_skybeta.py
```

First run is slow: the environment USD streams from NVIDIA's asset server.

**Expect:** an Isaac Sim window with the Iris spawned in the Curved Gridroom, the
timeline already playing (the script calls `timeline.play()`), and in the Isaac console

```
[skybeta] PX4 MAVLink: tcpin:localhost:4560  autolaunch=False
[skybeta] ROS 2 topics under /drone1/  (cameras: left, right)
[skybeta] camera intrinsics applied: 1280x800 fx=762.69 fy=762.69 cx=640.0 cy=400.0 hfov=80.0deg baseline=0.0750m
```

and, in **T2**, PX4 unblocking:

```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
...
pxh>
```

The `pxh>` prompt appearing *is* the lockstep handshake succeeding: the prompt cannot
appear until HIL_SENSOR has advanced PX4's clock.

Now set the parameters (they persist because `$PX4_RUN_DIR` is a real directory — this
is why `px4_autolaunch` is off; Pegasus's launcher uses a temporary directory that is
deleted with the process). Paste the block from `sim/px4_sitl.env`, then:

```
pxh> param save
pxh> uxrce_dds_client stop
pxh> uxrce_dds_client start -t udp -p 8888
```

`UXRCE_DDS_SYNCT` is read once when the client starts, so it needs that restart (or a
full PX4 restart) to take effect.

**Failure:** if PX4 stays on "Waiting for simulator…", Pegasus never opened the server.
Check the Isaac console for a Python traceback, and that no other PX4 or Gazebo instance
already owns 4560 (`ss -ltnp | grep 4560`). If Isaac Sim starts and immediately exits,
the driver is below 595 or the GPU is below the 16 GB floor.

### Step 3 — the agent and the `/fmu` topics

```bash
docker run --rm -it --runtime nvidia --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID=0 -v "$PWD/sim:/root/skybeta_ws/sim:ro" \
  skybeta-sim:$(cat VERSION) bash
# inside:
MicroXRCEAgent udp4 -p 8888        # or: run_agent.sh  (XRCE_TRANSPORT=udp4 is baked in)
```

**Expect:** a session, then a burst of creates:

```
[1737...] info     | UDPv4AgentLinux.cpp   | init                     | running... | port: 8888
[1737...] info     | Root.cpp              | create_client            | create     | client_key: 0x...
[1737...] info     | ProxyClient.cpp       | create_publisher         | publisher created  | ...
```

In a second container shell:

```bash
ros2 topic list | grep -c '^/fmu'          # expect >= 50
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

The topic *count* is the weak test; the **echo returning a message is the real one**.
A `px4_msgs` generation that does not match the firmware can advertise topics that carry
no data. Note the `_v1` suffix: PX4 v1.16 versions `VehicleStatus`, and
`skybeta_flight_interface/topics.py` resolves these names from the installed `.msg`
files rather than hard-coding either spelling.

**Failure:** no client at all → `uxrce_dds_client` is not running (check `pxh> uxrce_dds_client status`)
or the container is not on the host network. Topics present but the echo hangs →
`ROS_DOMAIN_ID` differs between PX4 and the container, or `px4_msgs` is on the wrong
branch.

### Step 4 — the cameras actually publish

**This is the step most likely to fail, and the reason this spike exists.**

`MonocularCamera.update()` in Pegasus v5.1.0 has its frame grab commented out:

```python
#self._state["image"] = self._camera.get_rgba()[:, :, :3]
...
#if self._depth:
#    self._state["depth"] = self._camera.get_depth()
```

Reading only that, you would conclude no image reaches ROS 2. Reading `ros2_backend.py`
says otherwise: `add_monocular_camera_writter()` attaches an Omniverse Replicator writer
(`LdrColorSDROS2PublishImage`) to `data["camera"]._render_product_path`, so the pixels go
GPU → ROS 2 without ever passing through that commented-out line. The commented code
only removes a CPU-side copy nobody consumes. **The analysis says it works; this step is
where you find out.**

```bash
ros2 topic list | grep drone1
ros2 topic hz  /drone1/left/color/image_raw
ros2 topic echo /drone1/left/color/image_raw --field encoding --once
ros2 topic echo /drone1/left/color/camera_info --once
```

**Expect:** `/drone1/{left,right}/color/{image_raw,camera_info}`, `/drone1/sensors/imu`,
`/drone1/state/pose`; ~30 Hz; encoding `rgb8`; and camera_info with `width: 1280`,
`height: 800`, `distortion_model: plumb_bob`, `d: [0,0,0,0,0]`,
`k: [762.7, 0, 640, 0, 762.7, 400, 0, 0, 1]`.

The images appear a few seconds after play, not instantly: `MonocularCamera.update()`
discards its first 100 calls, and the writer is created on the first call that gets past
that.

Three things to check against, in order of how badly they bite:

* **No `*/color/image_raw` topic at all.** The RGB path really is dead in this build.
  Fallback A: patch `monocular_camera.py` to un-comment the `get_rgba()` line — cheap,
  but note it does not affect the writer, so if the writer is the problem this will not
  help. Fallback B (the real one): drop `pub_graphical_sensors` and build the camera
  publishers with Isaac Sim's own ROS 2 camera graph
  (`isaacsim.ros2.bridge` `ROS2CameraHelper`), attaching them to the same camera prims
  `MonocularCamera` created. The topic names are then yours to choose, so
  `skybeta_sim.launch.py` needs no change beyond `namespace`/`vehicle_id`.
* **Rate well under 30 Hz.** The publish gate is `int(60 / frequency)` against a 60 Hz
  render tick, so 30 Hz is achievable only if the renderer holds 60 fps with two
  1280×800 render products. If it does not, lower `SKYBETA_IMAGE_FPS` to 20 or 15 (both
  divide 60) and raise `image_jitter_threshold_ms` to match, or drop the resolution.
  Do not leave cuVSLAM being fed at an unstable rate and then tune EKF2 around it.
* **Image upside down.** Set `SKYBETA_CAMERA_ROLL=3.14159` — Pegasus's own default
  camera orientation is `[0, 0, 180]`, and this script deliberately uses `[0, 0, 0]`
  (forward, upright) because Isaac's "world" camera convention is +X forward / +Z up.
  If it turns out inverted is correct, change **both** `SKYBETA_CAMERA_ROLL` and the
  launch's `camera_roll`.

### Step 5 — cuVSLAM tracks

```bash
# in the container
ros2 launch /root/skybeta_ws/sim/skybeta_sim.launch.py
```

This starts the converters, cuVSLAM, the static TFs, the `/oak/*` mirrors, the agent and
the bridge. If you already have an agent running in another shell, add `run_agent:=false`.

```bash
ros2 topic hz /oak/left/image_mono
ros2 topic echo /visual_slam/status --field vo_state
ros2 run tf2_ros tf2_echo base_link left
ros2 run tf2_ros tf2_echo odom base_link
```

**Expect:** `image_mono` at the camera rate; `vo_state: 1` (Success — `0` is Unknown, `2`
is Failed) once the vehicle moves; `base_link → left` at
`[0.100, 0.0375, 0.000]`; and `odom → base_link` tracking the motion. Move the vehicle by
arming and flying it in step 7, or just drag it in the Isaac viewport with the timeline
playing.

**Failure:**

* *"Unsupported distortion model"* thrown at startup: `camera_info.distortion_model` is
  something cuVSLAM 3.2 does not accept. Step 4 should have caught it — the sim writes
  zero OpenCV coefficients precisely so `read_camera_info()` reports `plumb_bob`.
* *`vo_state` stuck at 0/2, or an immediate crash on the TF lookup*: the static TFs are
  not up, or `camera_optical_frames` (`['left','right']`) does not match the `frame_id`
  Pegasus stamps (which is the camera *name*). Rename in both files together.
* *Tracking drifts absurdly*: the baseline cuVSLAM believes is not the one Isaac
  rendered. cuVSLAM takes the stereo geometry **from TF**, never from the CameraInfo `P`
  matrix (`visual_slam_impl.cpp` calls `GetLatestTransform(base_frame,
  camera_optical_frames[idx])`), so `baseline` in the launch and `SKYBETA_BASELINE` in
  the Pegasus script must agree. Both default to 0.075.
* *Featureless white walls*: "Curved Gridroom" is sparse. `SKYBETA_ENV="Full Warehouse"`
  gives a stereo front end something to hold on to.

### Step 6 — vision odometry reaches PX4

```bash
ros2 topic hz   /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

**Expect:** ≈30 Hz, and `pose_frame: 2` (`POSE_FRAME_FRD`). FRD, not NED, for the same
reason as on the drone: cuVSLAM's `odom` frame is initialised at the rig's startup pose
and has no heading reference, which is the literal definition of FRD.

Then, at `pxh>`:

```
pxh> listener estimator_status_flags
```

**Expect:** `cs_ev_pos: True` and `cs_ev_yaw: True` (this is what `EKF2_EV_CTRL=9` buys).
If they stay false, EKF2 is receiving the messages but rejecting them — check
`EKF2_EV_CTRL` really saved, and look at `listener estimator_innovation_test_ratios`.

`EKF2_EV_DELAY` is a **measurement**, not the 50 ms placeholder. Run
`tools/latency_probe.py` against this pipeline and use what it reports; the sim's latency
is not the drone's, so record the two separately.

### Step 7 — fly the square

```bash
# container, third shell
ros2 launch competitor_example competitor.launch.py \
    allow_mode_request:=true square_side_m:=5.0
```

`allow_mode_request:=true` is **SITL only** — it enables the `/skybeta/request_offboard`
service. On the vehicle the safety pilot flips the RC switch and this stays false.

```
pxh> commander arm
```

```bash
ros2 service call /skybeta/request_offboard std_srvs/srv/Trigger {}
ros2 topic echo /skybeta/state
```

`skybeta_flight_interface` never arms — there is no arm service — so arming happens at
the `pxh>` prompt, and the mode request only succeeds once the offboard setpoint stream
is already running (which the flight interface starts on launch).

**Expect:** hover at 1.5 m for 5 s, then a 5 m square. Score it against ground truth,
which Pegasus publishes for free:

```bash
ros2 topic echo /drone1/state/pose        # ENU ground truth, "map" frame
```

Acceptance is **< 0.3 m RMS** between `/drone1/state/pose` and the commanded corners.
Then kill the competitor node (`Ctrl-C` on that launch) and confirm PX4 does what
`COM_OBL_RC_ACT` says — the flight interface keeps streaming a hold setpoint, so the
vehicle should hover rather than drift.

**Failure:** *offboard rejected* → the setpoint stream is not at >2 Hz, or the vehicle is
not armed. *Immediately disarms* → RC-loss failsafe; `COM_RCL_EXCEPT` bit 2 (value 4) is
in `px4_sitl.env` for exactly this. *Flies a rotated square* → the ENU/NED handling
disagrees somewhere; compare `/skybeta/setpoint/pose` against `/drone1/state/pose`
before blaming the bridge.

---

## Risk: PX4 v1.16.2 against a Pegasus tested on v1.14.3

Pegasus v5.1.0's README badge says PX4-Autopilot 1.14.3 and its install docs check out
that tag. We deliberately run v1.16.2. What that does and does not put at risk:

The Pegasus↔PX4 interface is **MAVLink HIL + lockstep** — `HIL_SENSOR`, `HIL_GPS`,
`HIL_STATE_QUATERNION`, `HIL_ACTUATOR_CONTROLS` over TCP 4560 — and none of those
messages changed between 1.14 and 1.16. The pieces that *are* version-sensitive have been
checked at the v1.16.2 tag rather than assumed: `10015_gazebo-classic_iris` still exists,
`px4-rc.simulator` still routes a non-`gz`, non-`sihsim` airframe to
`px4-rc.mavlinksim`, `simulator_mavlink start -c $((4560+i))` still opens the same port
in the same client role, and `rcS` still starts `uxrce_dds_client -t udp -p 8888`
unconditionally. What has *not* been checked is behavioural: rotor mixing (v1.15
reworked control allocation), the `CA_ROTOR*` values in the Iris airframe versus
Pegasus's `input_scaling`/`zero_position_armed` thrust model, and lockstep timing under
the v1.16 scheduler. Symptoms to watch for are a vehicle that will not hold altitude, or
one that spins up unevenly. If that happens, the diagnostic is to run the *same* Pegasus
scene against a v1.14.3 build: if it flies there and not on 1.16.2, it is the airframe
model, not the sim. Do not conclude anything about the vision pipeline from a vehicle
that cannot hover.

## Fallback: Gazebo Harmonic

If Isaac Sim proves unavailable (no RTX card in the room, driver stuck below 595) or the
camera path in step 4 cannot be made to publish, the escape hatch is PX4's own
**Gazebo (gz) Harmonic** integration, which ships inside PX4-Autopilot and needs no
extra simulator install: `make px4_sitl gz_x500_mono_cam` or `gz_x500_depth` gives a
quadrotor with a camera, and `ros_gz_bridge` republishes the Gazebo camera topics as
`sensor_msgs/Image` and `sensor_msgs/CameraInfo`. A stereo pair means editing the model
SDF to add a second camera at a 7.5 cm offset — `x500_mono_cam` carries one — and
bridging both. Everything downstream of the camera topics is unchanged: the same
`ImageFormatConverterNode` → cuVSLAM → `px4_odom_bridge` chain, the same static TFs, the
same `MicroXRCEAgent udp4 -p 8888` (Gazebo SITL starts the same `uxrce_dds_client`), so
`skybeta_sim.launch.py` needs only new input topic names. What is lost is rendering
fidelity — Gazebo's camera is far less photoreal, which flatters a feature tracker in
some scenes and starves it in others — and the ability to reuse Isaac's warehouse assets.
It is a worse sim, not a different architecture.

---

## Upstream quirks these files work around

Recording them here so nobody "fixes" the workarounds.

| Where | Quirk | What we do |
|---|---|---|
| `ros2_backend.py:78,132` | Topics are `namespace + str(vehicle_id) + "/"`, and the *default* namespace is already `"drone"+id` — so `vehicle_id=1` with no namespace gives `/drone11/...` | Pass `namespace="drone"` explicitly |
| `monocular_camera.py:150,158` | RGB and depth frame grabs are commented out in `update()` | Nothing — the ROS 2 path is a Replicator writer on the render product, not this. Step 4 verifies |
| `ros2_backend.py:409` | The depth writer keys off a `"depth"` entry the sensor no longer writes, so it is never created | `depth: False` anyway; cuVSLAM never reads depth |
| `monocular_camera.py:136` | The first 100 `update()` calls return early | Cameras appear ~3 s after play |
| `monocular_camera.py:103` (Isaac 5.1) | `set_rational_polynomial_properties()` is deprecated: it ignores the `intrinsics` fx/fy it is handed and ignores `max_fov`/`diagonal_fov`, deriving fx/fy from the prim's focal length and aperture instead | `pegasus_skybeta.py::_apply_camera_intrinsics()` sets focal length, both apertures and the `OmniLensDistortionOpenCvPinholeAPI` fx/fy/cx/cy directly, before the CameraInfo writer snapshots them |
| `camera_info_utils.py:56` | `read_camera_info()` reports `rational_polynomial` unless k4/k5/k6 are zero — and cuVSLAM 3.2's `FillCameraDistortion` throws on anything outside {`plumb_bob`, `equidistant`, `rational_polynomial`, empty} | Zero distortion coefficients ⇒ `plumb_bob`, which is also what `rectified_images: True` asserts |
| `px4_launch_tool.py:41` | `px4_autolaunch` runs PX4 with `cwd=TemporaryDirectory()`, so `param save` does not survive | `px4_autolaunch` defaults to **off**; launch PX4 by hand from `$PX4_RUN_DIR` |
| `uxrce_dds_client.cpp:215` | `UXRCE_DDS_SYNCT` is read once at client start | Restart the client (or PX4) after setting it |
| `examples/8_camera_vehicle.py:95` | Passes `{"update_rate": 60.0}` to `MonocularCamera`, which reads `frequency` and ignores `update_rate` | Use `frequency` |
