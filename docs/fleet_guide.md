# Fleet guide — bringing up 10+ identical drones

For the operator standing in front of a rack of airframes, not the competitor writing
navigation code. If you are the latter, you want
[`competitor_guide.md`](competitor_guide.md).

**The whole job is making ten vehicles identical and being able to prove it.** Every step
below has a verification command and an expected output. A step is not done when the
command has been typed; it is done when the output matches. Steps 1–8 run once per drone,
in order, and none of them can be usefully done out of order.

Budget roughly 90 minutes per airframe the first time, 30 once you have the rhythm — the
image pull (step 3) and the sensor calibration (step 2) dominate.

Before you start, run [`hardware_checklist.md`](hardware_checklist.md) §1–§4 on the
airframe. In particular the TELEM2 connector: a marginal one will make steps 4 and 5 fail
intermittently and there is no way to tell that from a configuration error.

---

## The order, at a glance

| # | Step | Where | Gate |
|---|---|---|---|
| 1 | Flash PX4 v1.16.2 | QGC | `ver all` reports `v1.16.2` |
| 2 | Baseline params, then **per-unit** calibration | QGC | `param_diff` clean; QGC calibration all green |
| 3 | `host/provision.sh` | Jetson | `host/doctor.sh` exits 0 |
| 4 | Stage 01 — TELEM2 as MAVLink | QGC + Jetson | MAVProxy HEARTBEAT |
| 5 | Stage 02 — TELEM2 as uXRCE-DDS | QGC + container | `check_topics.sh` **echoes data** |
| 6 | Stage 03a — height reference | QGC | bench hover holds altitude |
| 7 | Stage 03b — EV fusion, then (7c) autostart | QGC / flasher + Jetson | `cs_ev_pos` / `cs_ev_yaw` true; delay re-checked on this airframe; with 7c, true again after a cold boot with nobody logged in |
| 8 | Record the per-drone baseline | repo | `param_diff` reports zero unexplained drift |

---

## 1. Flash PX4 v1.16.2

The firmware is vendored: [`../px4/firmware/px4_fmu-v6c_default-v1.16.2.px4`](../px4/firmware/px4_fmu-v6c_default-v1.16.2.px4).
Verify it before you push it to ten boards.

```bash
sha256sum -c px4/firmware/px4_fmu-v6c_default-v1.16.2.px4.sha256
```

```
px4_fmu-v6c_default-v1.16.2.px4: OK
```

Then in QGroundControl: **Vehicle Setup → Firmware → Advanced settings → Custom firmware
file**, plug the FC in over USB, and select that file.

> ### Flashing erases every parameter.
>
> This is not a caveat, it is the reason the rest of this guide is ordered the way it is.
> A flashed board comes up on firmware defaults: no airframe, no calibration, no serial
> configuration. **Do not flash a drone you have already configured** unless you are
> prepared to redo steps 2 through 7. If you must, save its parameters first
> (QGC → Parameters → Tools → **Save to file**) into `px4/params/fleet/<serial>.params`.

Flashing all ten to the same version is the point. Mixed firmware in a fleet produces
bugs that appear on one airframe and not another, which is the most expensive kind.

### Verify

On the QGC MAVLink console (**Analyze Tools → MAVLink Console**):

```
ver all
```

```
FW version: v1.16.2 0
FW git-hash: ...
...
HW arch: PX4_FMU_V6C
```

The line that matters is `FW version: v1.16.2`. Anything else and stop here — every
downstream pin in this repo (`px4_msgs @ release/1.16`, `px4-ros2-interface-lib @
release/1.16`) assumes the 1.16 message generation.

---

## 2. Baseline parameters, then per-unit calibration

Two halves, and the order between them matters.

### 2a. Load the fleet baseline

**QGC → Vehicle Setup → Parameters → Tools → Load from file →**
[`../px4/params/00_baseline.params`](../px4/params/00_baseline.params), then **reboot the
FC**.

907 parameters: the airframe (`SYS_AUTOSTART = 4001`, generic quadcopter), the controller
gains, the battery configuration, the failsafes, the logging profile — everything that
should be byte-identical across the fleet. It was generated from a dump of the reference
airframe with every per-unit value stripped out; the header of the file lists exactly what
was removed and why, and `tools/param_diff.py --make-baseline` regenerates it.

Then load [`../px4/params/04_rc_only_fd_roll_off.params`](../px4/params/04_rc_only_fd_roll_off.params)
(`COM_RC_IN_MODE=0`: RC transmitter only, no joystick; `FD_FAIL_R=0`: roll failure detector
off) and reboot again. An FC left on the May-07 value
`COM_RC_IN_MODE=1` (Joystick only) **ignores the RC transmitter**.

Then load [`../px4/params/05_rc_switches_board_rot.params`](../px4/params/05_rc_switches_board_rot.params)
(`RC_MAP_ARM_SW=7`, `RC_MAP_KILL_SW=5`, `SENS_BOARD_ROT=8`: FMU board roll 180°, upside down)
and reboot again. The May-07 dump had arm on channel 5 and kill on 7, the other way round, and
no board rotation. Run the level-horizon calibration (2b) **after** 05 is in, so the trim is
measured with the rotation applied.

Then [`../px4/params/06_rc_radiomaster.params`](../px4/params/06_rc_radiomaster.params): the
fleet RadioMaster's AETR stick order (`RC_MAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4`; the May dump
was TAER, which scrambles the sticks) and the RC1..RC18 endpoints, trims and deadzones read off
drone 20 after QGC's Radio calibration. One transmitter model fleet-wide makes this a fleet
value, so step 2b's radio calibration is only needed for a drone on a different transmitter.
Then [`../px4/params/07_airframe_geometry.params`](../px4/params/07_airframe_geometry.params):
the real rotor positions (`CA_ROTOR<n>_PX/PY`, the dump had PX4's ±1 m placeholders) and the
range-finder height `EKF2_RNG_POS_Z`. The fleet flasher loads 04 to 07 for you.

### 2b. Calibrate **this** airframe

**QGC → Vehicle Setup → Sensors**, and run all of:

- [ ] Compass
- [ ] Gyroscope
- [ ] Accelerometer
- [ ] Level Horizon
- [ ] **Radio** (Vehicle Setup → Radio)

> ### Calibration must never be cloned between drones.
>
> `CAL_*`, `SENS_BOARD_*` and the `RC<n>_MIN/MAX/TRIM` endpoints describe **this board, in
> this orientation, wired to this receiver**. Accelerometer bias and level-horizon trim are
> properties of one specific IMU die and one specific mounting; magnetometer offsets are
> properties of one specific airframe's iron and current paths; RC endpoints are properties
> of one transmitter+receiver pair.
>
> Copy them from another drone and you get a vehicle that *believes* it is level when it is
> not. On the bench it looks fine. On take-off it drifts, and the drift is proportional to
> how different the two boards happened to be. It is a silent failure with no error message
> anywhere, which is why those parameters are excluded from the baseline by construction
> rather than by discipline.

This is also why 2a comes first: the baseline does not contain calibration, so loading it
after calibrating would not undo your work — but loading it *first* means the reboot it
requires is out of the way before you start waving the airframe around.

### Verify

```bash
tools/param_diff.py --master /dev/ttyTHS1,57600 --no-staged
```

Or, if the serial link is not up yet (it is not, at this point in the guide — that is
step 4), save from QGC and diff the file:

```bash
# QGC -> Parameters -> Tools -> Save to file -> px4/params/fleet/<serial>.params
tools/param_diff.py --dump px4/params/fleet/<serial>.params --no-staged
```

```
  expected  px4/params/00_baseline.params                  907 params
  actual    px4/params/fleet/<serial>.params              1129 params
  merged expected set: 907 params

OK -- no drift. Every expected parameter matches.
```

`--no-staged` because none of the staged files have been loaded yet. Calibration values
show up as `EXTRA` only with `--all`; they are outside the baseline on purpose.

In QGC, every entry under Sensors should be green, and Radio should show live stick
movement on all channels.

---

## 3. Provision the Jetson

```bash
git clone <your fork> skybeta && cd skybeta
host/provision.sh --dry-run          # read it before you run it
host/provision.sh
```

Five phases, recorded one per line in `$HOME/.skybeta/state`:

| Phase | Root? | Does |
|---|---|---|
| `prereqs` | yes | apt deps, the `03e7` OAK-D udev rule, registers the docker `nvidia` runtime, `dialout` + `docker` group membership |
| `uart_patch` | yes | the R36.5 UARTA DMA/IOMMU device-tree patch. **Needs one reboot the first time** |
| `power` | yes | `nvpmodel` to the MAXN model (resolved by name — on the Orin Nano Super that is ID 2, not 0) + `jetson_clocks`, made to survive reboot |
| `pull` | no | pulls the image at the tag in `VERSION`. Needs `REGISTRY` set |
| `doctor` | no | the health check |

### Give the drone its identity

Every Jetson carries `/etc/skybeta/drone.env`: drone number, team, **DDS domain**, camera
mount and the team workspace. It is what turns ten identical images into ten separate drones
on one network. Start from the template:

```bash
sudo install -D -m 0644 host/drone.env.example /etc/skybeta/drone.env
sudoedit /etc/skybeta/drone.env        # DRONE_ID=3, ROS_DOMAIN_ID=3, TEAM=..., CAMERA_*
```

`ROS_DOMAIN_ID` = the drone number, unique across the fleet, 1–99. Step 5 sets the FC's
`UXRCE_DDS_DOM_ID` to the same number. Leave `SKYBETA_DDS_SCOPE=local`: DDS then stays on the
Jetson (loopback + shared memory) and nothing — not even the 30 Hz camera streams — goes over
the WiFi. Flip it to `network` only to debug from a laptop on the same domain.

`host/run.sh` reads this file on every start and prints the drone, domain and scope in its
banner; without it, it warns and runs as an anonymous drone on domain 0.

### The reboot, and resuming

The first run stops after `uart_patch` and tells you to reboot. That is a successful run,
not a failure — it exits 0.

```bash
sudo reboot
# then, after it comes back:
host/provision.sh          # detects the live patch and carries on
```

```bash
host/provision.sh --list             # which phases are done
host/provision.sh --from power       # re-run this phase and everything after
host/provision.sh --skip pull        # repeatable
```

Running it a second time on a finished machine is a no-op that exits 0. That property is
what makes it safe to put in a fleet script.

### Pull the image

The registry is a deployment choice, so everything here uses a `REGISTRY` placeholder.
Set it to wherever your fleet's images live and keep it consistent across all ten drones.
[`distribution.md`](distribution.md) compares the options (LAN `registry:2`, TLS, a hosted
private registry, or none) and covers plain-HTTP registries and pull-only credentials.

```bash
export REGISTRY=<your-registry>/<your-org>
host/pull_image.sh $(cat VERSION)
```

The image is **42.6 GB**, of which **40.3 GB is the public NGC Isaac ROS base**.
`pull_image.sh` fetches that base from `nvcr.io` **first**, so the subsequent registry
pull finds every base layer already on disk and transfers only the skybeta delta (~2.3 GB
unpacked, ~0.6 GB over the wire). Pull skybeta first and you drag 13 GB across the venue
network — per drone, ten times. With no route to `nvcr.io` the pull still succeeds: the base
layers then come from your registry too.

At a venue with a poor uplink, stand up a pull-through cache:

```bash
docker run -d -p 5000:5000 --restart=always --name mirror registry:2
MIRROR=192.168.1.10:5000 REGISTRY=<your-registry>/<your-org> host/pull_image.sh $(cat VERSION)
```

**`latest` is never pushed and never pulled** — `pull_image.sh` refuses it outright. A
moving tag is how a ten-drone fleet silently splits mid-competition. Record the **digest**
the pull prints in the fleet table; that, not the tag, proves two drones are running the
same bits.

### Verify

```bash
host/doctor.sh
echo "doctor exit: $?"
```

Twelve checks, `PASS`/`WARN`/`FAIL` one line each, exit 1 if anything FAILed: L4T release ·
UARTA `iommus`/`dmas` in the live device tree · `3100000.serial` bound to `serial-tegra`
owning `ttyTHS1` · exactly one `FDT` line in `extlinux.conf` · `dialout` + `docker`
membership · docker `nvidia` runtime · OAK-D enumerated · UART traffic · image present ·
power model · drone identity (`drone.env`, DDS domain) · team workspace.

Two expected non-failures at this point in the guide:

- **`uart-traffic` FAILs** until step 4 puts MAVLink on TELEM2. That is correct — nothing
  is transmitting yet.
- **`oakd-usb` at `03e7:2485`, 480 Mbps is a PASS.** The MyriadX enumerates twice; it
  re-appears as `03e7:f63b` at 5000 Mbps once depthai pushes firmware. Use
  `host/doctor.sh --boot-camera` if you want the real link speed.

Also record the image digest:

```bash
docker inspect --format '{{index .RepoDigests 0}}' skybeta:$(cat VERSION)
```

---

## 4. Stage 01 — TELEM2 as MAVLink

**QGC → Parameters → Tools → Load from file →**
[`../px4/params/01_telem2_mavlink.params`](../px4/params/01_telem2_mavlink.params), then
**reboot the FC**.

Six parameters. It puts TELEM2 on MAVLink at **57600** — deliberately slower than the
921600 the stack eventually uses, because 57600 is far more tolerant of marginal wiring
while you are still establishing that the wire works at all.

The one rule: `MAV_1_CONFIG` and `UXRCE_DDS_CFG` must never both point at `102`. One
serial port, one owner. This file sets `UXRCE_DDS_CFG = 0` for exactly that reason.

### Verify

```bash
mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors
```

```
Connect /dev/ttyTHS1 source_system=255
Log Directory:
Telemetry log: mav.tlog
online system 1
STABILIZE> Received 1174 parameters
```

An `online system 1` line within a second or two is the pass. **Do not move on until you
have it.** DDS on a dead wire looks identical to DDS misconfigured, and you will spend the
difference debugging the wrong layer.

While you are here, save the parameter dump — `param download` then `param save
px4/params/fleet/<serial>.params` — it is free and step 8 wants it anyway.

### If it is silent

Go to [`hardware_checklist.md`](hardware_checklist.md) §1 and work down it. In order:
connector seating, then loopback (`host/uart_loopback_test.py` with pins 8 and 10
jumpered), then the crossed TX/RX wiring, then parameters. Not the other way round — see
[`debugging.md`](debugging.md) §1.

---

## 5. Stage 02 — TELEM2 as uXRCE-DDS

**QGC → Parameters → Tools → Load from file →**
[`../px4/params/02_telem2_uxrce_dds.params`](../px4/params/02_telem2_uxrce_dds.params),
then **reboot the FC**.

This hands TELEM2 from MAVLink to the uXRCE-DDS client at 921600. **MAVProxy will stop
working on this port — that is the expected outcome, not a regression.**
The file also sets **`UXRCE_DDS_PTCFG = 2`**: PX4 asks the agent for its DDS participant by
reference, and the agent builds it on loopback from `host/dds/agent_refs_local.xml`. Leave it
at 0 and the stack connects, the agent reports a session, and not one `/fmu` message reaches
the bridge — the ROS nodes are on loopback and the agent's participant is not.

**Then set `UXRCE_DDS_DOM_ID` by hand** to this drone's `ROS_DOMAIN_ID` — the value in its
`/etc/skybeta/drone.env` (convention: the drone number) — and reboot the FC again. It is
deliberately not in the 02 file: it is the one DDS setting that must *differ* between drones.
Leave ten drones on domain 0 and every agent on the WiFi can see every other drone's
`/fmu/in/*` setpoints.

```bash
host/run.sh stack        # tmux windows: agent, slam, bridge, competitor
```

or, to test the link alone:

```bash
host/run.sh shell
run_agent.sh
```

> **Operating rule on PX4 v1.16.2: after stopping or restarting the stack (or rebooting the
> Jetson), power-cycle the FC before the next run.** When the agent goes away the FC's
> TELEM2 transmitter stays dead until the FC reboots, so the next run gets 0 bytes and no
> `/fmu` data; reseating the cable does not help ([`CLAUDE.md`](../CLAUDE.md) bug #15).

### Verify — and this is the step people get wrong

In a second shell on the container:

```bash
check_topics.sh
```

```
[check] /fmu topics: 51 total, 27 under /fmu/out/
         /fmu/out/failsafe_flags
         /fmu/out/sensor_combined
         /fmu/out/timesync_status
         /fmu/out/vehicle_attitude
         /fmu/out/vehicle_local_position
         /fmu/out/vehicle_status_v1
         ...

[check] /fmu/out/vehicle_status_v1 ... OK (message received)
[check] /fmu/out/timesync_status   ... OK (message received)
```

> ### The pass condition is **data echoing**, not topic names appearing.
>
> `ros2 topic list` showing fifty `/fmu/*` topics proves exactly one thing: the agent
> built a session with the client and created the datawriters. The **agent** creates
> those endpoints from the client's own advertisement. It does not prove PX4 is filling
> them, and it does not prove your `px4_msgs` generation matches the firmware.
>
> A mismatched `px4_msgs` is precisely the failure that connects, advertises a full topic
> list, and moves no data — and it will happily pass a topic-count check while the
> estimator downstream never sees a single message. That is why `check_topics.sh` resolves
> each topic against the live list and then does a timed `--once` echo. **A returned
> sample is the pass. The count is decoration.**
>
> Note the `_v1` suffix. PX4 v1.16 versions its messages, so `/fmu/out/vehicle_status` is
> spelled `/fmu/out/vehicle_status_v1`. Nothing in the stack hard-codes either spelling;
> `check_topics.sh` and `skybeta_flight_interface/topics.py` both resolve it at runtime.

### Verify — and prove nothing reaches the WiFi

Data echoing is half the pass. The other half, for a drone that will share a network, is that
none of its DDS traffic goes over the air. From the Jetson, with the stack running (no sudo
needed — the capture runs in a throwaway container):

```bash
docker run --rm --network host alpine sh -c "apk add -q tcpdump && \
  timeout 30 tcpdump -ni <wifi-iface> 'udp portrange 7400-7700' | wc -l"
```

```
0
```

Zero packets in 30 s is the pass (bench: 0 with the full stack up). Then run the control once:
set `SKYBETA_DDS_SCOPE=network`, restart the stack, repeat — you must now see packets (bench:
71), or the capture is not looking at the right interface. Set the scope back to `local`.

If the echo passes but packets appear in local scope, the agent's participant is on the
network: `UXRCE_DDS_PTCFG` is not 2. If the echo *fails* in local scope but passes with
`SKYBETA_DDS_SCOPE=network`, it is the same cause seen from the other side.

If the list is healthy but the echo hangs: check `ROS_DOMAIN_ID` against
`UXRCE_DDS_DOM_ID`, then `uxrce_dds_client status` on the FC, then that the installed
`VehicleOdometry.msg` carries `MESSAGE_VERSION` (the quick proof you are on the 1.16
generation).

If the session never establishes at all, and the cable is anything other than new: reseat
it and re-run. A *negative* UART result on a suspect connector is not a result.

---

## 6. Stage 03a — height reference, then a bench hover

**QGC → Parameters → Tools → Load from file →**
[`../px4/params/03a_height_reference.params`](../px4/params/03a_height_reference.params),
then **reboot the FC**.

Five parameters, PX4 v1.16's documented **"Indoor (non-flat terrain)"** height configuration:
the range finder is the reference, the barometer is the backup.

| Parameter | Value | |
|---|---|---|
| `EKF2_HGT_REF` | `2` (range) | trust the range finder: one downward ~1° laser, PX4 VL53L1X driver — the airframe's only ground sensor (there is **no optical flow** on any skybeta airframe, see `08_no_optical_flow.params`) |
| `EKF2_BARO_CTRL` | `1` | baro takes over whenever range is not fused |
| `EKF2_RNG_CTRL` | `1` (conditional) | range only below 1 m/s and below `EKF2_RNG_A_HMAX`, with a height-above-ground consistency gate, so boxes and tables do not drag the height around. `2` (always) is PX4's flat-floor variant |
| `EKF2_RNG_A_HMAX` | `3.0` m | inside the VL53L1X driver's 4 m maximum |
| `EKF2_GPS_CTRL` | `0` | indoor |

Set each drone's ceiling to match: `GF_MAX_VER_DIST` (e.g. 3 m), in its fixes file.

This step depends on nothing else in the stack — not vision, not the UART, not the
container — so it can be done at any point after step 2. It is here because it is the
runaway-climb fix, and you want it in before anything hovers.

The state it corrects: the reference airframe shipped with `EKF2_HGT_REF = 2` (Range) but
`EKF2_BARO_CTRL = 0` and GPS off. Whenever the range finder stopped delivering (above its
range, over an obstacle, a rejected sample) there was **no height source left**: the
estimate free-ran, the controller read it as sinking, and commanded a climb that never
terminated. Range as the reference was never the problem; having nothing behind it was.

### Verify — bench hover check

**Props off for the static part.** Then, with a safety pilot and
[`hardware_checklist.md`](hardware_checklist.md) §5 complete:

```bash
# on the Jetson, container shell:
ros2 topic echo /fmu/out/vehicle_local_position \
  --qos-reliability best_effort --qos-durability transient_local \
  --field z
```

1. Static, on the bench, props off, 60 s: `z` should sit within a few centimetres of a
   constant. A slow monotonic ramp here means the height reference is still wrong — stop.
2. Lift the airframe by hand about a metre and hold: `z` should go **negative** by about a
   metre (PX4 is NED; down is positive, so up is negative) and come back when you set it
   down. Latency of a few hundred ms is fine; a wrong sign is not.
3. Props on, pilot hovers manually at ~1.5 m for 60 s in Altitude or Position mode: the
   altitude should hold with no trend. Record the bag.

```bash
host/run.sh bag --duration 60
```

A drift of more than ~0.2 m over 60 s of stationary hover means do not proceed to step 7.

---

## 7. Stage 03b — EV fusion

`03b_ekf2_vision.params` contains `EKF2_EV_DELAY = 70`, **measured** on the bench Orin on
2026-10-16 (image → bridge publish p50 68.7 ms, p95 102 ms, plus ~1–2 ms for the serial
hop). It replaced a guessed 50. Fusing stale position as if it were current turns any
oscillation into a growing orbit (toilet-bowl suspect #3), so re-check the number on each
airframe with 7a. Identical hardware should land within a few ms.

### 7a. Measure

With the full stack running:

```bash
# tools/ is not inside the image -- copy the probe into the running stack:
docker cp tools/latency_probe.py skybeta-stack:/tmp/latency_probe.py
docker exec -it skybeta-stack bash -lc 'source skybeta_env.sh; python3 /tmp/latency_probe.py --seconds 30'
```

Drone 16, 2026-11-03: full chain p50 60.4 ms, p95 106 ms (driver leg 36.5 ms) → `60`.

The probe reports two legs against the node's own clock: the **driver leg**
(`/oak/left/image_raw` header stamp → arrival) and the **full chain**
(`/fmu/in/vehicle_visual_odometry` `timestamp_sample` → arrival). Because the bridge
copies the cuVSLAM header stamp into `timestamp_sample`, and cuVSLAM copies the image
stamp into that, the second number covers image → depthai → mono conversion → cuVSLAM →
bridge → local DDS.

It does **not** include the uXRCE-DDS serialisation and the 921600-baud serial hop to the
FC. Those are small but real. Treat the printed p50 as a starting point and cross-check it
on the vehicle with PX4's own method: set `SDLOG_PROFILE` bit 7 ("EKF2 replay"),
hand-move or fly, then run the replay and inspect the innovation timing.

Record the number in the fleet table. It is per-airframe in principle, though on identical
hardware it should not vary much — a drone that is an outlier here is telling you
something.

### 7b. Load, with your measured value

Edit `EKF2_EV_DELAY` in your copy of the file, or set it directly in QGC after loading.
Then **QGC → Parameters → Tools → Load from file →**
[`../px4/params/03b_ekf2_vision.params`](../px4/params/03b_ekf2_vision.params), and
**reboot the FC**.

`EKF2_EV_CTRL = 9` is bit 0 (horizontal position) + bit 3 (yaw). Vertical position
(bit 1 → 11) and 3D velocity (bit 2 → 13) stay **off** until vision altitude is trusted —
vertical is the weakest axis for pure stereo VO, and `enable_imu_fusion` is off, so there
is no inertial or gravity constraint.

`EKF2_EV_POS_X/Y/Z` stay at zero. The camera lever arm is carried by the static TF that
`host/run.sh stack <x> <y> <z> <roll> <pitch> <yaw>` drives. Setting both applies the
offset twice.

Deliberately **not** in this file: `EKF2_RNG_CTRL` (height stays 03a's job). Optical flow
is not a choice: no airframe has a flow sensor, and `08_no_optical_flow.params` (in the
flasher's default set) turns the inherited flow configuration off. VIO is therefore the
**only** horizontal position source — if it loses tracking, Position mode has nothing left
and PX4 falls back to Altitude behaviour.

With the fleet flasher instead of QGC (FC on the laptop's USB; the default set is replaced
when you name files, so name them all):

```bash
cd ~/pixhawk_params && ./flash.sh 00_baseline.params 02_telem2_uxrce_dds.params \
  03a_height_reference.params 04_rc_only_fd_roll_off.params 05_rc_switches_board_rot.params \
  06_rc_radiomaster.params 07_airframe_geometry.params 08_no_optical_flow.params \
  03b_ekf2_vision.params --set EKF2_EV_DELAY=<measured> --once --drone-id <N> \
  --device /dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00
```

### Verify

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --field pose_frame --once
```

```
2
```

`2` is `POSE_FRAME_FRD`. If you see `1` (`POSE_FRAME_NED`), something overrode the
bridge's default and you are about to reproduce the toilet bowl — see
[`debugging.md`](debugging.md) §4.1.

Then, on the MAVLink console:

```
listener estimator_status_flags
```

Expect `cs_ev_pos: True` and `cs_ev_yaw: True`. That pair is what `EKF2_EV_CTRL = 9`
buys, and it is the proof EKF2 is *fusing* the vision rather than merely receiving it. If
they stay false, EKF2 is rejecting the measurements — check
`listener estimator_innovation_test_ratios`.

The same flags without the MAVLink console, from the stack:
`ros2 topic echo --once /fmu/out/estimator_status_flags` (and `failsafe_flags`:
`local_position_invalid: false` is what lets Position mode engage; `vehicle_local_position`:
`xy_valid: true`).

Bench, props off, before flying: move the airframe a metre by hand and confirm
`/fmu/out/vehicle_local_position` tracks it. Yaw it 90° and confirm the cuVSLAM and FC
headings differ by a constant, not a growing amount.

Arming checks that fail **only on the bench** and are not a VIO problem: "USB connected"
(PX4 refuses to arm with the FC on USB), no RC (Manual/Altitude/Position need the
transmitter), "Strong magnetic interference" (a warning only at `COM_ARM_MAG_STR=2`; mag
is not fused while EV yaw is), and the global-position check (GPS modes only).

### 7c. Turn it on at boot (per drone)

Once 7b verifies, make the drone fly VIO with nobody logged in: the stack starts at boot as
`skybeta-stack.service`.

```bash
cd ~/skybeta && sudo host/autostart.sh enable      # once; asks for the sudo password
host/autostart.sh status
```

The service waits up to 90 s for NTP time sync before starting, and restarts the stack if the
clock steps later: the Jetson has no RTC, and a clock step after the camera driver started makes
every vision timestamp stale, so EKF2 drops vision and Position mode is lost (drone 16,
2026-11-03). A stack you start by hand after a cold boot has the same trap; start it once
`timedatectl` says `System clock synchronized: yes`.

From then on: power the drone up, keep it **still for the first minute** (cuVSLAM sets its
origin and heading from the startup pose), and Position mode is available on the mode
switch. The service (`host/stack_service.sh`) restarts the whole stack if the agent, SLAM
or bridge window dies — safe only on the patched FC firmware (bug #15); a crashed
competitor window is logged and left alone.

| To | Run |
|---|---|
| look at the windows | `docker exec -it skybeta-stack tmux attach -t skybeta` (Ctrl-b d detaches) |
| restart it (re-reads `drone.env`) | `docker rm -f skybeta-stack` — back within ~10 s |
| stop it until the next boot | `sudo systemctl stop skybeta-stack` |
| logs | `journalctl -u skybeta-stack -f` |
| turn autostart off | `sudo host/autostart.sh disable` |

`host/run.sh stack` refuses to start a second copy and prints the same table;
`host/doctor.sh` shows a `stack-autostart` line and skips its UART sniff while the stack
holds the port. The usbprov stack test removes `skybeta-stack` itself, so stop the
service before running it on an autostart drone. Enabled on: **drone 16** (2026-11-03).

---

## 8. Record the per-drone baseline and prove there is no drift

The last step, and the one that makes the previous seven auditable.

```bash
# QGC -> Vehicle Setup -> Parameters -> Tools -> Save to file
#   ->  px4/params/fleet/<serial>.params
```

Commit it. That file is this airframe's ground truth: baseline + staged files + its own
calibration, as actually running.

### Verify

```bash
tools/param_diff.py --dump px4/params/fleet/<serial>.params
```

```
  expected  px4/params/00_baseline.params                  907 params
  expected  px4/params/01_telem2_mavlink.params              6 params
  expected  px4/params/02_telem2_uxrce_dds.params            6 params
  expected  px4/params/03a_height_reference.params           5 params
  expected  px4/params/03b_ekf2_vision.params                9 params
  expected  px4/params/04_rc_only_fd_roll_off.params         2 params
  expected  px4/params/05_rc_switches_board_rot.params       3 params
  expected  px4/params/06_rc_radiomaster.params             94 params
  expected  px4/params/07_airframe_geometry.params           9 params
  actual    px4/params/fleet/<serial>.params              1174 params
  merged expected set: 1037 params

OK -- no drift. Every expected parameter matches.
```

**Require zero unexplained drift**, and the exit status is 0 only when there is none, so
this drops straight into a script:

```bash
for f in px4/params/fleet/*.params; do
  tools/param_diff.py --dump "$f" >/dev/null || echo "DRIFT: $f"
done
```

Anything reported under `CHANGED` is either a mistake or a deliberate per-airframe
deviation. **Deliberate deviations must be written into the fleet table**, otherwise the
next person cannot tell them from mistakes — which is the entire failure mode this step
exists to prevent.

Two things the tool does *not* flag, by design:

- **`EXTRA`** — parameters on the vehicle that the baseline does not mention. That is
  calibration, RC endpoints and flight counters, per-unit by construction. Pass `--all`
  to see them.
- Anything at all if you point it at the wrong file. `--dump` takes MAVProxy `.parm` and
  QGC `.params` interchangeably; it does not check that the dump came from the drone you
  think it did.

Against a live FC instead of a saved dump:

```bash
tools/param_diff.py --master /dev/ttyTHS1,57600
```

Note 57600: that only works while stage 01 is loaded. After stage 02, TELEM2 belongs to
the DDS client and there is no MAVLink on it — dump from QGC over USB instead.

Re-run this after any firmware change, any parameter session, and before every
competition day.

---

## Fleet table

Keep one row per airframe. The columns are the ones that have actually distinguished two
drones in practice.

| Serial | Jetson hostname | Domain ID | FC firmware | Image tag | Image digest | Cal. date | `EKF2_EV_DELAY` (ms) | Camera x/y/z, r/p/y | TELEM2 cable | Last param_diff clean | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `SN0001` | `skybeta-01` | 1 | v1.16.2 | `0.1.0` | `sha256:…` | 2026-09-03 | | | new 2026-09-03 | 2026-09-03 | reference airframe |
| `SN0002` | `skybeta-02` | 2 | | | | | | | | | |
| `SN0003` | `skybeta-03` | 3 | | | | | | | | | |
| `SN0004` | `skybeta-04` | 4 | | | | | | | | | |
| `SN0005` | `skybeta-05` | 5 | | | | | | | | | |
| `SN0006` | `skybeta-06` | 6 | | | | | | | | | |
| `SN0007` | `skybeta-07` | 7 | | | | | | | | | |
| `SN0008` | `skybeta-08` | 8 | | | | | | | | | |
| `SN0009` | `skybeta-09` | 9 | | | | | | | | | |
| `SN0010` | `skybeta-10` | 10 | | | | | | | | | |
| Jetson `1423825093262` | `skybeta-16` | 16 | v1.16.2-1-g7a39c59 | `0.3.0` | `sha256:6688685e1623` | | 60 | 0.1053/0/0.0304, 0/0/0 | | 2026-11-03 | 03b loaded; VIO position hold + stack autostart **on by default** |

Per-airframe physical details — OAK-D serial, IMU part, kill-switch channel, radio
calibration date — go in the record card at the end of
[`hardware_checklist.md`](hardware_checklist.md).

---

## Routine operations

### Rolling out a new image

```bash
# build machine, once:
host/build_image.sh
REGISTRY=<your-registry>/<your-org> host/push_image.sh

# each drone:
REGISTRY=<your-registry>/<your-org> host/pull_image.sh $(cat VERSION)
```

Bump `VERSION` when you cut an image the fleet is meant to run, so a semver tag never
points at two different builds. Roll out to **one** drone, fly it, then the rest. Ten
drones on a bad image is ten drones down. The code and the image move together (the
checkout's `VERSION` names the image `run.sh` starts); the update order per drone is in
[`distribution.md`](distribution.md#updating-a-fleet).

### Before every competition day

1. `host/doctor.sh` on every Jetson — exit 0.
2. `tools/param_diff.py` against every `px4/params/fleet/*.params` — no drift.
3. Image digest on every drone matches the fleet table.
4. [`hardware_checklist.md`](hardware_checklist.md) §5 per airframe.

### When one drone behaves differently

The fleet table and `param_diff` exist for exactly this. In order: compare the image
digest, then `param_diff` the two airframes' saved dumps against each other
(`--dump A --baseline B --no-staged`), then the physical record card, then start
debugging with [`debugging.md`](debugging.md).
