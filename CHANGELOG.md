# Changelog

One entry per image version (`VERSION`, tag `skybeta:<VERSION>`). The evidence behind each
change is in [`CLAUDE.md`](CLAUDE.md); this file says what changed and what you must do.

## 0.3.2 — arm, then Position (2026-11-06, branch `fleet-ready`)

### You must do this

- **Move each drone to the `skybeta:0.3.2` image.** The pilot's order is now arm, **then** switch to
  Position, within 10 s of arming (`COM_DISARM_PRFLT`). No FC parameter changes.

- **The go signal moved from arming to the switch to Position after arming.** Requested by the fleet
  owner: arming with the switch already on Position (0.3.1's go signal) now gives no autonomy, and
  that is how a pilot flies by hand.
- **New rule: the vehicle must be on the ground.** Otherwise a pilot flying in Altitude who flips to
  Position to hold would hand the vehicle to team code in the air. `flight_interface` subscribes to
  `/fmu/out/vehicle_land_detected` (in the patched firmware's topic list), and `/skybeta/state` gains
  `landed`. Unknown counts as not landed. A team launch file needs no change. It has not flown.

## 0.3.1 — autonomy after arming (2026-11-06, branch `fleet-ready`)

### You must do this

- **Move each drone to the `skybeta:0.3.1` image.** 0.3.0 has no auto-Offboard. The order is
  Position, **then** arm: arming in another mode and switching to Position does not start autonomy.
  Team launch files must start `flight_interface` with `auto_offboard_on_arm:=true`.

- **Autonomy now starts when the pilot arms in Position.** Competitor code could not fly before:
  nothing selected Offboard (`RC_MAP_OFFB_SW=0`, no Offboard in `COM_FLTMODE*`). `flight_interface`
  has a new `auto_offboard_on_arm` parameter (on in `competitor.launch.py`), and a team launch
  file must set it too. **The vehicle takes off under team code straight after arming.** Load
  `09_autonomy_takeover.params` (`COM_RC_OVERRIDE=3`, so sticks take over from Offboard) on every
  FC, and check the handover with the props off before flying (competitor guide §4). This changes
  `src/`, so it needs a new image. It has not flown.

- **Power-cycle the flight controller after any stack stop** (stack stop or restart,
  `docker rm -f`, Jetson reboot, agent crash) until the patched firmware is flashed on that FC.
  On PX4 v1.16.2 the FC's TELEM2 transmitter stays dead after the agent goes away, and the next
  stack run gets 0 bytes and no `/fmu` data. Reseating the TELEM2 cable does not clear it.
- **Re-run the fleet flasher (or load `px4/params/04_rc_only_fd_roll_off.params`) on every FC
  provisioned before 2026-10-22.** They carry `COM_RC_IN_MODE=1` (Joystick only), inherited
  from the reference airframe's May-07 dump, so the RC transmitter is ignored. 04 sets
  `COM_RC_IN_MODE=0` (RC only) and `FD_FAIL_R=0` (roll failure detector off).

- **Re-run the fleet flasher on every FC flashed before 2026-11-01, then redo level horizon.**
  `px4/params/05_rc_switches_board_rot.params` moves the arm switch to RC channel 7 and the kill
  switch to channel 5 (the May-07 dump had them swapped) and sets `SENS_BOARD_ROT=8` (FMU roll
  180°, upside down, as on drone 20). The level-horizon trim is per-unit and depends on the
  rotation, so calibrate it after 05 is in. `06_rc_radiomaster.params` carries the RadioMaster's
  AETR stick order and RC1..18 calibration (the baseline had the May transmitter's TAER order:
  **a re-flash scrambled the sticks on a RadioMaster drone**), `07_airframe_geometry.params` the
  real rotor positions and range-finder height instead of PX4's ±1 m placeholders.

- **Re-run the fleet flasher on every FC (2026-11-03): `08_no_optical_flow.params`** is now in
  the default set. No skybeta airframe has an optical-flow sensor -- the only ground sensor is
  one downward ~1° laser rangefinder -- but the baseline (May-07 dump) carried
  `EKF2_OF_CTRL=1`, `SENS_EN_PMW3901=1`, `SENS_TFLOW_CFG=103`. 08 turns all three off. VIO is the
  only horizontal position source. Baseline regenerated: 904 rows.

### Added

- **VIO position hold on by default, drone 16 (2026-11-03).** Its FC carries 03b
  (`EKF2_EV_CTRL=9`) with `EKF2_EV_DELAY=60` measured on that airframe, plus 08, and the Jetson
  starts the stack at boot. Bench-verified: `cs_ev_pos`/`cs_ev_yaw` true, `xy_valid` true,
  `local_position_invalid` false. Dump: `px4/params/fleet/drone16-vio-2026-11-03.params`.
- **Boot-time stack, per drone: `sudo host/autostart.sh enable`** installs
  `skybeta-stack.service` (template `host/systemd/skybeta-stack.service.in`), which runs
  `host/stack_service.sh`: the stack is started detached and restarted if the agent, SLAM or
  bridge window dies. Needs the patched FC firmware. `host/run.sh` gained `SKYBETA_DETACH=1`
  (`docker run -d -t`) and, when the service already runs the stack, says how to attach, restart
  or stop it. No image change.
- `host/stack_service.sh` waits up to 90 s for NTP sync before starting the stack and restarts it
  on a system clock step: the Jetson has no RTC, NTP steps the clock after boot, and the OAK-D
  driver's frame stamps then lag by the step, so EKF2 rejects every vision sample (drone 16: 317 s
  stale, position lost mid hand test). **A hand-started stack after a cold boot has the same
  problem:** start it after `timedatectl` reports synchronized, or restart it after the sync.
- `host/doctor.sh`: `stack-autostart` line (13 checks now), and `uart-traffic` is skipped while a
  container holds `/dev/ttyTHS1` -- sniffing a live agent's UART corrupts its session.

### Fixed / documented

- Fleet guide 7a: `tools/latency_probe.py` is not in the image (`/workspace/tools` does not
  exist); it is now `docker cp`'d into the running stack.

- **New staged files `05_rc_switches_board_rot.params`** (`RC_MAP_ARM_SW=7`, `RC_MAP_KILL_SW=5`,
  `SENS_BOARD_ROT=8`), **`06_rc_radiomaster.params`** (AETR map + RC1..18 calibration) and
  **`07_airframe_geometry.params`** (`CA_ROTOR<n>_PX/PY`, `EKF2_RNG_POS_Z`), all read off drone 20
  (`px4/params/fleet/drone20-radiomaster-2026-11-01.params`) and registered in
  `tools/param_diff.py`; `00_baseline.params` regenerated without those rows (907 parameters).
  `param_diff` now treats only `SENS_BOARD_[XYZ]_OFF` as per-unit, and `RC<n>_*` not at all
  (one transmitter model fleet-wide). Selftest passes; drone 20 diffs clean except 03b.
- **New staged file `04_rc_only_fd_roll_off.params`** (`COM_RC_IN_MODE=0`, `FD_FAIL_R=0`),
  registered in `tools/param_diff.py`; `00_baseline.params` regenerated without those two rows
  (922 parameters). Selftest passes.
- **Patched firmware vendored and verified**: `px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4`
  (sha256 `fb23d654…e8d9c9`) is PX4 v1.16.2 plus only the NuttX stm32h7 UART TX DMA fix
  (apache/nuttx PR #17203). Flash it to every FC; the TELEM2 transmitter then survives agent
  loss and the uXRCE client reconnects on its own. Verified 2026-10-21 on drone 16: three
  back-to-back stack tests passed with no FC power cycle (`CLAUDE.md` bug #15). Build script, patch and notes in
  `px4/firmware/txdma-fix/`. The fleet flasher must compare the git hash to tell it from stock.

- **PX4 v1.16.2 TELEM2 TX DMA wedge** ([`CLAUDE.md`](CLAUDE.md) bug #15, found 2026-10-20 on
  drone 16): when the agent stops answering, the FC's `uxrce_dds_client` disconnects and
  reopens `/dev/ttyS3`, and a NuttX STM32H7 UART TX DMA bug then leaves the transmitter dead
  until the FC is power-cycled or rebooted. Fixed upstream in NuttX (apache/nuttx PR #17203,
  shipped in PX4 v1.17.0), not in any v1.16 release. Documented in `CLAUDE.md`,
  `docs/debugging.md`, `docs/fleet_guide.md` and the README. No image change.
- `host/doctor.sh`: the `uart-traffic` FAIL on 0 bytes now names this cause first (link worked
  earlier in this FC power cycle, stack since stopped: power-cycle the FC), then the connector,
  FC power and TX/RX swap. Check name and FAIL result unchanged.
- Fleet dashboard (skybeta-dashboard): the FC hint on `fc_no_topics` / `fc_no_data` says to
  power-cycle the FC if the stack was restarted since it booted.

## 0.3.0 — no geofence (2026-10-16, branch `fleet-ready`)

### You must do this when moving a drone from 0.2.0

1. **Rebuild or pull the image**: `skybeta:0.3.0` (`src/` changed). `fleetctl update` does both
   the code and the image.
2. **Team code that reads `/skybeta/state`**: `geofence_ok` is now `envelope_ok`, and
   `envelope.geofence_radius_m` is gone. A launch file that still passes
   `geofence_radius_m` to the flight interface is ignored, not an error.

### Changed

- **The horizontal geofence is removed from `skybeta_flight_interface`.** Position setpoints
  are no longer pulled back to a 10 m radius about the odometry origin; x and y reach PX4
  as written. The altitude band (`min_altitude_m` 0.3 / `max_altitude_m` 3.0) and the speed
  cap (`max_speed_mps` 1.5) are unchanged, and so is the FC's own vertical limit
  (`GF_MAX_VER_DIST` in each fixes file). Staying inside the flying area is now the team's
  code and the safety pilot's job.
- `/skybeta/state`: `geofence_ok` renamed `envelope_ok` (false only above the ceiling);
  `clamped` can no longer contain `geofence_radius`.

## 0.2.0 — fleet-ready (2026-10-12, branch `fleet-ready`)

Makes the stack safe to run on many drones sharing one WiFi network, with teams writing code
on the drone itself.

### You must do this when moving a drone from 0.1.0

1. **Write `/etc/skybeta/drone.env`** from `host/drone.env.example`: a unique `DRONE_ID` and
   `ROS_DOMAIN_ID` (convention: the drone number), the camera mount pose, the team.
2. **On the flight controller**: load `px4/params/02_telem2_uxrce_dds.params` again (it now
   sets **`UXRCE_DDS_PTCFG = 2`**), set **`UXRCE_DDS_DOM_ID`** to the drone's `ROS_DOMAIN_ID`
   by hand, and reboot the FC. With `PTCFG=0` the stack connects but **no `/fmu` data
   arrives** in the default local DDS scope.
3. **Rebuild or pull the image**: `skybeta:0.2.0`. The container scripts changed.
4. **Move team code** out of any container into `~/team_ws/src`.

### Changed

- **DDS isolation.** Each drone has its own DDS domain, and by default
  (`SKYBETA_DDS_SCOPE=local`) its DDS never leaves the Jetson: the ROS nodes use
  `host/dds/fastdds_localhost.xml`, the agent builds PX4's participant from
  `host/dds/agent_refs_local.xml` (`MicroXRCEAgent -r`, FC `UXRCE_DDS_PTCFG=2`).
  `SKYBETA_DDS_SCOPE=network` opens a drone up for laptop debugging. Bench: 0 DDS packets on
  WiFi with the full stack running; cuVSLAM still at 30 Hz.
- **Per-drone identity** in `/etc/skybeta/drone.env`, read by `host/run.sh` and
  `host/doctor.sh`. Parsed, never sourced.
- **Team workspace.** `$TEAM_WS` (default `~/team_ws`) is mounted at `/root/team_ws` in every
  container and survives restarts and image updates. `team_build.sh` builds it as an overlay;
  the stack's `competitor` window builds and launches it. Extra apt/pip packages:
  `host/run.sh build --init`, then `host/run.sh build` (image `skybeta-team:<VERSION>`).
- **The flight pipeline never loads team code**: agent, cuVSLAM and the bridge source the
  skybeta underlay only, so a team `px4_msgs` cannot break the state estimate.
- `host/run.sh shell` attaches to the running stack instead of refusing.
- `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` now actually reach the container (they never did).
- `host/doctor.sh`: two new checks, `drone-identity` and `team-ws` (twelve in total).
- Image distribution, verified against a real registry (`docs/distribution.md`):
  `push_image.sh` now tags the commit the image was **built from** (it used the current
  checkout's), `build_image.sh` stamps it as the `io.skybeta.git-sha` label, and
  `pull_image.sh` no longer aborts when `nvcr.io` is unreachable (the base layers come from
  your registry instead). The image source label now names `Artash-N/skybeta`.
- `host/doctor.sh --json`: one JSON object instead of the table (schema 1: `checks[]` of
  `{name, result, detail}` plus `pass`/`warn`/`fail`/`ok`), same checks and exit status.
  The fleet status agent reads it. Tested by `host/tests/test_doctor_json.sh`.
- `UXRCE_DDS_DOM_ID` is per-airframe: removed from the 02 file, excluded by `param_diff`.
- pip packages pinned to what 0.1.0 shipped (`depthai==3.10.0`, `MAVProxy==1.8.74`,
  `pymavlink==2.4.49`, `numpy<2`); an unpinned rebuild pulled numpy 2 and opencv 5.
- The reference airframe's parameter dump is committed
  (`px4/params/fleet/SN0001-reference-2026-05-07.parm`), so `param_diff --selftest` runs all
  six steps anywhere.
- `.gitattributes` forces LF, so scripts from a Windows checkout run on a Jetson.

### Not yet verified

- `/fmu` data through the 0.2.0 stack on a real FC with `UXRCE_DDS_PTCFG=2` (the bench
  TELEM2 connector is damaged and dropped the link first).
- Two drones on one access point.

## 0.1.0 (2026-09-03, branch `competition-base`)

PX4 v1.16.2 competition base: pinned firmware and staged parameters, fleet provisioning
(`host/provision.sh`, `doctor.sh`, `run.sh`), the competitor API (`skybeta_flight_interface`,
examples), the sim profile. See the commit message of `00567ef` and `CLAUDE.md`.
