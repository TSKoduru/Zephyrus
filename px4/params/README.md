# PX4 parameter sets

Staged QGroundControl `.params` files for the skybeta stack. Values verified against the
**PX4 v1.16** parameter reference — the firmware this airframe actually runs (see `EKF2_RNG_FOG`
in the May dump; it does not exist in v1.14 or v1.15).

## Back up first

```
QGC -> Vehicle Setup -> Parameters -> Tools -> Save to file
```

Do this before loading anything. These are **delta files** — they contain only the parameters
being changed, so everything else on the vehicle is left alone. Your calibration,
`SYS_AUTOSTART`, and the measured `EKF2_RNG_POS_Y = -0.027432` are not touched. A backup means
you can get back regardless.

## Load them in order, one at a time, rebooting between

| File | Purpose | Load when | Needs the UART? |
|---|---|---|---|
| `03a_height_reference.params` | height: range finder reference, baro backup (PX4 "Indoor (non-flat terrain)") | **now** — fixes the runaway climb | no |
| `01_telem2_mavlink.params` | TELEM2 → MAVLink | first — proves the UART wiring | — |
| `02_telem2_uxrce_dds.params` | TELEM2 → uXRCE-DDS | after stage 1 gives a HEARTBEAT | yes |
| `03b_ekf2_vision.params` | EV fusion (`EKF2_EV_CTRL=9`, delay measured per airframe) | **per drone**, once its stack shows `/fmu/in/vehicle_visual_odometry` at ~30 Hz (the `pose_frame` fix is done). **Not** in the flasher's default set. Loaded on **drone 16** (2026-11-03, `EKF2_EV_DELAY=60`) | yes |
| `04_rc_only_fd_roll_off.params` | `COM_RC_IN_MODE=0` (RC transmitter only, no joystick), `FD_FAIL_R=0` (roll failure detector off) | **now**, on every FC — the fleet flasher loads it by default | no |
| `05_rc_switches_board_rot.params` | `RC_MAP_ARM_SW=7`, `RC_MAP_KILL_SW=5`, `SENS_BOARD_ROT=8` (FMU roll 180°, upside down) | **now**, on every FC, before level horizon — the fleet flasher loads it by default | no |
| `06_rc_radiomaster.params` | AETR stick order + RC1..RC18 endpoints/trims/deadzones of the fleet RadioMaster (from drone 20) | **now**, on every FC — the fleet flasher loads it by default; replaces QGC Radio calibration for drones on that transmitter | no |
| `07_airframe_geometry.params` | `CA_ROTOR<n>_PX/PY` rotor positions, `EKF2_RNG_POS_Z` (from drone 20) | **now**, on every FC — the fleet flasher loads it by default | no |
| `08_no_optical_flow.params` | `EKF2_OF_CTRL=0`, `SENS_EN_PMW3901=0`, `SENS_TFLOW_CFG=0`: **no skybeta airframe has an optical-flow sensor** — the only ground sensor is one downward ~1° laser rangefinder | **now**, on every FC — the fleet flasher loads it by default | no |

**`03a` is deliberately separable.** The height-reference fix has nothing to do with vision, the
UART or the container — it addresses the upward drift on its own and can go in today. Everything
else waits on a working link. Do not load `03b` early: `EKF2_EV_CTRL = 9` is inert while no
vision data arrives, but it means you would later bring up the link and enable vision fusion in
the same step, against a bridge that still declares `POSE_FRAME_NED` — which is the toilet-bowl
bug. Fix the bridge first, then load `03b`. *(2026-11-03: the bridge fix is done -- it sends
`POSE_FRAME_FRD` -- and 03b is loaded on drone 16; see CLAUDE.md "Drone 16".)*

`QGC -> Parameters -> Tools -> Load from file`, then **reboot the FC** — every parameter in
these files is reboot-gated.

### After stage 1

```bash
mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors
```

A HEARTBEAT within a second or two means the wire is good. Nothing means the fault is physical —
see the loopback test (`uart_loopback_test.py`) and the wiring table in `CLAUDE.md`. **Do not
move on to stage 2 until this works**; DDS on a dead wire looks identical to DDS misconfigured.

### After stage 2

```bash
run_agent.sh                       # in the container
ros2 topic list | grep /fmu/       # should list many topics
check_topics.sh                    # echoes /fmu/out/timesync_status
```

### After stage 3

Bench first, props off. Confirm `EKF2_EV_CTRL` is actually fusing before flying — watch
`/fmu/out/estimator_status_flags` and check the vehicle's local position tracks the camera when
you move the airframe by hand.

## Stage 4: RC only, roll failure detector off (added 2026-10-22)

`04_rc_only_fd_roll_off.params` carries two fleet-wide values that differ from the reference
airframe's May-07 dump, so they live in a staged file rather than in `00_baseline.params`
(which `--make-baseline` must still reproduce from that dump):

- **`COM_RC_IN_MODE = 0`**, RC Transmitter only. The May dump had **1 = Joystick only**, under
  which PX4 disables RC input handling and its checks, so every FC provisioned before
  2026-10-22 ignored the safety pilot's transmitter. Re-run the flasher (or load this file)
  on those FCs. With 0, the RC-loss failsafe is live and QGC's Radio calibration per drone
  (fleet guide step 2b) is what makes the sticks right.
- **`FD_FAIL_R = 0`** disables the failure detector's roll check (default 60 deg for 0.3 s,
  which locks the motors down on takeoff and would terminate the flight if `CBRK_FLIGHTTERM`
  were 0). Roll is not a failure criterion on these airframes. `FD_FAIL_P` stays at 60.

## Stage 5: arm switch ch 7, kill switch ch 5, board roll 180 (added 2026-11-01)

`05_rc_switches_board_rot.params` carries three more fleet-wide values that differ from the
May-07 dump, kept out of `00_baseline.params` for the same reason as 04:

- **`RC_MAP_ARM_SW = 7`** and **`RC_MAP_KILL_SW = 5`**. The dump had them the other way round
  (arm 5, kill 7). PX4 disables stick arming once an arm switch is mapped; `MAN_ARM_GESTURE`
  is already 0 in the baseline. Flight mode stays on channel 6. The switch thresholds
  (`RC_ARMSWITCH_TH`, `RC_KILLSWITCH_TH` = 0.75) are unchanged.
- **`SENS_BOARD_ROT = 8`**, PX4's "Roll 180°": the Pixhawk 6C hangs upside down on these
  airframes, the value QGC's Autopilot Orientation was set to on drone 20 (a board the right way
  up but facing the tail would be 4, Yaw 180°; the two are not interchangeable). Reboot-gated. Only the level-horizon trim
  (`SENS_BOARD_X/Y/Z_OFF`) remains per-unit; `param_diff` and the flasher used to treat the
  whole `SENS_BOARD_` family as per-unit and now exclude just those three. **Run level horizon
  after loading 05**, with the rotation in place, otherwise the trim is measured in the wrong
  frame and must be redone.

FCs flashed before 2026-11-01 (drones 2, 3, 5, 6, 7, 8, 9, 11, 16, 18, 19, 21; drone 20 was
configured by hand and is the source) carry the swapped switches and no rotation: re-run the
flasher on each, then redo level horizon.

## Stage 6: RadioMaster sticks and channel calibration (added 2026-11-01)

`06_rc_radiomaster.params` is QGC's Radio calibration of the fleet RadioMaster, read off drone
20 and made fleet-wide because every drone flies the same transmitter model with the same model
setup: `RC_MAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4` (AETR; the May dump was TAER 4/3/2/1, which
scrambles the sticks on a RadioMaster) plus `RC1..RC18` `_MIN/_TRIM/_MAX/_DZ/_REV`. Until now
the `RC<n>_*` family was treated as per-unit by `param_diff` and the flasher; it no longer is, so
the baseline carries no RC rows and this file is the one place the radio is defined. A drone on
a different transmitter needs its own stage-6 file.

## Stage 7: airframe geometry (added 2026-11-01)

`07_airframe_geometry.params` carries the rotor positions `CA_ROTOR0..3_PX/PY` (front arms
x = +0.08, rear x = −0.15, y = ±0.16 front / ±0.14 rear) and the range-finder height
`EKF2_RNG_POS_Z = 0.03445`, both entered by hand on drone 20. The May dump, and so the baseline
until now, had PX4's generic ±1 m placeholders, i.e. the control allocator believed the frame
was a 2 m square.

## Stage 8: no optical flow (added 2026-11-03)

`08_no_optical_flow.params` sets `EKF2_OF_CTRL=0`, `SENS_EN_PMW3901=0` and `SENS_TFLOW_CFG=0`.
**No skybeta airframe has an optical-flow sensor.** The only ground-facing sensor is one
downward laser rangefinder with a narrow (~1°) beam, driven by PX4's VL53L1X driver
(`SENS_EN_VL53L1X=1`, kept; its `DISTANCE_SENSOR` reports laser, downward, 0–4 m). The May-07
dump's flow configuration belonged to an older sensor set: harmless with nothing attached, but it
made every reader believe the airframe had a flow fallback for Position mode. It does not —
horizontal position comes from the Jetson's VIO and nothing else. Reboot the FC after loading.
Once loaded, the `SENS_FLOW_*` parameters disappear from the FC (their driver is off), so
`param_diff --dump` lists them as MISSING; that is expected.

## Recorded dumps: `fleet/`

One file per airframe, as it is actually running (fleet guide step 8), diffed with
`tools/param_diff.py --dump fleet/<serial>.params`.

- **`fleet/SN0001-reference-2026-05-07.parm`** — MAVProxy dump of the reference airframe,
  1129 parameters, captured 2026-05-07, before any staged file was loaded. `00_baseline.params`
  is generated from it (`tools/param_diff.py --make-baseline`), and `--selftest` uses it to
  prove the baseline round-trips. It predates `UXRCE_DDS_PTCFG` (absent) and was taken with
  TELEM2 still on MAVLink. Do not overwrite it; record new dumps alongside it.
- **`fleet/SN0001-2026-10-15.params`** — QGC dump of the same airframe as it actually ran on
  2026-10-15: **firmware v1.16.1** (git `94cb2012` = tag `v1.16.1`), not the v1.16.2 the fleet
  pins. `param_diff --dump` reports 28 changed parameters; the ones that matter:
  `UXRCE_DDS_DOM_ID=0` and `UXRCE_DDS_PTCFG=0` (no `/fmu` data reaches the stack),
  `EKF2_HGT_REF=0` with `EKF2_BARO_CTRL=0` and `EKF2_GPS_CTRL=0` (no height aiding once the
  range finder drops out), `EKF2_EV_CTRL=9` with the placeholder `EKF2_EV_DELAY=50` and EV
  gates widened to 50, `NAV_RCL_ACT=6` (Disarm on RC loss), and motor functions on both
  `PWM_MAIN_FUNC1-4` and `PWM_AUX_FUNC1-4`.
- **`fleet/SN0001-fixes-2026-10-15.params`** — the delta that airframe should load (QGC "Load
  from file", then reboot): the DDS pair for drone 1, PX4's "Indoor (flat terrain)" height
  configuration ("Indoor (non-flat terrain)": range finder as the reference, conditional, baro
  kept as backup), vision fusion kept on with the measured `EKF2_EV_DELAY=70` and the EV gates
  back at 5 / 3, and a 3 m ceiling inside the VL53L1X's 4 m range. Its
  header explains each value and lists what it deliberately leaves for a human decision.
- **`fleet/drone20-radiomaster-2026-11-01.params`** — drone 20, configured by hand in QGC; the
  source of 05, 06 and 07.
- **`fleet/drone16-vio-2026-11-03.params`** — drone 16 after the fleet default set + 08 + **03b**
  with `EKF2_EV_DELAY=60` (measured on that airframe). The VIO-position-hold-by-default drone
  (CLAUDE.md "Drone 16"). `param_diff --dump` against baseline+staged: only `EKF2_EV_DELAY`
  (60 vs 03b's 70) changed; `SENS_FLOW_*` and `MAV_1_*` missing (drivers/instance off).

## Deliberately not included

- **`EKF2_RNG_CTRL`.** Retiring the rangefinder as the height reference is a separate decision
  (03a). Optical flow is not a decision at all: there is **no flow sensor on any airframe**, and
  `08_no_optical_flow.params` turns off the flow configuration the May-07 dump carried. With VIO
  the **only** horizontal position source, losing tracking leaves Position mode with no
  horizontal aiding -- PX4 falls back to Altitude behaviour.
- **Vertical position and velocity EV fusion.** `EKF2_EV_CTRL = 9` is horizontal position + yaw
  only. Add bit 1 (→ 11) for vertical, bit 2 (→ 13) for velocity, once vision is trusted —
  vertical is the weakest axis for pure stereo VO with `enable_imu_fusion` off.

## Values that are placeholders, not answers

- **`EKF2_EV_DELAY = 70`** — measured on the bench Orin, not yet on a flying airframe
  (2026-10-16, `tools/latency_probe.py`: image → bridge publish p50 68.7 ms, p95 102 ms, plus ~1–2 ms
  for the serial hop). It replaced a guessed 50. Re-run the probe on each airframe and cross-check
  with an EKF2 replay log (`SDLOG_PROFILE` bit 7); a wrong delay is a prime toilet-bowl cause.
- **`UXRCE_DDS_DOM_ID`** — no longer in `02_telem2_uxrce_dds.params`: it is **per airframe**.
  Set it by hand to that drone's `ROS_DOMAIN_ID` from `/etc/skybeta/drone.env` (convention:
  `= DRONE_ID`). On a shared network, every drone left on 0 shares one DDS domain.
  `param_diff` treats it like `MAV_SYS_ID`: excluded from the fleet comparison.
