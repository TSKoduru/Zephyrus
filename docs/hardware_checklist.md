# Hardware checklist

Physical checks, each paired with **the failure it prevents**. That pairing is the point:
every item on this list exists because skipping it once produced a symptom that looked
like a software bug and consumed real debugging time.

Two passes:

- **Build-time** — once per airframe, during assembly. §1–§4.
- **Pre-flight** — every single flight, no exceptions. §5.

---

## 1. TELEM2 ↔ Jetson serial link

### 1.1 The connector — read this before anything else

> **The single most expensive item on this list.** A damaged or partly-seated TELEM2
> JST-GH connector produces *intermittent* silence: hours of nothing, then one flawless
> run, then nothing again. It is indistinguishable from a configuration fault, and it
> cost an entire debugging session on the reference airframe before the cable was
> identified.

The Pixhawk 6C's TELEM2 port is a **JST-GH 1.25 mm 6-pin**. The pins are small, the
latch is small, and a housing that has been levered out sideways once will make
contact when you push on it and lose contact when you let go.

**Check:**

- [ ] Housing clicks home and the latch engages. Tug the cable gently along its axis —
      it must not move.
- [ ] All six crimps visible in the housing, none backed out.
- [ ] No cracked housing, no splayed latch, no bent pins on the board header.
- [ ] Cable is strain-relieved to the frame so vibration and prop wash do not work it
      loose in flight.

**Prevents:** intermittent or total loss of the EV datalink. On the ground it looks like
a parameter problem; in flight, if the link drops while `EKF2_EV_CTRL` is fusing vision,
EKF2 loses position aiding and the estimator degrades — which presents as exactly the
drift you would otherwise blame on `pose_frame` or `EKF2_EV_DELAY`.

**The debugging rule that follows from this.** Any *negative* UART result recorded with a
suspect cable in place is uninformative — it tells you nothing about configuration.
Positive results are still valid. Replace the cable, then re-run everything that
previously "failed". **Never bodge it with tape or glue on a flight vehicle.**

### 1.2 Wiring

Three wires. **VCC is not one of them.**

| Pixhawk 6C TELEM2 (JST-GH 6-pin) | → | Jetson 40-pin header | Signal |
|---|---|---|---|
| pin 1 — **VCC (+5 V)** | ✗ | **not connected** | see below |
| pin 2 — **TX** | → | **pin 10** (`UART1_RX`) | FC transmits, Jetson receives |
| pin 3 — **RX** | ← | **pin 8** (`UART1_TX`) | Jetson transmits, FC receives |
| pin 4 — CTS | ✗ | not connected | no flow control |
| pin 5 — RTS | ✗ | not connected | no flow control |
| pin 6 — **GND** | → | **pin 6** (GND) | common reference |

- [ ] TX→RX and RX→TX, i.e. **crossed**. Straight-through is the classic mistake and
      gives perfect silence with no error anywhere.
- [ ] GND connected. Without it you get framing errors and garbage rather than silence.
- [ ] **VCC left unconnected.** Both boards are separately powered; joining the 5 V rails
      back-feeds one from the other. `MAV_1_FLOW_CTRL = 0` in
      [`../px4/params/01_telem2_mavlink.params`](../px4/params/01_telem2_mavlink.params)
      matches the unwired CTS/RTS.

Jetson pins 8/10 are `serial@3100000` → `/dev/ttyTHS1`.

**Verify (Jetson side alone, before the FC is even involved):**

```bash
# Jumper pin 8 to pin 10, nothing else connected:
python3 host/uart_loopback_test.py
```

Loopback passing means the Jetson's TX/RX path and the DMA/IOMMU device-tree patch are
good, so any remaining silence is on the cable or the FC. Loopback failing means stop —
nothing about the flight controller matters yet.

**Verify (end to end, after §1.1 and the FC has `01_telem2_mavlink.params` loaded):**

```bash
mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors
```

Expected: a HEARTBEAT within a second or two. 57600 rather than 921600 deliberately —
more tolerant of marginal wiring while you are still chasing a dead link.

**Prevents:** every UART symptom in this repo's history.

---

## 2. OAK-D-S2

### 2.1 It must be on the USB **3.0** hub

- [ ] Plugged into a **USB 3.0** port (blue, or the USB3 hub on `bus 2`), with a
      USB 3-rated cable. A USB 2 cable in a USB 3 port negotiates USB 2.

**Verify — and do not be fooled by `lsusb` before the camera boots:**

The MyriadX enumerates **twice**.

| Stage | ID | Speed | Bus |
|---|---|---|---|
| Unbooted (power-on, no firmware) | `03e7:2485` | **480 Mbps** | bus 1 |
| Booted (depthai has pushed firmware) | `03e7:f63b` | **5000 Mbps** | `2-1.1` (bus 2) |

Seeing `03e7:2485` at 480 Mbps is **normal and expected** on a healthy USB 3.0 link. It
is not evidence of anything. `host/doctor.sh` reports it as a pass for this reason.

To actually measure the link, boot the device:

```bash
host/doctor.sh --boot-camera
# or, in the container:
python3 -c "
import depthai as dai
with dai.Device() as d: print(d.getUsbSpeed())"     # expect UsbSpeed.SUPER
```

**Prevents:** hours of bandwidth reasoning built on the pre-boot reading. Every
throughput conclusion drawn from the 480 Mbps figure is wrong.

Related, and worth knowing so you do not chase it: the frame-rate ceiling on this camera
is **not** USB. The MyriadX's XLink transport tops out near 120–140 MB/s regardless of
link rate, which is why the stack runs **1024×640 @ 30 Hz** and not the sensor's native
1280×800 (which measures 23.3 Hz with mismatched left/right frames). See
[`debugging.md`](debugging.md) §3.

### 2.2 Camera identity

Read it off the device rather than trusting the label:

```bash
host/run.sh shell        # then, inside:
python3 -c "
import depthai as dai
with dai.Device() as d:
    for f in d.getConnectedCameraFeatures(): print(f)"
```

Expected on an OAK-D-S2: `CAM_A = IMX378` 4056×3040 colour; `CAM_B`/`CAM_C` = **`OV9282`
1280×800 monochrome**; IMU **BNO086** (units built Q2 2021–Q2 2023 carry a 6-axis BMI270
instead, which changes what IMU fusion can do).

- [ ] Stereo pair reports `OV9282`, not OV9782.
- [ ] IMU part recorded in the fleet table.

**Prevents:** IMU-fusion plans that assume a 9-axis part on a drone that has a 6-axis one.

### 2.3 Mounting and the mount pose

- [ ] Camera rigidly mounted — no foam, no zip-tie flex. A stereo rig that moves relative
      to itself invalidates the baseline cuVSLAM reads from TF.
- [ ] Lenses clean, no protective film left on.
- [ ] Field of view clear of props, legs and the battery. Rotor blur in frame is a
      texture source that moves with the vehicle.
- [ ] Cable routed so it cannot foul a prop, with a service loop at the camera end.

**Measure the mount pose and write it down.** Translation from the **Pixhawk's primary
IMU** (that is where PX4 puts the body origin — not the CoG, not the frame centre, not
the GPS antenna) to the **left** stereo lens focal point, in FLU body axes: +x forward
along the Pixhawk's mounting arrow, +y left, +z up. Then the rotation of the camera in
that same frame.

Put them in `/etc/skybeta/drone.env`, which is what the boot-time service reads:

```bash
CAMERA_X=0.10  CAMERA_Y=0.00  CAMERA_Z=-0.02        # metres
CAMERA_ROLL=0.0  CAMERA_PITCH=0.35  CAMERA_YAW=0.0  # radians
```

- [ ] Translation measured, to ~5 mm.
- [ ] **Rotation measured.** A down-pitched VIO camera is the norm, and 0.35 rad ≈ 20°
      down is a typical value. Leaving it at zero when the camera is tilted is a real
      error, not a rounding one.
- [ ] `EKF2_EV_POS_X/Y/Z` on the flight controller are **all zero**.

**Prevents:** two distinct failures. An unmodelled mount *rotation* couples vehicle
rotation into apparent translation, which orbits — this is toilet-bowl suspect #4. And
setting `EKF2_EV_POS_*` while the static TF also carries the lever arm applies the offset
**twice**; pick exactly one, and this stack has picked the TF.

---

## 3. Jetson

- [ ] Powered from a supply that holds up under load. An Orin Nano browning out mid-flight
      takes the whole navigation stack with it.
- [ ] Adequate airflow over the heatsink. Thermal throttling shows up as frame-rate
      instability that looks like a camera problem.
- [ ] microSD / NVMe has room for bags. `record_bag.sh` writes both camera streams.
- [ ] `host/doctor.sh` exits 0.

`doctor.sh` covers, without needing sudo: L4T release · UARTA `iommus`/`dmas` in the live
device tree · `3100000.serial` bound to `serial-tegra` owning `ttyTHS1` · exactly one
`FDT` line in `extlinux.conf` · `dialout` + `docker` membership · docker `nvidia` runtime
· OAK-D enumerated · UART traffic · image present · power model · drone identity (`drone.env`, DDS domain) · team workspace.

Two results that are expected on a bench unit and are **not** faults: `uart-traffic` FAILs
when the FC is unpowered or the cable is the suspect one (§1.1), and `oakd-usb` reporting
`03e7:2485` at 480 Mbps is the unbooted stage (§2.1).

---

## 4. RC and the safety pilot's controls

There is **always a safety pilot on RC**. The flight interface has no arm path. The pilot arms and
switches to Position, the interface then enters Offboard, and the pilot can take the vehicle back at
any instant with the sticks or the mode switch. Nothing in the software substitutes for these checks.

- [ ] Receiver **bound** to this vehicle's transmitter, and the transmitter's model
      memory is the right one. Ten identical airframes make this an easy mistake.
- [ ] Radio calibration done **on this airframe** (QGC → Radio). It is per
      transmitter+receiver pair and is deliberately excluded from
      [`00_baseline.params`](../px4/params/00_baseline.params).
- [ ] Mode switch assigned, and the pilot knows which position is Position.
      `RC_MAP_FLTMODE` is channel 6: Manual, Altitude, Position. There is no Offboard position.
- [ ] Take-over practised with props off: throttle up from low, then the mode switch to Altitude.
- [ ] **Kill switch assigned and physically tested** — motors off, then arm, confirm the
      switch disarms, then disarm. `RC_MAP_KILL_SW` on the reference airframe is
      channel 7. Test it before every session, not once at build time.
- [ ] Failsafe behaviour understood: `COM_RCL_EXCEPT`, `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`.
      Know what the vehicle does when your node stops publishing, before it does.
- [ ] Range check at the flying site.

**Prevents:** the class of failure where the only remaining option is to wait and watch.

---

## 5. Pre-flight — every flight

Ordered so a failure stops you before the props spin.

1. [ ] **Props off** for the first run of any new code. Bench it.
2. [ ] TELEM2 connector reseated and tugged (§1.1). Thirty seconds. It is the cheapest
       item here and the one that has actually bitten.
3. [ ] OAK-D lenses clean, view unobstructed, cable clear of the props.
4. [ ] Battery charged, secured, and the correct pack for the tuned parameters.
5. [ ] `host/doctor.sh` → exits 0.
6. [ ] `host/run.sh stack` up; on a ground-station shell:
       - [ ] `/skybeta/state` shows `px4_link: true`
       - [ ] `/visual_slam/tracking/odometry` publishing, and `odom_enu` moves sensibly
             when you carry the vehicle
       - [ ] your node is publishing `/skybeta/setpoint/pose` **before** the pilot arms
7. [ ] `record_bag.sh` (or `host/run.sh bag`) started. Record every flight — it is the
       only way to explain a bad one afterwards, and it costs nothing on a good one.
8. [ ] Kill switch tested this session (§4).
9. [ ] Safety pilot has the transmitter, is standing clear, and has said so out loud.
10. [ ] Pilot arms in Altitude, throttle low, then switches to Position within a few seconds. Team
        code takes off from here, so be ready to raise the throttle to take over (competitor guide §4).

---

## 6. Per-airframe record

Fill one of these per drone and keep it with the fleet table in
[`fleet_guide.md`](fleet_guide.md).

| Field | Value |
|---|---|
| Airframe serial | |
| Pixhawk 6C serial | |
| FC firmware | `v1.16.2` (sha256 `f23a3ed4…`) |
| Jetson hostname / serial | |
| OAK-D serial (`getMxId()`) | |
| OAK-D IMU part | BNO086 / BMI270 |
| Camera mount x, y, z (m, FLU) | |
| Camera mount roll, pitch, yaw (rad) | |
| TELEM2 cable replaced on | |
| RC receiver bound to TX | |
| Kill switch channel | |
| Radio calibration date | |
| Sensor calibration date | |
| Image tag | |
| Last `param_diff` clean on | |
