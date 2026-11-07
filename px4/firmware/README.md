# PX4 firmware — pinned for the fleet

One file, vendored on purpose. Ten drones running three different PX4 builds is
the kind of fleet problem that only shows up at the venue.

| | |
|---|---|
| File | `px4_fmu-v6c_default-v1.16.2.px4` |
| Board | Pixhawk 6C (`board_id` 56, `PX4FMUv6C`) |
| PX4 version | **v1.16.2**, `git_identity` `v1.16.2` |
| Released | 2026-04-22 |
| Size | 1 845 192 bytes (1.8 MB) |
| SHA-256 | `f23a3ed4334ff14c9bcc927ce378a55a62fb6d871cb1ef05813c080e04c5776f` |
| Source | <https://github.com/PX4/PX4-Autopilot/releases/download/v1.16.2/px4_fmu-v6c_default.px4> |

The upstream filename is `px4_fmu-v6c_default.px4`; the version is appended here
so a directory listing tells you what you are holding.

## Verify before flashing

```bash
sha256sum -c px4/firmware/px4_fmu-v6c_default-v1.16.2.px4.sha256
```

The published SHA-256 is not a claim of ours — GitHub reports the same digest for
the release asset (`GET /repos/PX4/PX4-Autopilot/releases/tags/v1.16.2`, the
asset's `digest` field), and the two were compared when this file was vendored.

You can also read the identity straight out of the bundle without flashing it. A
`.px4` file is JSON:

```bash
python3 -c "
import json; d=json.load(open('px4/firmware/px4_fmu-v6c_default-v1.16.2.px4'))
print(d['summary'], d['git_identity'], 'board_id', d['board_id'])"
# PX4FMUv6C v1.16.2 board_id 56
```

## Patched build: v1.16.2 + the NuttX TX DMA fix (bug #15)

| | |
|---|---|
| File | `px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` |
| What it is | PX4 **v1.16.2** (`54f0455f`) with exactly one change: NuttX submodule bumped by the cherry-pick of PX4/NuttX `d50ffd354495ce5d30814dba41d2eb70a32b8fa3` (apache/nuttx PR #17203 "arch/arm/stm32h7: Fix UART TX DMA getting stuck", in PX4 main via PR #25791, shipped in v1.17.0) |
| `git_identity` / `git_hash` | `v1.16.2-1-g7a39c59` / `7a39c5954139c917d21e0c5cdf6cc10449b1c011` |
| Board | `board_id` 56, `PX4FMUv6C`; image 1 959 884 of 1 966 080 bytes (72 bytes more than stock) |
| SHA-256 | `fb23d654a5c213e80231451c848f519194bc505ffecb35d41d9e13f957e8d9c9` |
| Built | 2026-10-20, natively with Arm GNU Embedded 9-2020-q2-update (gcc 9.3.1 20200408, the same compiler as the stock release), cmake 3.16.3, ninja 1.10.0; see `txdma-fix/BUILD_NOTES.md` |

Why: on stock v1.16.2 the FC's TELEM2 transmitter dies whenever the uXRCE-DDS agent stops
answering (stack stop, Jetson reboot, agent crash) and stays dead until the FC is power-cycled
(`CLAUDE.md` bug #15). This build is the fix. Every FC in the fleet should move to it.

What the FC reports afterwards: `ver all` still says 1.16.2 but with git-hash `7a39c595...`;
MAVLink `AUTOPILOT_VERSION.flight_sw_version` carries the **dev** type byte (`0x01100200`
instead of `0x011002FF`) because the identity has a `-1-g<sha>` suffix, and
`flight_custom_version` carries `7a39c59541`. QGC shows it as a 1.16.2 development build.
The fleet flasher (`pixhawk_autoparams.py`) must therefore compare the git hash, not only the
version, or it treats a stock v1.16.2 board as already up to date.

Reproduce: `txdma-fix/build_fmu-v6c_v1.16.2-txdma.sh` (docker mode via PX4's
`px4io/px4-dev-nuttx-focal:2022-08-12`, or `NATIVE=1` with no sudo). The resulting bytes
differ run to run only by the embedded build date/time; `txdma-fix/BUILD_NOTES.md` records
the exact commands, toolchain and checks. The cherry-picked change is
`txdma-fix/nuttx-stm32h7-uart-txdma-stuck-d50ffd3.patch`.

```bash
sha256sum -c px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4.sha256
```

## Why v1.16.2 and not v1.14

The repo used to claim v1.14 throughout. It was wrong. The reference airframe's
own parameter dump contains `EKF2_RNG_FOG`, which does not exist in the v1.14 or
v1.15 parameter reference and first appears in v1.16. Everything downstream was
re-pinned to match: `px4_msgs` and `px4-ros2-interface-lib` are both
`release/1.16` in `docker/Dockerfile.jetson`.

v1.16.2 is the latest 1.16.x stable at the time of writing, and it is the version
`sim/` targets for SITL, so the simulator and the vehicle run the same firmware
generation. A sim on different firmware from the vehicle is a sim that lies about
parameters.

## Flashing

QGroundControl → Vehicle Setup → Firmware → *Advanced settings* → **Custom
firmware file** → select this file.

**Flashing erases every parameter.** That is the whole reason
`px4/params/00_baseline.params` exists. Flash, then load the baseline, then
calibrate. The ordered procedure is [`docs/fleet_guide.md`](../../docs/fleet_guide.md).

## The parameter metadata is in here too

The bundle carries the full v1.16.2 parameter reference as a zlib+base64
`parameter_xml` blob — 2065 definitions with their types. `tools/param_diff.py`
reads it, which is why the generated baseline's `INT32`/`REAL32` column is what
the firmware declares rather than something inferred from decimal points.
