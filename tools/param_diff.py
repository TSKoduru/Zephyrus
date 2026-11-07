#!/usr/bin/env python3
"""Compare a flight controller's parameters against the skybeta fleet baseline.

Ten drones are only a fleet if they are configured identically. This tool answers
one question: **what on this vehicle differs from what we think we put there?**

The expected configuration is built by overlaying, in order:

    px4/params/00_baseline.params          the fleet-common airframe config
    px4/params/01_telem2_mavlink.params    \\
    px4/params/02_telem2_uxrce_dds.params   |  the staged bring-up files,
    px4/params/03a_height_reference.params  |  later files winning
    px4/params/03b_ekf2_vision.params      /

The actual configuration comes from either a live FC or a saved dump:

    tools/param_diff.py --master /dev/ttyTHS1,57600
    tools/param_diff.py --dump px4/params/fleet/SN0001.params
    tools/param_diff.py --dump px4/params/fleet/SN0001-reference-2026-05-07.parm

Anything a drone carries that the baseline does not mention -- calibration,
RC endpoints, flight counters -- is deliberately *not* reported unless you ask
for it with ``--all``. Those are per-unit by construction (see the header of
``00_baseline.params``); listing them every run would bury the drift you care
about under 200 lines of noise.

Exit status is 0 only when nothing expected has changed or gone missing, so
this drops straight into a pre-flight script.

Two side modes:

    tools/param_diff.py --selftest          no hardware; checks the repo's own files
    tools/param_diff.py --make-baseline px4/params/fleet/SN0001-reference-2026-05-07.parm

``--make-baseline`` is how ``00_baseline.params`` was produced, kept here so the
derivation is reproducible rather than a one-off. It reads a MAVProxy ``.parm``
or QGC ``.params`` dump of a known-good reference airframe and strips everything
per-unit.

File formats, both accepted on input and auto-detected per line:

    QGC .params    <vehicle-id>\\t<component-id>\\t<NAME>\\t<VALUE>\\t<TYPE>
                   TYPE 6 = INT32, 9 = REAL32 (the MAV_PARAM_TYPE enum)
    MAVProxy .parm <NAME> <VALUE>
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import sys
import zlib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PARAM_DIR = REPO / "px4" / "params"
BASELINE = PARAM_DIR / "00_baseline.params"
STAGED = [
    PARAM_DIR / "01_telem2_mavlink.params",
    PARAM_DIR / "02_telem2_uxrce_dds.params",
    PARAM_DIR / "03a_height_reference.params",
    PARAM_DIR / "03b_ekf2_vision.params",
    PARAM_DIR / "04_rc_only_fd_roll_off.params",
    PARAM_DIR / "05_rc_switches_board_rot.params",
    PARAM_DIR / "06_rc_radiomaster.params",
    PARAM_DIR / "07_airframe_geometry.params",
    PARAM_DIR / "08_no_optical_flow.params",
    PARAM_DIR / "09_autonomy_takeover.params",
]
FIRMWARE_DIR = REPO / "px4" / "firmware"
# The MAVProxy dump of the reference airframe that 00_baseline.params was
# generated from (1129 params, captured 2026-05-07). Committed so --selftest
# runs its dump checks anywhere, not only on the machine that holds the file.
REFERENCE_DUMP = PARAM_DIR / "fleet" / "SN0001-reference-2026-05-07.parm"

MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9

# Floats arrive as 6-decimal text from MAVProxy and as IEEE-754 singles from
# MAVLink, so an exact compare is meaningless. This is loose enough to absorb
# both and tight enough that a real edit never hides inside it.
ABS_TOL = 1e-6
REL_TOL = 1e-6


# --------------------------------------------------------------------------
# per-unit exclusions -- the rules that turn a vehicle dump into a baseline
# --------------------------------------------------------------------------
# Each entry is (predicate, reason). The reason is written into the generated
# file's header, so the exclusion list and its justification cannot drift apart.
# RC<n>_(MIN|MAX|TRIM|DZ|REV) used to be excluded as per-transmitter calibration. Since
# 2026-11-01 the fleet flies one transmitter model, so 06_rc_radiomaster.params pins them
# fleet-wide and they are plain staged values.

EXCLUSIONS = [
    (lambda n: n.startswith("CAL_"),
     "accelerometer / gyro / magnetometer / level-horizon calibration -- "
     "physically per-board, and QGC rewrites them during step 2"),
    (lambda n: n in ("SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "SENS_BOARD_Z_OFF"),
     "level-horizon trim -- written by the same QGC calibration step as CAL_*. "
     "SENS_BOARD_ROT (the mount orientation) is fleet-wide and lives in 05"),
    (lambda n: n == "SENS_DPRES_OFF",
     "differential-pressure zero offset -- per-sensor calibration"),
    (lambda n: n in ("SYS_CAL_ACCEL", "SYS_CAL_BARO", "SYS_CAL_GYRO"),
     "one-shot 'calibrate on next boot' triggers, not configuration"),
    (lambda n: n == "MAV_SYS_ID",
     "MAVLink system id must be UNIQUE per airframe on a shared GCS link"),
    (lambda n: n == "UXRCE_DDS_DOM_ID",
     "DDS domain must be UNIQUE per airframe on a shared network; equals "
     "ROS_DOMAIN_ID in that drone's /etc/skybeta/drone.env"),
    (lambda n: n in ("COM_FLIGHT_UUID", "LND_FLIGHT_T_HI", "LND_FLIGHT_T_LO"),
     "cumulative flight counters -- runtime state, not configuration"),
    (lambda n: n.startswith("_"),
     "MAVProxy pseudo-parameters (e.g. _HASH_CHECK), not real PX4 parameters"),
]


def is_per_unit(name: str) -> str | None:
    """Return the exclusion reason for *name*, or None if it is fleet-common."""
    for pred, reason in EXCLUSIONS:
        if pred(name):
            return reason
    return None


# --------------------------------------------------------------------------
# parsing
# --------------------------------------------------------------------------
class ParamFile:
    """A parsed parameter set: name -> (value, mav_param_type or None)."""

    def __init__(self, path: Path, params: dict[str, tuple[float, int | None]]):
        self.path = path
        self.params = params

    def __len__(self) -> int:
        return len(self.params)


def _coerce(text: str, ptype: int | None) -> float:
    if ptype == MAV_PARAM_TYPE_INT32:
        return float(int(float(text)))
    return float(text)


def parse_params(path: os.PathLike | str) -> ParamFile:
    """Parse a QGC .params or MAVProxy .parm file. Format detected per line."""
    path = Path(path).expanduser()
    out: dict[str, tuple[float, int | None]] = {}
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) == 5 and fields[0].isdigit() and fields[1].isdigit():
                # QGC: vehicle-id, component-id, name, value, type
                name, value, ptype = fields[2], fields[3], int(fields[4])
            elif len(fields) == 2:
                # MAVProxy: name, value. Type inferred from the literal --
                # MAVProxy writes %d for INT32 and %f for REAL32, which was
                # cross-checked against the v1.16.2 firmware's own parameter
                # metadata for all 1128 real parameters in the reference dump:
                # zero disagreements.
                name, value = fields
                ptype = MAV_PARAM_TYPE_REAL32 if "." in value else MAV_PARAM_TYPE_INT32
            else:
                raise ValueError(f"{path}:{lineno}: cannot parse {line!r}")
            try:
                out[name] = (_coerce(value, ptype), ptype)
            except ValueError as exc:
                raise ValueError(f"{path}:{lineno}: bad value {value!r}") from exc
    return ParamFile(path, out)


def firmware_param_types(firmware: Path) -> dict[str, int]:
    """Read authoritative parameter types out of a .px4 firmware bundle.

    A .px4 file is JSON with a zlib+base64 ``parameter_xml`` blob. Using it means
    the generated baseline carries the types the firmware itself declares rather
    than types guessed from decimal points.
    """
    doc = json.loads(Path(firmware).read_text())
    blob = doc.get("parameter_xml")
    if not blob:
        return {}
    xml = zlib.decompress(base64.b64decode(blob)).decode("utf-8", "replace")
    types: dict[str, int] = {}
    for name, ptype in re.findall(
        r'<parameter\s+name="([A-Z0-9_]+)"\s+default="[^"]*"\s+type="(\w+)"', xml
    ):
        types[name] = (
            MAV_PARAM_TYPE_INT32 if ptype == "INT32" else MAV_PARAM_TYPE_REAL32
        )
    if not types:  # attribute order differs between PX4 releases -- retry loosely
        for tag in re.findall(r"<parameter\b[^>]*>", xml):
            n = re.search(r'name="([A-Z0-9_]+)"', tag)
            t = re.search(r'type="(\w+)"', tag)
            if n and t:
                types[n.group(1)] = (
                    MAV_PARAM_TYPE_INT32
                    if t.group(1) == "INT32"
                    else MAV_PARAM_TYPE_REAL32
                )
    return types


def find_firmware() -> Path | None:
    if not FIRMWARE_DIR.is_dir():
        return None
    candidates = sorted(FIRMWARE_DIR.glob("*.px4"))
    return candidates[0] if candidates else None


# --------------------------------------------------------------------------
# expected set
# --------------------------------------------------------------------------
def build_expected(
    baseline: Path | None, staged: list[Path]
) -> tuple[dict[str, tuple[float, int | None]], dict[str, Path], list[ParamFile]]:
    """Overlay baseline + staged files. Later files win. Also record provenance."""
    merged: dict[str, tuple[float, int | None]] = {}
    origin: dict[str, Path] = {}
    loaded: list[ParamFile] = []
    for path in ([baseline] if baseline else []) + staged:
        if path is None:
            continue
        pf = parse_params(path)
        loaded.append(pf)
        for name, val in pf.params.items():
            merged[name] = val
            origin[name] = path
    return merged, origin, loaded


def values_equal(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL)


def fmt(value: float, ptype: int | None) -> str:
    if ptype == MAV_PARAM_TYPE_INT32 or (ptype is None and float(value).is_integer()):
        return str(int(round(value)))
    return f"{value:.6f}"


# --------------------------------------------------------------------------
# live read
# --------------------------------------------------------------------------
def fetch_live(master: str, timeout: float = 60.0) -> dict[str, tuple[float, int | None]]:
    """Download every parameter from a live FC over MAVLink."""
    try:
        from pymavlink import mavutil
    except ImportError:  # pragma: no cover -- environment-dependent
        raise SystemExit(
            "pymavlink is not installed. Either `pip3 install pymavlink`, or use\n"
            "  --dump <file>  against a QGC / MAVProxy parameter dump instead."
        )
    import time

    dev, _, baud = master.partition(",")
    conn = mavutil.mavlink_connection(dev, baud=int(baud or 57600))
    print(f"[param_diff] waiting for HEARTBEAT on {dev} @ {baud or 57600} ...",
          file=sys.stderr)
    hb = conn.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise SystemExit(
            "no HEARTBEAT. Before blaming configuration, reseat the TELEM2 JST-GH\n"
            "connector -- a negative UART result on a suspect cable proves nothing\n"
            "(docs/hardware_checklist.md)."
        )
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)

    params: dict[str, tuple[float, int | None]] = {}
    expected_count: int | None = None
    seen_index: set[int] = set()
    deadline = time.time() + timeout
    last_rx = time.time()
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            if expected_count and len(params) >= expected_count:
                break
            if time.time() - last_rx > 5.0:
                # stalled: re-request the indices we never saw
                if expected_count is None:
                    break
                missing = [i for i in range(expected_count) if i not in seen_index]
                if not missing:
                    break
                for idx in missing[:40]:
                    conn.mav.param_request_read_send(
                        conn.target_system, conn.target_component, b"", idx
                    )
                last_rx = time.time()
            continue
        last_rx = time.time()
        expected_count = msg.param_count
        seen_index.add(msg.param_index)
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        name = name.rstrip("\x00")
        ptype = (
            MAV_PARAM_TYPE_INT32
            if msg.param_type in (1, 2, 3, 4, 5, 6, 7, 8)
            else MAV_PARAM_TYPE_REAL32
        )
        params[name] = (float(msg.param_value), ptype)
        if expected_count and len(params) >= expected_count:
            break
    if expected_count and len(params) < expected_count:
        print(
            f"[param_diff] WARNING: got {len(params)} of {expected_count} parameters."
            " Treat MISSING entries as unknown, not as drift.",
            file=sys.stderr,
        )
    return params


# --------------------------------------------------------------------------
# baseline generation
# --------------------------------------------------------------------------
BASELINE_HEADER = """\
# Onboard parameters for Vehicle 1
#
# 00_baseline.params -- the fleet-common configuration for every skybeta drone.
#
# Load this SECOND, immediately after flashing PX4 v1.16.2 (which erases all
# parameters), and BEFORE the per-unit calibration in QGC. Then load the staged
# files 01 / 02 / 03a / 03b in order as the bring-up in docs/fleet_guide.md
# reaches each one, and 04 (RC only, roll failure detector off), 05 (arm switch ch 7,
# kill switch ch 5, board roll 180), 06 (RadioMaster sticks + calibration) and 07
# (airframe geometry) on every FC.
# Reboot the FC after loading.
#
#   QGC -> Vehicle Setup -> Parameters -> Tools -> Load from file
#
# DERIVATION
#
# Generated from a MAVProxy parameter dump of the reference airframe
# ({source}, {source_count} parameters, captured {source_note}) by
#
#   tools/param_diff.py --make-baseline <dump>
#
# Parameter types are the ones the firmware itself declares, read out of
# {firmware_note}
#
# WHAT IS *NOT* IN HERE, AND WHY
#
# A baseline that carried per-unit values would clone one airframe's
# calibration onto nine others. Accelerometer bias, magnetometer offsets and
# level-horizon trim are properties of a specific board in a specific
# orientation; copying them produces a vehicle that believes it is level when
# it is not, and drifts on take-off. Each drone must be calibrated on its own.
#
{exclusions}#
# Also excluded: every parameter set by 01/02/03a/03b/04/05/06/07, so that the staged files
# remain the single place those values are defined and this file never
# contradicts them.
#
# CARRIED OVER DELIBERATELY, BUT WORTH KNOWING
#
#   SYS_AUTOSTART = 4001          Generic Quadcopter -- change it if your
#                                 airframe is not one.
#   EKF2_OF_CTRL, SENS_EN_PMW3901, SENS_TFLOW_CFG
#                                 the reference dump has optical flow on, but NO
#                                 skybeta airframe has a flow sensor: these are
#                                 owned by 08_no_optical_flow.params (all off).
#   EKF2_RNG_POS_Y = -0.027432    measured on the reference airframe. Re-measure
#                                 if your rangefinder sits elsewhere.
#   BAT1_V_DIV / BAT1_A_PER_V     the Pixhawk 6C power-module defaults. If a
#                                 drone reports the wrong pack voltage, that
#                                 drone needs its own value.
#
# Vehicle-Id\tComponent-Id\tName\tValue\tType
"""


def make_baseline(dump: Path, firmware: Path | None) -> str:
    src = parse_params(dump)
    staged_names: set[str] = set()
    for path in STAGED:
        if path.exists():
            staged_names |= set(parse_params(path).params)

    types = firmware_param_types(firmware) if firmware else {}
    firmware_note = (
        f"{firmware.name} (its embedded parameter_xml)."
        if types
        else "the dump's own numeric formatting (no firmware bundle available)."
    )

    reasons: dict[str, list[str]] = {}
    rows: list[tuple[str, float, int]] = []
    for name in sorted(src.params):
        value, ptype = src.params[name]
        reason = is_per_unit(name)
        if reason:
            reasons.setdefault(reason, []).append(name)
            continue
        if name in staged_names:
            continue
        rows.append((name, value, types.get(name, ptype or MAV_PARAM_TYPE_REAL32)))

    excl_lines = []
    for reason, names in reasons.items():
        excl_lines.append(f"#   {len(names):>4} x  {reason}\n")
        excl_lines.append(f"#          e.g. {', '.join(sorted(names)[:4])}\n")

    header = BASELINE_HEADER.format(
        source=Path(dump).name,
        source_count=len(src),
        source_note="2026-05-07",
        firmware_note=firmware_note,
        exclusions="".join(excl_lines),
    )
    body = "".join(
        f"1\t1\t{name}\t{fmt(value, ptype)}\t{ptype}\n" for name, value, ptype in rows
    )
    return header + body


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def diff(expected, origin, actual, show_all=False):
    changed, missing, extra = [], [], []
    for name, (want, wtype) in sorted(expected.items()):
        if name not in actual:
            missing.append(name)
            continue
        got = actual[name][0]
        if not values_equal(want, got):
            changed.append((name, want, wtype, got))
    if show_all:
        for name in sorted(actual):
            if name not in expected:
                extra.append(name)
    return changed, missing, extra


def report(expected, origin, actual, loaded, actual_label, show_all):
    for pf in loaded:
        try:
            rel = pf.path.relative_to(REPO)
        except ValueError:
            rel = pf.path
        print(f"  expected  {str(rel):<44} {len(pf):>5} params")
    print(f"  actual    {actual_label:<44} {len(actual):>5} params")
    print(f"  merged expected set: {len(expected)} params\n")

    changed, missing, extra = diff(expected, origin, actual, show_all)

    if changed:
        print(f"CHANGED ({len(changed)})  -- the vehicle disagrees with the baseline")
        print(f"  {'PARAMETER':<20} {'EXPECTED':>14} {'ACTUAL':>14}   defined in")
        for name, want, wtype, got in changed:
            src = origin[name].name
            print(f"  {name:<20} {fmt(want, wtype):>14} {fmt(got, wtype):>14}   {src}")
        print()
    if missing:
        print(f"MISSING ({len(missing)})  -- expected, but the vehicle never reported it")
        print("  A truncated download looks identical to a parameter that does not")
        print("  exist in this firmware. Re-read before treating these as drift.")
        for name in missing:
            print(f"  {name:<20} {'':>14} {'':>14}   {origin[name].name}")
        print()
    if extra:
        print(f"EXTRA ({len(extra)})  -- on the vehicle, outside the baseline")
        print("  Expected: calibration, RC endpoints, flight counters. See the")
        print("  header of 00_baseline.params for why these are not tracked.")
        for name in extra:
            print(f"  {name:<20} {fmt(*actual[name]):>14}")
        print()
    if not changed and not missing:
        print("OK -- no drift. Every expected parameter matches.")
    return 0 if (not changed and not missing) else 1


# --------------------------------------------------------------------------
# selftest
# --------------------------------------------------------------------------
def selftest() -> int:
    failures: list[str] = []

    def check(ok: bool, label: str, detail: str = "") -> None:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  {detail}" if detail else ""))
        if not ok:
            failures.append(label)

    print("param_diff selftest -- no hardware required\n")

    print("1. staged files parse and are well-formed QGC .params")
    for path in STAGED:
        if not path.exists():
            check(False, f"{path.name} exists")
            continue
        bad = []
        for lineno, raw in enumerate(path.read_text().splitlines(), 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            fields = raw.rstrip("\n").split("\t")
            if len(fields) != 5:
                bad.append(f"line {lineno}: {len(fields)} tab-separated fields")
            elif fields[4] not in ("6", "9"):
                bad.append(f"line {lineno}: type {fields[4]!r} is not 6 or 9")
        pf = parse_params(path)
        check(not bad, f"{path.name}", f"{len(pf)} params" if not bad else bad[0])

    print("\n2. baseline is well-formed and carries nothing per-unit")
    if not BASELINE.exists():
        check(False, "00_baseline.params exists")
    else:
        bad, cal, badtype = [], [], []
        for lineno, raw in enumerate(BASELINE.read_text().splitlines(), 1):
            if raw.startswith("#") or not raw.strip():
                continue
            fields = raw.split("\t")
            if len(fields) != 5:
                bad.append(f"line {lineno}: {len(fields)} fields")
                continue
            if fields[4] not in ("6", "9"):
                badtype.append(f"line {lineno}: type {fields[4]!r}")
            if is_per_unit(fields[2]):
                cal.append(fields[2])
        base = parse_params(BASELINE)
        check(not bad, "every row has 5 tab-separated fields",
              bad[0] if bad else f"{len(base)} rows")
        check(not badtype, "every type is 6 (INT32) or 9 (REAL32)",
              badtype[0] if badtype else "")
        check(not cal, "no per-unit parameters (CAL_*, SENS_BOARD_*_OFF, MAV_SYS_ID, ...)",
              ", ".join(cal[:5]) if cal else "")
        staged_names: set[str] = set()
        for path in STAGED:
            if path.exists():
                staged_names |= set(parse_params(path).params)
        overlap = sorted(set(base.params) & staged_names)
        check(not overlap, "baseline does not restate anything the staged files set",
              ", ".join(overlap[:5]) if overlap else f"{len(staged_names)} staged names")

    print("\n3. reference dump parses in MAVProxy format")
    dump = REFERENCE_DUMP
    if not dump.exists():
        print(f"  [SKIP] {dump} not present -- steps 3-5 skipped")
        return 1 if failures else 0
    ref = parse_params(dump)
    check(len(ref) > 1000, str(dump), f"{len(ref)} params")

    print("\n4. baseline round-trips: regenerating it from the dump reproduces it")
    if BASELINE.exists():
        regenerated = make_baseline(dump, find_firmware())
        on_disk = BASELINE.read_text()
        same_body = [l for l in regenerated.splitlines() if not l.startswith("#")] == \
                    [l for l in on_disk.splitlines() if not l.startswith("#")]
        check(same_body, "--make-baseline output matches the committed file",
              "" if same_body else "regenerate with --make-baseline")

    print("\n5. diffing the reference dump against baseline+staged")
    expected, origin, loaded = build_expected(
        BASELINE if BASELINE.exists() else None, [p for p in STAGED if p.exists()]
    )
    changed, missing, _ = diff(expected, origin, ref.params, show_all=False)
    staged_names = set()
    for path in STAGED:
        if path.exists():
            staged_names |= set(parse_params(path).params)
    unexplained_changed = [c for c in changed if c[0] not in staged_names]
    unexplained_missing = [m for m in missing if m not in staged_names]
    check(
        not unexplained_changed,
        "every non-staged baseline value matches the dump it came from",
        ", ".join(c[0] for c in unexplained_changed[:5]),
    )
    check(
        not unexplained_missing,
        "nothing expected is absent from the dump except staged parameters",
        ", ".join(unexplained_missing[:5]),
    )
    print(f"  [INFO] {len(changed)} staged parameters differ from the May-07 dump "
          f"(expected: that is the point of the staged files)")
    print(f"  [INFO] {len(missing)} staged parameters absent from the dump "
          f"({', '.join(missing) if missing else 'none'})")

    print("\n6. firmware bundle parses and yields parameter metadata")
    fw = find_firmware()
    if fw is None:
        print("  [SKIP] no px4/firmware/*.px4 vendored")
    else:
        types = firmware_param_types(fw)
        check(len(types) > 1500, f"{fw.name}", f"{len(types)} parameter definitions")
        mismatched = [
            n for n, (v, t) in ref.params.items()
            if n in types and types[n] != t
        ]
        check(not mismatched,
              "dump's inferred types agree with the firmware's own metadata",
              ", ".join(mismatched[:5]))

    print()
    if failures:
        print(f"SELFTEST FAILED: {len(failures)} check(s)")
        return 1
    print("SELFTEST PASSED")
    return 0


# --------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See docs/fleet_guide.md step 8.",
    )
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--master", metavar="DEV,BAUD",
                     help="read a live FC, e.g. /dev/ttyTHS1,57600")
    src.add_argument("--dump", metavar="FILE",
                     help="read a saved .parm / .params dump")
    ap.add_argument("--baseline", default=str(BASELINE),
                    help="baseline file (default: px4/params/00_baseline.params)")
    ap.add_argument("--staged", nargs="*", default=None,
                    help="staged files, in load order (default: 01, 02, 03a, 03b)")
    ap.add_argument("--no-staged", action="store_true",
                    help="compare against the baseline alone")
    ap.add_argument("--all", action="store_true",
                    help="also list parameters present on the vehicle but absent "
                         "from the baseline (calibration, RC endpoints, counters)")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="seconds to spend downloading from a live FC")
    ap.add_argument("--selftest", action="store_true",
                    help="run offline checks against the repo's own files")
    ap.add_argument("--make-baseline", metavar="DUMP",
                    help="regenerate 00_baseline.params from a reference dump")
    ap.add_argument("--output", metavar="FILE",
                    help="write --make-baseline output here instead of stdout")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    if args.make_baseline:
        text = make_baseline(Path(args.make_baseline), find_firmware())
        if args.output:
            Path(args.output).write_text(text)
            n = sum(1 for l in text.splitlines() if not l.startswith("#"))
            print(f"[param_diff] wrote {args.output}: {n} parameters", file=sys.stderr)
        else:
            sys.stdout.write(text)
        return 0

    if not args.master and not args.dump:
        ap.error("one of --master, --dump, --selftest or --make-baseline is required")

    baseline = Path(args.baseline).expanduser()
    if not baseline.exists():
        raise SystemExit(f"baseline not found: {baseline}")
    if args.no_staged:
        staged = []
    elif args.staged is None:
        staged = [p for p in STAGED if p.exists()]
    else:
        staged = [Path(p).expanduser() for p in args.staged]

    expected, origin, loaded = build_expected(baseline, staged)

    if args.dump:
        pf = parse_params(args.dump)
        actual, label = pf.params, str(Path(args.dump).expanduser())
    else:
        actual, label = fetch_live(args.master, args.timeout), args.master

    print()
    return report(expected, origin, actual, loaded, label, args.all)


if __name__ == "__main__":
    sys.exit(main())
