#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_NAME=$(basename "$0")
WORKDIR_DEFAULT="/var/tmp/jetson_uart_dma_iommu_universal"
PATCHED_DTB="/boot/dtb/uart_dma_fix.dtb"
EXTLINUX="/boot/extlinux/extlinux.conf"
BACKUP_EXTLINUX="/boot/extlinux/extlinux.conf.jetson_uart_dma_iommu_universal.bak"
ACTIVE_DTB_BACKUP="/boot/dtb/uart_dma_fix.preexisting.bak"
# The patch is applied to the board's STOCK kernel DTB, never to the live tree.
# UEFI trims the DTB it boots for each chip's fused-off units (which 6 of the 12
# A78 cores exist differs per Orin Nano chip). A patched DTB built from the live
# tree bakes in the trim of the board it was made on; a golden-image clone with
# a different core set then boots with only 4 cores and nvpmodel fails
# (found 2026-10-20 on drone 16, cloned from 15). Override with BASE_DTB=<file>.
DTB_DIR="/boot/dtb"
SID_GPCDMA_HEX="0x04"
UART_NODE="serial@3100000"

log() { printf '[%s] %s\n' "$SCRIPT_NAME" "$*"; }
warn() { printf '[%s] WARNING: %s\n' "$SCRIPT_NAME" "$*" >&2; }
die() { printf '[%s] ERROR: %s\n' "$SCRIPT_NAME" "$*" >&2; exit 1; }
need_root() { [[ ${EUID:-$(id -u)} -eq 0 ]] || die "Run as root: sudo $SCRIPT_NAME ..."; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"; }

usage() {
  cat <<USAGE
Usage:
  sudo $SCRIPT_NAME apply [--reboot] [--force]
       $SCRIPT_NAME apply --dry-run            # no root; shows the extlinux diff
  sudo $SCRIPT_NAME status
       $SCRIPT_NAME current                    # exit 0 if $PATCHED_DTB is built from the stock DTB
  sudo $SCRIPT_NAME restore [--reboot]
       $SCRIPT_NAME rewrite-extlinux <file> <dtb-path>   # helper, used by tests

Purpose:
  Patch Jetson Orin / tegra234 UARTA serial@3100000 for the R36.5 UART DMA/IOMMU issue.

What apply does:
  - Decompiles the board's stock kernel DTB ($DTB_DIR/kernel_tegra234-*.dtb, or BASE_DTB=...).
    Not the live tree: UEFI trims the booted DTB per chip (CPU cores and other
    fused-off units), so a DTB built from the live tree only fits the board it
    was made on -- a golden-image clone would boot with 4 of its 6 cores.
  - Locates serial@3100000
  - Ensures the node has:
      dma-names = "rx", "tx";
      dmas = <...>;
      iommus = <smmu_niso0 0x04>;
  - Compiles /boot/dtb/uart_dma_fix.dtb
  - Points /boot/extlinux/extlinux.conf at that DTB
  - Verifies the staged DTB before success

Idempotency:
  - apply is a no-op (exit 0) when the patch is already live: the running device
    tree carries serial@3100000/iommus, $PATCHED_DTB exists and was built from
    the stock DTB (same /cpus node count), and extlinux.conf has exactly one FDT
    line, pointing at it. Use --force to rewrite anyway.
  - A $PATCHED_DTB built from the live tree by older versions of this script
    (fewer /cpus nodes than the stock DTB) is rebuilt; reboot afterwards.
  - If the patch is live but extlinux.conf has duplicate FDT lines (left by old
    versions of this script), apply rewrites extlinux.conf only -- backup first,
    DTB untouched, no reboot needed.
  - The extlinux rewrite deletes every FDT line inside "LABEL primary" and then
    inserts exactly one immediately after LINUX, so repeated runs cannot stack
    duplicate FDT entries.

Restore:
  - Restores extlinux.conf from backup.
USAGE
}

ensure_tools() {
  need_cmd dtc
  need_cmd python3
  need_cmd grep
  need_cmd sed
  need_cmd awk
  need_cmd cp
  need_cmd mkdir
}

read_nv_tegra_release() {
  [[ -f /etc/nv_tegra_release ]] && cat /etc/nv_tegra_release || true
}

assert_supported_platform() {
  local rel model compat
  rel=$(read_nv_tegra_release || true)
  model=$(tr -d '\0' < /sys/firmware/devicetree/base/model 2>/dev/null || true)
  compat=$(tr -d '\0' < /sys/firmware/devicetree/base/compatible 2>/dev/null || true)

  [[ -n "$rel" ]] || die "This does not look like a Jetson with /etc/nv_tegra_release present."
  [[ "$rel" == *"R36"* ]] || warn "Unexpected L4T release. Current release: ${rel//$'\n'/ }"
  [[ "$rel" == *"REVISION: 5."* || "$rel" == *"R36.5"* ]] || warn "This fix is mainly for R36.5.x. Current release: ${rel//$'\n'/ }"
  [[ "$model" == *"Orin"* || "$compat" == *"tegra234"* ]] || die "Unsupported platform. This script targets Orin / tegra234-class boards. Model='$model' compatible='$compat'"
}

make_workdir() {
  WORKDIR=${WORKDIR:-$WORKDIR_DEFAULT}
  # The default lives under /var/tmp and is created root-owned by the first
  # `sudo apply`. Unprivileged callers (--dry-run, the unit test) then cannot
  # write into it, so fall back to a per-user directory rather than dying.
  if ! mkdir -p "$WORKDIR" 2>/dev/null || [[ ! -w "$WORKDIR" ]]; then
    WORKDIR="${TMPDIR:-/tmp}/jetson_uart_dma_iommu_universal-$(id -un)"
    mkdir -p "$WORKDIR" || die "Cannot create a writable work directory ($WORKDIR)"
  fi
}

dump_live_dts() {
  ACTIVE_DTS="$WORKDIR/active.dts"
  dtc -I fs -O dts -o "$ACTIVE_DTS" /sys/firmware/devicetree/base >/dev/null 2>&1 || \
    die "Failed to dump live device tree"
}

# The stock kernel DTB this board boots without the patch. Exactly one
# kernel_tegra234-*.dtb ships in $DTB_DIR on a JetPack install; BASE_DTB overrides.
stock_dtb() {
  if [[ -n "${BASE_DTB:-}" ]]; then
    [[ -f "$BASE_DTB" ]] || die "BASE_DTB=$BASE_DTB does not exist"
    echo "$BASE_DTB"; return 0
  fi
  local found=()
  shopt -s nullglob
  found=("$DTB_DIR"/kernel_tegra234-*.dtb)
  shopt -u nullglob
  [[ ${#found[@]} -eq 1 ]] || \
    die "Expected exactly one stock $DTB_DIR/kernel_tegra234-*.dtb, found ${#found[@]}: ${found[*]:-none}. Pass BASE_DTB=<file>."
  echo "${found[0]}"
}

# Number of cpu@ nodes under /cpus in a DTB file (fdtget if present, else dtc).
dtb_cpu_count() {
  if command -v fdtget >/dev/null 2>&1; then
    fdtget -l "$1" /cpus 2>/dev/null | grep -c '^cpu@' || true
  else
    dtc -I dtb -O dts "$1" 2>/dev/null | grep -cE '^[[:space:]]*cpu@[0-9a-f]+ \{' || true
  fi
}

# True when $PATCHED_DTB was built from the full stock DTB rather than a live
# (per-chip trimmed) tree. Readable without root.
patched_dtb_is_current() {
  [[ -f "$PATCHED_DTB" ]] || return 1
  local stock n_stock n_patched
  stock="$(stock_dtb 2>/dev/null)" || return 1
  n_stock="$(dtb_cpu_count "$stock")"
  n_patched="$(dtb_cpu_count "$PATCHED_DTB")"
  [[ -n "$n_stock" && "$n_stock" -gt 0 && "$n_patched" == "$n_stock" ]]
}

dump_stock_dts() {
  ACTIVE_DTS="$WORKDIR/active.dts"
  STOCK_DTB="$(stock_dtb)"
  [[ "$(readlink -f "$STOCK_DTB")" != "$(readlink -f "$PATCHED_DTB")" ]] || \
    die "BASE_DTB points at $PATCHED_DTB itself; use the stock kernel DTB"
  log "base DTB: $STOCK_DTB ($(dtb_cpu_count "$STOCK_DTB") cpu nodes; UEFI trims them per chip at boot)"
  dtc -I dtb -O dts -o "$ACTIVE_DTS" "$STOCK_DTB" >/dev/null 2>&1 || \
    die "Failed to decompile $STOCK_DTB"
}

python_helper() { python3 - "$@"; }

summarize_dts_to_json() {
  local dts_file="$1"
  python_helper "$dts_file" <<'PY'
import json, re, sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(errors="replace")

def find_block(text, node_name):
    m = re.search(r'(^|\n)(?P<indent>\s*)' + re.escape(node_name) + r'\s*\{', text)
    if not m:
        return None
    start = m.start()
    brace_start = m.end() - 1
    depth = 0
    i = brace_start
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                semi = text.find(";", i)
                return start, semi + 1, text[start:semi + 1]
        i += 1
    return None

def get_smmu_info(text):
    sym = find_block(text, "__symbols__")
    smmu_path = None
    if sym:
        _, _, symtxt = sym
        m = re.search(r'\bsmmu_niso0\s*=\s*"([^"]+)"\s*;', symtxt)
        if m:
            smmu_path = m.group(1)

    phandle = None
    candidates = []
    if smmu_path:
        candidates.append(smmu_path.rstrip("/").split("/")[-1])
    candidates += ["iommu@12000000", "smmu@12000000"]

    for node_name in candidates:
        for m in re.finditer(r'(^|\n)\s*' + re.escape(node_name) + r'\s*\{', text):
            block = find_block(text[m.start():], node_name)
            if not block:
                continue
            _, _, node = block
            pm = re.search(r'\bphandle\s*=\s*<\s*(0x[0-9a-fA-F]+|\d+)\s*>\s*;', node)
            if pm:
                return smmu_path, pm.group(1)
    return smmu_path, phandle

block = find_block(text, "serial@3100000")
if not block:
    print(json.dumps({"found": False}))
    raise SystemExit(0)

_, _, node = block
smmu_path, smmu_phandle = get_smmu_info(text)

def prop(pattern):
    m = re.search(pattern, node, flags=re.S)
    return m.group(1).strip() if m else None

state = {
    "found": True,
    "has_dmas": bool(re.search(r'\bdmas\s*=\s*[^;]+;', node, flags=re.S)),
    "has_dma_names": bool(re.search(r'\bdma-names\s*=\s*[^;]+;', node, flags=re.S)),
    "has_iommus": bool(re.search(r'\biommus\s*=\s*<[^;]+>\s*;', node, flags=re.S)),
    "dmas_value": prop(r'\bdmas\s*=\s*([^;]+);'),
    "dma_names_value": prop(r'\bdma-names\s*=\s*([^;]+);'),
    "iommus_value": prop(r'\biommus\s*=\s*(<[^;]+>);'),
    "smmu_path": smmu_path,
    "smmu_phandle": smmu_phandle,
}
print(json.dumps(state))
PY
}

print_state_pretty() {
  python_helper "$1" <<'PY'
import json, sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    print(json.dumps(json.load(f), indent=2))
PY
}

patch_dts() {
  PATCHED_DTS="$WORKDIR/uart_dma_fix.dts"
  python_helper "$ACTIVE_DTS" "$PATCHED_DTS" "$SID_GPCDMA_HEX" <<'PY'
import re, sys
from pathlib import Path

src = Path(sys.argv[1]).read_text(errors="replace")
out = Path(sys.argv[2])
sid = sys.argv[3]
node_name = "serial@3100000"

def find_node_span(text, node_name):
    m = re.search(r'(^|\n)(?P<indent>\s*)' + re.escape(node_name) + r'\s*\{', text)
    if not m:
        raise SystemExit(f"{node_name} not found")
    start = m.start()
    open_brace = m.end() - 1
    indent = m.group("indent")
    depth = 0
    i = open_brace
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                semi = text.find(";", i)
                if semi < 0:
                    raise SystemExit("UART node closing semicolon not found")
                return start, semi + 1, text[start:semi + 1], indent
        i += 1
    raise SystemExit("UART node end not found")

def find_smmu_phandle(text):
    # Prefer __symbols__. Fall back to common Orin SMMU node.
    smmu_path = None
    try:
        _, _, sym, _ = find_node_span(text, "__symbols__")
        m = re.search(r'\bsmmu_niso0\s*=\s*"([^"]+)"\s*;', sym)
        if m:
            smmu_path = m.group(1)
    except SystemExit:
        pass

    candidates = []
    if smmu_path:
        candidates.append(smmu_path.rstrip("/").split("/")[-1])
    candidates += ["iommu@12000000", "smmu@12000000"]

    for cand in candidates:
        for m in re.finditer(r'(^|\n)\s*' + re.escape(cand) + r'\s*\{', text):
            # Parse from this node occurrence by manual local scan.
            start = m.start()
            brace = text.find("{", m.start())
            depth = 0
            i = brace
            while i < len(text):
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                    if depth == 0:
                        semi = text.find(";", i)
                        block = text[start:semi + 1]
                        pm = re.search(r'\bphandle\s*=\s*<\s*(0x[0-9a-fA-F]+|\d+)\s*>\s*;', block)
                        if pm:
                            return pm.group(1)
                        break
                i += 1
    raise SystemExit("Could not find SMMU phandle")

start, end, node, indent = find_node_span(src, node_name)
inner_indent = indent + "\t"

# Preserve existing DMA values if present; otherwise use UARTA/GPCDMA channel 8 defaults seen on affected systems.
m = re.search(r'\bdmas\s*=\s*([^;]+);', node, flags=re.S)
dmas_value = " ".join(m.group(1).split()) if m else "<0xed 0x08 0xed 0x08>"

m = re.search(r'\bdma-names\s*=\s*([^;]+);', node, flags=re.S)
dma_names_value = " ".join(m.group(1).split()) if m else '"rx", "tx"'

smmu_phandle = find_smmu_phandle(src)
iommus_value = f"<{smmu_phandle} {sid}>"

# Remove old versions of the properties from inside the node.
node2 = re.sub(r'\n[ \t]*dma-names\s*=\s*[^;]+;', '', node, flags=re.S)
node2 = re.sub(r'\n[ \t]*dmas\s*=\s*[^;]+;', '', node2, flags=re.S)
node2 = re.sub(r'\n[ \t]*iommus\s*=\s*[^;]+;', '', node2, flags=re.S)

# Insert before the final closing brace of THIS node, robust to tabs/spaces.
close_match = list(re.finditer(r'\n(?P<indent>[ \t]*)\};\s*$', node2, flags=re.S))
if not close_match:
    raise SystemExit("Could not locate final UART node closing brace")
cm = close_match[-1]
insert = (
    f"\n{inner_indent}dma-names = {dma_names_value};"
    f"\n{inner_indent}dmas = {dmas_value};"
    f"\n{inner_indent}iommus = {iommus_value};"
)
node2 = node2[:cm.start()] + insert + node2[cm.start():]

patched = src[:start] + node2 + src[end:]
out.write_text(patched)

print(f"Patched {node_name}")
print(f"  dma-names = {dma_names_value};")
print(f"  dmas = {dmas_value};")
print(f"  iommus = {iommus_value};")
PY
}

compile_and_install_dtb() {
  mkdir -p /boot/dtb
  COMPILED_DTB="$WORKDIR/uart_dma_fix.dtb"
  dtc -I dts -O dtb -o "$COMPILED_DTB" "$PATCHED_DTS" >/dev/null || die "Failed to compile patched DTS"

  [[ -f "$PATCHED_DTB" && ! -f "$ACTIVE_DTB_BACKUP" ]] && cp "$PATCHED_DTB" "$ACTIVE_DTB_BACKUP"
  cp "$COMPILED_DTB" "$PATCHED_DTB"
}

# Rewrite one extlinux.conf-shaped file in place so that "LABEL primary" holds
# exactly one FDT line, immediately after its LINUX line.
#
#   rewrite_extlinux_file <file> <dtb-path>
#
# Deliberately takes the file as an argument (rather than always touching
# $EXTLINUX) so the transformation can be unit-tested on a copy without root --
# see host/tests/test_extlinux_fix.sh and `apply --dry-run`.
rewrite_extlinux_file() {
  local target="$1" dtb="$2"
  python_helper "$target" "$dtb" <<'PY'
import re, sys
from pathlib import Path

extlinux = Path(sys.argv[1])
dtb_path = sys.argv[2]
lines = extlinux.read_text().splitlines()

out = []
in_primary = False
inserted = False

for line in lines:
    stripped = line.strip()

    # Comments are passed through untouched and never change block state --
    # extlinux.conf ships with a commented-out "# LABEL backup" stanza.
    if stripped.startswith("#"):
        out.append(line)
        continue

    if stripped.startswith("LABEL "):
        in_primary = stripped == "LABEL primary"

    # Pre-pass: drop EVERY pre-existing FDT inside LABEL primary. The old code
    # rewrote the first one it met *after* already inserting a new one after
    # LINUX, so each run grew the file by one duplicate line (bug #2).
    if in_primary and re.match(r'^\s*FDT\s+', line):
        continue

    out.append(line)

    if in_primary and re.match(r'^\s*LINUX\s+', line) and not inserted:
        out.append('      FDT ' + dtb_path)
        inserted = True

if not inserted:
    raise SystemExit("Could not insert FDT line in extlinux.conf: "
                     "no LINUX line found inside 'LABEL primary'")

extlinux.write_text("\n".join(out) + "\n")
print("extlinux FDT ->", dtb_path)
PY
}

update_extlinux_to_use_patched_dtb() {
  [[ -f "$EXTLINUX" ]] || die "Missing $EXTLINUX"
  [[ ! -f "$BACKUP_EXTLINUX" ]] && cp "$EXTLINUX" "$BACKUP_EXTLINUX"
  rewrite_extlinux_file "$EXTLINUX" "$PATCHED_DTB"
}

# True when the patch is already live on this boot. All three probes are
# readable without root, so `apply` can short-circuit before need_root.
already_applied() {
  [[ -e /proc/device-tree/bus@0/serial@3100000/iommus ]] || return 1
  [[ -f "$PATCHED_DTB" ]]                                || return 1
  patched_dtb_is_current                                 || return 1
  [[ -f "$EXTLINUX" ]]                                   || return 1
  grep -qE "^[[:space:]]*FDT[[:space:]]+${PATCHED_DTB}[[:space:]]*$" "$EXTLINUX" || return 1
  return 0
}

# Show what `apply` would do to extlinux.conf, without root and without
# touching /boot. Operates on a copy under $WORKDIR.
dry_run_extlinux() {
  make_workdir
  [[ -f "$EXTLINUX" ]] || die "Missing $EXTLINUX"
  local copy="$WORKDIR/extlinux.conf.dryrun"
  cp "$EXTLINUX" "$copy"
  rewrite_extlinux_file "$copy" "$PATCHED_DTB" >/dev/null
  echo "=== dry run: diff $EXTLINUX -> proposed ==="
  if diff -u "$EXTLINUX" "$copy" --label "a/$EXTLINUX" --label "b/$EXTLINUX"; then
    echo "(no change -- extlinux.conf is already correct)"
  fi
  echo
  echo "FDT lines inside LABEL primary after rewrite:"
  grep -cE '^[[:space:]]*FDT[[:space:]]' "$copy" | sed 's/^/  /'
  log "Dry run only. Nothing was written to $EXTLINUX or /boot."
}

verify_staged_dtb() {
  local staged_dts="$WORKDIR/staged.dts"
  local staged_json="$WORKDIR/staged_state.json"
  dtc -I dtb -O dts -o "$staged_dts" "$PATCHED_DTB" >/dev/null 2>&1 || die "Failed to decompile staged DTB"
  summarize_dts_to_json "$staged_dts" > "$staged_json"

  python_helper "$staged_json" <<'PY'
import json, sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    s = json.load(f)
missing = [k for k in ("has_dmas", "has_dma_names", "has_iommus") if not s.get(k)]
if not s.get("found"):
    raise SystemExit("Verification failed: serial@3100000 not found")
if missing:
    raise SystemExit("Verification failed: " + ", ".join(missing) + " false")
print("Verified staged DTB: serial@3100000 has dmas, dma-names, and iommus.")
print(json.dumps(s, indent=2))
PY
  patched_dtb_is_current || \
    die "Verification failed: $PATCHED_DTB has $(dtb_cpu_count "$PATCHED_DTB") cpu nodes, stock has $(dtb_cpu_count "$(stock_dtb)")"
  log "Verified staged DTB keeps all $(dtb_cpu_count "$PATCHED_DTB") stock cpu nodes (UEFI trims per chip)."
}

status_report() {
  ensure_tools
  make_workdir
  dump_live_dts
  local state_json="$WORKDIR/live_state.json"
  summarize_dts_to_json "$ACTIVE_DTS" > "$state_json"

  echo "=== patched DTB base ==="
  if patched_dtb_is_current; then
    echo "$PATCHED_DTB built from the stock DTB ($(dtb_cpu_count "$PATCHED_DTB") cpu nodes) -- OK on any board"
  elif [[ -f "$PATCHED_DTB" ]]; then
    echo "STALE: $PATCHED_DTB has $(dtb_cpu_count "$PATCHED_DTB") cpu nodes, stock has $(dtb_cpu_count "$(stock_dtb)") -- built from a live tree; run: sudo $SCRIPT_NAME apply, then reboot"
  fi
  echo "online CPUs: $(cat /sys/devices/system/cpu/online 2>/dev/null)"
  echo
  echo "=== extlinux FDT lines ==="
  [[ -f "$EXTLINUX" ]] && grep -nE '^LABEL |^[[:space:]]*FDT |^[[:space:]]*LINUX ' "$EXTLINUX" || true
  echo
  echo "=== Active UARTA state from live DT ==="
  print_state_pretty "$state_json"
  echo
  echo "=== UARTA mode from dmesg ==="
  dmesg | egrep '3100000.serial|ttyTHS1|DMA|PIO|iommu group|Adding to iommu group' || true
}

restore_previous() {
  need_root
  [[ -f "$BACKUP_EXTLINUX" ]] || die "No backup extlinux found at $BACKUP_EXTLINUX"
  cp "$BACKUP_EXTLINUX" "$EXTLINUX"
  log "Restored $EXTLINUX from backup."
}

apply_fix() {
  local force="${1:-0}"

  # Cheapest exit first: if the patch is already live there is nothing to do,
  # and detecting that needs no privileges.
  if [[ "$force" -ne 1 ]] && already_applied; then
    local n_fdt
    n_fdt="$(grep -cE '^[[:space:]]*FDT[[:space:]]' "$EXTLINUX")"
    if [[ "$n_fdt" -eq 1 ]]; then
      log "already applied -- serial@3100000 has iommus in the live DT, $PATCHED_DTB exists,"
      log "and $EXTLINUX points at it. Nothing to do (pass --force to rewrite anyway)."
      return 0
    fi
    # The patch is live, but older runs of this script left duplicate FDT lines
    # (bug #2). Before 2026-10-12 this branch returned "nothing to do", so plain
    # `apply` could never clean them up. Repair extlinux.conf ONLY: rebuilding
    # the DTB from the live tree is unnecessary risk when the booted DTB is
    # already the patched one. Every FDT line points at that same file, so the
    # running system is unchanged and no reboot is needed.
    need_root
    local bak
    bak="${EXTLINUX}.bak-$(date +%Y%m%d-%H%M%S)"
    cp "$EXTLINUX" "$bak"
    log "patch already live; $EXTLINUX has $n_fdt FDT lines -- rewriting it to exactly one."
    log "backup: $bak"
    rewrite_extlinux_file "$EXTLINUX" "$PATCHED_DTB"
    log "FDT lines now: $(grep -cE '^[[:space:]]*FDT[[:space:]]' "$EXTLINUX"). DTB untouched; no reboot needed."
    return 0
  fi

  need_root
  ensure_tools
  assert_supported_platform
  make_workdir
  dump_stock_dts

  local pre_json="$WORKDIR/pre_state.json"
  summarize_dts_to_json "$ACTIVE_DTS" > "$pre_json"
  echo "Pre-check state:"
  print_state_pretty "$pre_json"

  patch_dts
  compile_and_install_dtb
  update_extlinux_to_use_patched_dtb
  verify_staged_dtb

  log "Patched DTB installed at $PATCHED_DTB and extlinux.conf updated."
  log "Reboot required. After reboot, run: sudo $SCRIPT_NAME status"
}

main() {
  local mode="${1:-}"
  local do_reboot=0
  local dry_run=0
  local force=0

  case "$mode" in
    apply|status|restore) shift ;;
    current)
      # No root needed: used by provision.sh to decide whether uart_patch must re-run.
      if patched_dtb_is_current; then
        log "$PATCHED_DTB is built from the stock DTB"; exit 0
      fi
      log "$PATCHED_DTB missing or built from a live (per-chip) tree"; exit 1
      ;;
    rewrite-extlinux)
      # Helper entry point for host/tests/test_extlinux_fix.sh. Rewrites the
      # given file in place; never touches /boot unless you point it there.
      shift
      [[ $# -eq 2 ]] || die "Usage: $SCRIPT_NAME rewrite-extlinux <file> <dtb-path>"
      need_cmd python3
      rewrite_extlinux_file "$1" "$2"
      exit 0
      ;;
    -h|--help|'') usage; exit 0 ;;
    *) usage; die "Unknown mode: $mode" ;;
  esac

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --reboot)  do_reboot=1; shift ;;
      --dry-run) dry_run=1; shift ;;
      --force)   force=1; shift ;;
      *) die "Unknown argument: $1" ;;
    esac
  done

  case "$mode" in
    apply)
      if [[ $dry_run -eq 1 ]]; then
        need_cmd python3
        dry_run_extlinux
        exit 0
      fi
      apply_fix "$force"
      if [[ $do_reboot -eq 1 ]]; then
        log "Rebooting now..."
        reboot
      fi
      # Bug #1: this used to be a bare `[[ ... ]] && { ...; }` list, which
      # returns 1 when --reboot is absent. As the last command of main -- itself
      # the last command of the script under `set -Eeuo pipefail` -- that made a
      # successful apply exit 1, aborting every caller.
      exit 0
      ;;
    status)
      status_report
      exit 0
      ;;
    restore)
      restore_previous
      if [[ $do_reboot -eq 1 ]]; then
        log "Rebooting now..."
        reboot
      fi
      exit 0
      ;;
  esac
}

main "$@"
