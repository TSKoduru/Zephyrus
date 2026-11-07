#!/usr/bin/env bash
# skybeta host provisioning -- ONE idempotent, resumable entry point.
#
# Replaces 01_prereqs.sh / 02_uart_fix.sh / 02b_uart_fix_verify.sh /
# setup_and_build.sh. Phases run in order and each records itself in
#
#     $HOME/.skybeta/state          (one completed phase name per line)
#
# ...deliberately under $HOME rather than /var/lib/skybeta, so that recording
# progress never itself needs root.
#
# Phases:
#   prereqs     apt deps, OAK-D udev rule, docker nvidia runtime, group membership
#   uart_patch  jetson_uart_dma_fix.sh apply  (needs one reboot the first time)
#   power       nvpmodel MAXN + jetson_clocks, persisted across reboot
#   pull        host/pull_image.sh   (the container image; WP0 owns that script)
#   doctor      host/doctor.sh
#
# Usage:
#   host/provision.sh                     # resume from wherever it stopped
#   host/provision.sh --dry-run           # print what every phase would do
#   host/provision.sh --from power        # re-run from a phase, ignoring state
#   host/provision.sh --skip pull         # repeatable
#   host/provision.sh --list              # show phase status and exit
#
# The uart_patch phase needs a reboot. It prints instructions and exits 0; the
# next run detects the live patch and carries on.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"

STATE_DIR="${SKYBETA_STATE_DIR:-${HOME}/.skybeta}"
STATE="${STATE_DIR}/state"
PHASES=(prereqs uart_patch power pull doctor)

DRY_RUN=0
FROM=""
SKIP=()
LIST=0

log()  { printf '[provision] %s\n' "$*"; }
warn() { printf '[provision] WARNING: %s\n' "$*" >&2; }
die()  { printf '[provision] ERROR: %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,27p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

is_phase() { local p; for p in "${PHASES[@]}"; do [[ "$p" == "$1" ]] && return 0; done; return 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    --from)    FROM="${2:-}"; is_phase "${FROM}" || die "--from: unknown phase '${FROM}' (want one of: ${PHASES[*]})"; shift 2 ;;
    --skip)    is_phase "${2:-}" || die "--skip: unknown phase '${2:-}' (want one of: ${PHASES[*]})"; SKIP+=("$2"); shift 2 ;;
    --list)    LIST=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
done

# --- state -----------------------------------------------------------------
mkdir -p "${STATE_DIR}"
[[ -f "${STATE}" ]] || : > "${STATE}"

phase_done()   { grep -qxF "$1" "${STATE}" 2>/dev/null; }
mark_done()    { phase_done "$1" || echo "$1" >> "${STATE}"; }
phase_skipped(){ local p; for p in "${SKIP[@]:-}"; do [[ "$p" == "$1" ]] && return 0; done; return 1; }

# --- command runner --------------------------------------------------------
# In --dry-run nothing is executed -- in particular no `sudo`, which on this
# fleet prompts for a password (CLAUDE.md).
run() {
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '    would run: %s\n' "$*"
  else
    "$@"
  fi
}
note() { printf '    %s\n' "$*"; }

# --- platform gate ---------------------------------------------------------
assert_r36() {
  [[ -r /etc/nv_tegra_release ]] || die "/etc/nv_tegra_release missing -- this is not a JetPack Jetson rootfs. skybeta host provisioning targets L4T R36.x on Orin."
  local rel; rel="$(head -1 /etc/nv_tegra_release)"
  [[ "${rel}" == *"R36"* ]] || die "unsupported L4T release. Need R36.x, got: ${rel}"
  log "L4T: ${rel}"
}

# --- helpers ---------------------------------------------------------------
# Live AND built from the stock DTB. A patched DTB made from another board's live
# tree (golden-image clones of older images) boots, but with that board's CPU trim.
uart_patch_live() {
  [[ -e /proc/device-tree/bus@0/serial@3100000/iommus ]] && \
  [[ -e /proc/device-tree/bus@0/serial@3100000/dmas ]] && \
  bash "${HERE}/jetson_uart_dma_fix.sh" current >/dev/null 2>&1
}

maxn_pmode_id() {
  # Board-specific: on this Orin Nano Super, MAXN_SUPER is ID 2 and ID 0 is 15W
  # -- do NOT hardcode 0. Resolve by name, fall back to 0.
  local conf id
  conf="$(cat /var/lib/nvpmodel/conf_file_path 2>/dev/null || echo /etc/nvpmodel.conf)"
  id="$(grep -oE '< POWER_MODEL ID=[0-9]+ NAME=MAXN[^ ]*' "${conf}" 2>/dev/null \
        | head -1 | sed -n 's/.*ID=\([0-9]\+\).*/\1/p')"
  echo "${SKYBETA_PMODE:-${id:-0}}"
}

current_pmode_id() {
  sed -n 's/.*pmode:0*\([0-9]\+\).*/\1/p' /var/lib/nvpmodel/status 2>/dev/null
}

# ===========================================================================
# Phases
# ===========================================================================

phase_prereqs() {
  log "phase prereqs -- apt deps, udev rule, docker nvidia runtime, groups"

  # Docker itself ships with JetPack 6.x; installing docker.io here would
  # conflict with the JetPack containerd.
  run sudo apt-get update
  run sudo apt-get install -y \
    git curl wget jq \
    python3-pip python3-serial \
    udev usbutils \
    device-tree-compiler \
    nvidia-container-toolkit

  # OAK-D (Luxonis MyriadX) -- the device re-enumerates with a different PID
  # once depthai pushes firmware, so the rule matches on vendor only.
  local rule='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'
  if grep -qs '03e7' /etc/udev/rules.d/80-movidius.rules 2>/dev/null; then
    note "udev rule already present"
  else
    if [[ "${DRY_RUN}" -eq 1 ]]; then
      note "would write /etc/udev/rules.d/80-movidius.rules: ${rule}"
      note "would run: sudo udevadm control --reload-rules && sudo udevadm trigger"
    else
      echo "${rule}" | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
      sudo udevadm control --reload-rules
      sudo udevadm trigger
      note "udev rule installed -- replug the OAK-D"
    fi
  fi

  command -v docker >/dev/null 2>&1 || die "docker not found. JetPack normally ships it; otherwise: sudo apt-get install -y nvidia-docker2"

  # Register the nvidia runtime if it is missing. Deliberately NOT setting
  # default-runtime=nvidia (01_prereqs.sh used to): every docker run in this
  # repo passes --runtime nvidia explicitly, the image build needs no GPU, and
  # flipping the default restarts dockerd for no benefit. See CLAUDE.md.
  if docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null | grep -qw nvidia; then
    note "docker nvidia runtime already registered (default runtime left alone)"
  else
    if [[ "${DRY_RUN}" -eq 1 ]]; then
      note "would patch /etc/docker/daemon.json to add runtimes.nvidia, then: sudo systemctl restart docker"
    else
      local daemon=/etc/docker/daemon.json tmp
      tmp="$(mktemp)"
      if [[ -f "${daemon}" ]]; then
        sudo cp "${daemon}" "${daemon}.bak.$(date +%s)"
        jq '.runtimes.nvidia //= {"path":"nvidia-container-runtime","args":[]}' "${daemon}" > "${tmp}"
      else
        printf '{\n  "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "args": [] } }\n}\n' > "${tmp}"
      fi
      sudo mv "${tmp}" "${daemon}"
      sudo systemctl restart docker
      note "nvidia runtime registered; docker restarted"
    fi
  fi

  # Group membership: with dialout + docker the UART and docker need no sudo.
  local want=() g
  for g in docker dialout; do
    [[ " $(id -nG) " == *" ${g} "* ]] || want+=("${g}")
  done
  if [[ ${#want[@]} -eq 0 ]]; then
    note "$(id -un) already in docker and dialout"
  else
    run sudo usermod -aG "$(IFS=,; echo "${want[*]}")" "$(id -un)"
    note "added to: ${want[*]} -- LOG OUT AND BACK IN for this to take effect"
  fi
}

phase_uart_patch() {
  log "phase uart_patch -- Jetson Orin R36.5 UART DMA/IOMMU device-tree patch"

  if uart_patch_live; then
    note "already live: /proc/device-tree/bus@0/serial@3100000/{iommus,dmas} present"
    note "no reboot needed"
    return 0
  fi

  local fix="${HERE}/jetson_uart_dma_fix.sh"
  [[ -f "${fix}" ]] || die "missing ${fix}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    note "would run: sudo ${fix} apply"
    note "(preview of the extlinux.conf change, no root, nothing written:)"
    bash "${fix}" apply --dry-run 2>&1 | sed 's/^/      /' || true
    note "then: REBOOT, and re-run host/provision.sh to continue"
    return 0
  fi

  sudo "${fix}" apply
  cat <<'EOM'

    The patched DTB is staged and extlinux.conf points at it, but the DMA/IOMMU
    path only becomes live after a reboot. Nothing else can be provisioned until
    then, so this run stops here -- cleanly.

        sudo reboot
        host/provision.sh        # resumes at uart_patch, sees it live, continues

EOM
  # Deliberately NOT marked done: the next run re-enters this phase, sees the
  # live device tree, and marks it then.
  exit 0
}

phase_power() {
  log "phase power -- nvpmodel MAXN + jetson_clocks (persisted)"

  local want cur
  want="$(maxn_pmode_id)"
  cur="$(current_pmode_id || true)"
  note "power model: current=${cur:-unknown} target=${want} (MAXN)"

  if ! command -v nvpmodel >/dev/null 2>&1; then
    warn "nvpmodel not found -- skipping the power model change"
  elif [[ "${cur}" == "${want}" ]]; then
    note "already on power model ${want}; nvpmodel persists across reboot on its own"
  else
    run sudo nvpmodel -m "${want}"
  fi

  # jetson_clocks pins the clocks to max but does NOT survive a reboot, so
  # install a tiny oneshot unit rather than relying on someone remembering.
  if ! command -v jetson_clocks >/dev/null 2>&1; then
    warn "jetson_clocks not found -- skipping"
    return 0
  fi
  run sudo jetson_clocks

  local unit=/etc/systemd/system/skybeta-jetson-clocks.service
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    note "would write ${unit} (oneshot: nvpmodel -m ${want} && jetson_clocks) and: sudo systemctl enable skybeta-jetson-clocks.service"
  else
    sudo tee "${unit}" >/dev/null <<EOF
[Unit]
Description=skybeta: pin Jetson to MAXN and max clocks
After=nvpmodel.service
Wants=nvpmodel.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/nvpmodel -m ${want}
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable skybeta-jetson-clocks.service
    note "installed and enabled ${unit}"
  fi
}

phase_pull() {
  log "phase pull -- container image"

  local puller="${HERE}/pull_image.sh"
  local tag; tag="$(tr -d '[:space:]' < "${REPO}/VERSION" 2>/dev/null || true)"
  local image="skybeta:${tag:-latest}"

  if docker image inspect "${image}" >/dev/null 2>&1; then
    note "${image} already present locally"
    return 0
  fi

  if [[ ! -f "${puller}" ]]; then
    warn "${puller} not found (WP0 owns it). Build locally instead: host/build_image.sh"
    return 0
  fi
  if [[ -z "${tag}" ]]; then
    warn "no ${REPO}/VERSION file -- pull_image.sh refuses the 'latest' tag on purpose."
    warn "Pin a version in VERSION, or build locally: host/build_image.sh"
    return 0
  fi
  if [[ -z "${REGISTRY:-}" ]]; then
    warn "REGISTRY is unset -- pull_image.sh needs it, e.g. REGISTRY=ghcr.io/<org>."
    warn "Skipping the pull; build locally instead: host/build_image.sh"
    return 0
  fi
  run bash "${puller}" "${tag}"
}

phase_doctor() {
  log "phase doctor -- health check"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    note "would run: ${HERE}/doctor.sh"
    return 0
  fi
  # doctor exits non-zero on any FAIL; surface that but keep the state marker,
  # since the provisioning steps themselves did succeed.
  bash "${HERE}/doctor.sh" || DOCTOR_FAILED=1
}

# ===========================================================================
# Driver
# ===========================================================================

if [[ "${LIST}" -eq 1 ]]; then
  echo "phases (state: ${STATE})"
  for p in "${PHASES[@]}"; do
    if phase_done "$p"; then printf '  [x] %s\n' "$p"; else printf '  [ ] %s\n' "$p"; fi
  done
  exit 0
fi

assert_r36

[[ "${DRY_RUN}" -eq 1 ]] && log "DRY RUN -- nothing will be executed or written"
log "state file: ${STATE}"

DOCTOR_FAILED=0
started=0
[[ -z "${FROM}" ]] && started=1

for p in "${PHASES[@]}"; do
  if [[ -n "${FROM}" && "${started}" -eq 0 ]]; then
    if [[ "$p" == "${FROM}" ]]; then started=1; else
      log "phase ${p} -- before --from ${FROM}, skipping"
      continue
    fi
  fi

  if phase_skipped "$p"; then
    log "phase ${p} -- skipped (--skip)"
    continue
  fi

  # --from re-runs its phase and everything after, ignoring the state file.
  if [[ -z "${FROM}" ]] && phase_done "$p"; then
    log "phase ${p} -- already done (state); use --from ${p} to re-run"
    continue
  fi

  "phase_${p}"
  [[ "${DRY_RUN}" -eq 1 ]] || mark_done "$p"
  echo
done

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "dry run complete -- no changes made."
  exit 0
fi

if [[ "${DOCTOR_FAILED}" -eq 1 ]]; then
  log "provisioning steps completed, but doctor reported at least one FAIL (see above)."
  exit 1
fi

log "provisioning complete. Next: host/run.sh stack"
exit 0
