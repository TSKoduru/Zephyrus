#!/usr/bin/env bash
# The boot-time stack: what skybeta-stack.service runs (installed per drone by
# host/autostart.sh). Not meant to be run by hand -- use host/run.sh stack.
#
#   1. removes a leftover skybeta-stack container (crash, previous boot), and
#      waits up to STACK_TIME_SYNC_WAIT_S for the system clock to be NTP-synced;
#   2. starts the full stack detached: SKYBETA_DETACH=1 host/run.sh stack
#      (drone.env supplies domain, DDS scope, camera pose, competitor package);
#   3. every STACK_CHECK_S seconds checks the container is running and that the
#      tmux panes the VIO -> FC pipeline needs (agent, slam, bridge) are alive.
#      If one has died it logs the pane's exit status and last 40 lines to the
#      journal, removes the container and exits 1, and systemd starts a fresh
#      stack RestartSec later. A dead competitor pane is logged, never restarted:
#      team code must not be able to take the drone's position hold down with it.
#   4. on SIGTERM (systemctl stop, shutdown) stops the container cleanly.
#   5. restarts the stack if the system clock steps (wall clock moves against
#      CLOCK_MONOTONIC by more than STACK_CLOCK_STEP_S).
#
# Why the clock matters: the Jetson has no RTC battery and boots at the last
# shutdown time; NTP then steps it. The OAK-D driver keeps stamping frames on the
# clock it saw at start, so after a step every /fmu/in/vehicle_visual_odometry
# carries a timestamp_sample minutes off, EKF2 rejects it, and Position mode loses
# its only horizontal source. Seen on drone 16, 2026-11-03: boot at the restored
# 22:58:21, NTP step at 23:04:08, samples 317 s stale at the FC, cs_ev_pos false
# ~3 min later; a stack restart made them 57 ms old again.
#
# Restarting the agent is safe only on the patched FC firmware
# (v1.16.2-1-g7a39c59, CLAUDE.md bug #15): on stock v1.16.2 the FC's TELEM2 dies
# on every agent loss until the FC is power-cycled.
#
# Environment (all optional):
#   STACK_CHECK_S   seconds between health checks          (5)
#   STACK_GRACE_S   seconds allowed for all windows to appear (90)
#   STACK_TIME_SYNC_WAIT_S  max wait for NTP sync before starting (90; 0 = none).
#                   With no network the stack starts anyway once it expires; if
#                   NTP arrives later, the clock-step check restarts the stack.
#   STACK_CLOCK_STEP_S      wall-vs-monotonic jump that triggers a restart (0.5)
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="skybeta-stack"          # run.sh's default; the service owns this name
SESSION="skybeta"             # run_full_stack.sh's tmux session
CRITICAL=(agent slam bridge)
CHECK_S="${STACK_CHECK_S:-5}"
GRACE_S="${STACK_GRACE_S:-90}"
SYNC_WAIT_S="${STACK_TIME_SYNC_WAIT_S:-90}"
STEP_S="${STACK_CLOCK_STEP_S:-0.5}"

log() { printf '[stack-service] %s\n' "$*"; }

stopping=0
on_term() {
  stopping=1
  log "stop requested -- docker stop ${NAME}"
  docker stop -t 15 "${NAME}" >/dev/null 2>&1 || true
  exit 0
}
trap on_term TERM INT

# Sleep that a SIGTERM interrupts at once (bash runs traps only between commands).
nap() { sleep "$1" & wait $! 2>/dev/null || true; }

panes() {
  docker exec "${NAME}" tmux list-panes -s -t "${SESSION}" \
    -F '#{window_name} #{pane_dead} #{pane_dead_status}' 2>/dev/null
}

# wall clock minus monotonic clock, in seconds; constant unless the clock steps
clock_offset() {
  awk -v w="$(date +%s.%N)" '{ printf "%.3f", w - $1 }' /proc/uptime
}

ntp_synced() {
  [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]]
}

fail_with_pane() {
  local win="$1" status="$2"
  log "window '${win}' died (exit ${status:-?}) -- last lines:"
  docker exec "${NAME}" tmux capture-pane -p -J -S -40 -t "${SESSION}:${win}" 2>/dev/null \
    | sed '/^[[:space:]]*$/d; s/^/    /' || true
  log "removing ${NAME}; systemd restarts the whole stack"
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
  exit 1
}

if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  log "removing a leftover ${NAME} container"
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
fi

if [[ "${SYNC_WAIT_S}" != "0" ]] && command -v timedatectl >/dev/null 2>&1 && ! ntp_synced; then
  log "system clock not NTP-synced yet ($(date '+%F %T')) -- waiting up to ${SYNC_WAIT_S} s so sensor timestamps do not jump later"
  t_sync=${SECONDS}
  while ! ntp_synced && (( SECONDS - t_sync < SYNC_WAIT_S )); do nap 2; done
  if ntp_synced; then
    log "clock synced after $((SECONDS - t_sync)) s: $(date '+%F %T')"
  else
    log "no NTP sync after ${SYNC_WAIT_S} s (no network?) -- starting anyway; a later clock step restarts the stack"
  fi
fi

log "starting the stack (drone.env via host/run.sh)"
SKYBETA_DETACH=1 NAME="${NAME}" bash "${HERE}/run.sh" stack || {
  log "host/run.sh stack failed (see above)"
  exit 1
}

# Wait for run_full_stack.sh to open every window (it staggers them ~13 s).
t0=${SECONDS}
while :; do
  out="$(panes)" || out=""
  missing=()
  for w in "${CRITICAL[@]}"; do
    grep -q "^${w} " <<<"${out}" || missing+=("${w}")
  done
  [[ ${#missing[@]} -eq 0 ]] && break
  if [[ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" != "true" ]]; then
    log "${NAME} exited during start-up"
    docker logs --tail 40 "${NAME}" 2>&1 | sed 's/^/    /' || true
    exit 1
  fi
  if (( SECONDS - t0 > GRACE_S )); then
    log "windows still missing after ${GRACE_S} s: ${missing[*]}"
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    exit 1
  fi
  nap 2
done
log "stack up: windows ${CRITICAL[*]} running; checking every ${CHECK_S} s"
offset0="$(clock_offset)"

competitor_reported=0
tmux_misses=0
while :; do
  nap "${CHECK_S}"
  [[ ${stopping} -eq 1 ]] && exit 0
  if [[ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" != "true" ]]; then
    log "${NAME} is gone (docker rm -f / docker stop / crash) -- exiting so systemd restarts it"
    exit 1
  fi
  if ! out="$(panes)"; then
    tmux_misses=$((tmux_misses + 1))
    if (( tmux_misses >= 3 )); then
      log "tmux session '${SESSION}' not answering for $((3 * CHECK_S)) s"
      docker rm -f "${NAME}" >/dev/null 2>&1 || true
      exit 1
    fi
    continue
  fi
  tmux_misses=0
  offset="$(clock_offset)"
  if awk -v a="${offset}" -v b="${offset0}" -v t="${STEP_S}" 'BEGIN { d = a - b; if (d < 0) d = -d; exit !(d > t) }'; then
    log "system clock stepped by $(awk -v a="${offset}" -v b="${offset0}" 'BEGIN { printf "%+.1f", a - b }') s (NTP?) -- sensor timestamps are now wrong and EKF2 would reject the vision data; restarting the stack"
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    exit 1
  fi
  while read -r win dead status; do
    [[ "${dead}" == "1" ]] || continue
    for w in "${CRITICAL[@]}"; do
      [[ "${win}" == "${w}" ]] && fail_with_pane "${win}" "${status}"
    done
    if [[ "${win}" == "competitor" && ${competitor_reported} -eq 0 ]]; then
      competitor_reported=1
      log "competitor window exited (status ${status:-?}); VIO and the FC link keep running. Attach to see why: docker exec -it ${NAME} tmux attach -t ${SESSION}"
    fi
  done <<<"${out}"
done
