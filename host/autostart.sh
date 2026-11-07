#!/usr/bin/env bash
# Start the skybeta stack at boot on THIS drone (systemd: skybeta-stack.service).
#
#   sudo host/autostart.sh enable    install the unit for this checkout, start it now
#   sudo host/autostart.sh disable   stop it and remove the unit
#   host/autostart.sh status         is it installed / running, last log lines
#   host/autostart.sh render         print the unit that `enable` would install
#
# With autostart on, the stack is always running, so:
#   attach      docker exec -it skybeta-stack tmux attach -t skybeta   (Ctrl-b d detaches)
#   restart     docker rm -f skybeta-stack     (back in ~10 s, re-reading drone.env)
#   stop        sudo systemctl stop skybeta-stack   (until the next boot)
#   logs        journalctl -u skybeta-stack -f
# host/run.sh stack refuses to start a second copy and prints the same hints.
#
# Needs the patched FC firmware (v1.16.2-1-g7a39c59, CLAUDE.md bug #15): the
# service restarts the agent whenever a pipeline window dies, and on stock
# v1.16.2 the FC's TELEM2 goes silent on every agent loss until a power cycle.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"
TEMPLATE="${HERE}/systemd/skybeta-stack.service.in"
UNIT_NAME="skybeta-stack.service"
UNIT_PATH="${SKYBETA_UNIT_DIR:-/etc/systemd/system}/${UNIT_NAME}"

log() { printf '[autostart] %s\n' "$*"; }
die() { printf '[autostart] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

# The account the stack runs as: the person who typed sudo, not root.
STACK_USER="${SKYBETA_STACK_USER:-${SUDO_USER:-$(id -un)}}"

render() {
  [[ -f "${TEMPLATE}" ]] || die "${TEMPLATE} missing"
  [[ "${REPO}" != *[[:space:]@]* ]] || die "repo path '${REPO}' contains a space or @ -- systemd unit paths cannot"
  sed -e "s|@USER@|${STACK_USER}|g" -e "s|@REPO@|${REPO}|g" "${TEMPLATE}"
}

need_root() {
  [[ ${EUID} -eq 0 ]] || die "'$1' writes /etc/systemd/system -- run: sudo host/autostart.sh $1"
}

case "${1:-}" in
  render)
    render
    ;;

  enable)
    need_root enable
    [[ "${STACK_USER}" != "root" ]] || die "refusing to run the stack as root; run via sudo from the drone user, or set SKYBETA_STACK_USER"
    id "${STACK_USER}" >/dev/null 2>&1 || die "no such user: ${STACK_USER}"
    id -nG "${STACK_USER}" | tr ' ' '\n' | grep -qx docker \
      || die "${STACK_USER} is not in the docker group -- the service could not start containers"
    [[ -f /etc/skybeta/drone.env ]] \
      || log "WARNING: /etc/skybeta/drone.env missing -- the stack would run as an anonymous drone (see host/drone.env.example)"
    tmp="$(mktemp)"
    render > "${tmp}"
    install -D -m 0644 "${tmp}" "${UNIT_PATH}"
    rm -f "${tmp}"
    log "installed ${UNIT_PATH} (user ${STACK_USER}, repo ${REPO})"
    systemctl daemon-reload
    systemctl enable "${UNIT_NAME}"
    # A stack started by hand under the same name is replaced by the service's own.
    systemctl restart "${UNIT_NAME}"
    log "enabled and started. The stack now starts on every boot."
    log "watch it come up:  journalctl -u skybeta-stack -f"
    ;;

  disable)
    need_root disable
    if [[ -f "${UNIT_PATH}" ]]; then
      systemctl disable --now "${UNIT_NAME}" || true
      rm -f "${UNIT_PATH}"
      systemctl daemon-reload
      log "disabled and removed ${UNIT_PATH}. Start the stack by hand again: host/run.sh stack"
    else
      log "not installed (${UNIT_PATH} absent) -- nothing to do"
    fi
    ;;

  status)
    if [[ -f "${UNIT_PATH}" ]]; then
      log "installed: ${UNIT_PATH}"
      log "enabled:   $(systemctl is-enabled "${UNIT_NAME}" 2>/dev/null || true)"
      log "active:    $(systemctl is-active "${UNIT_NAME}" 2>/dev/null || true)"
    else
      log "not installed -- the stack starts only by hand (host/run.sh stack)"
    fi
    if docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep '^skybeta-stack '; then :; else
      log "no skybeta-stack container running"
    fi
    if [[ -f "${UNIT_PATH}" ]]; then
      journalctl -u "${UNIT_NAME}" -n 15 --no-pager 2>/dev/null \
        || log "(journalctl not readable by $(id -un); try: sudo journalctl -u skybeta-stack)"
    fi
    ;;

  -h|--help|'')
    usage
    ;;

  *)
    usage >&2
    die "unknown command: $1"
    ;;
esac
