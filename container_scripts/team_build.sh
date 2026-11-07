#!/usr/bin/env bash
# Inside the skybeta container -- build the team workspace as an overlay on
# /root/skybeta_ws.
#
#   team_build.sh                                  everything in /root/team_ws/src
#   team_build.sh --packages-select my_team_nav    any colcon build arguments
#
# /root/team_ws is the host's $TEAM_WS (default ~/team_ws), bind-mounted by
# host/run.sh, so src/, build/ and install/ all survive the container exiting.
# The container runs as root; afterwards everything is chowned back to the host
# user (HOST_UID/HOST_GID from host/run.sh) so it stays editable from the host.
#
# Builds against the skybeta underlay only, never a previous team install, so a
# stale overlay cannot leak into the build.
set -uo pipefail

WS=/root/team_ws

if [[ ! -d "${WS}/src" ]]; then
  echo "[team_build] ${WS}/src does not exist -- start the container with host/run.sh," >&2
  echo "             which mounts \$TEAM_WS (default ~/team_ws) there." >&2
  exit 1
fi

SKYBETA_UNDERLAY_ONLY=1
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh
unset SKYBETA_UNDERLAY_ONLY

# Warn about team packages that shadow one the underlay already provides. The
# overlay wins for the team's own nodes; the flight pipeline never sees it.
shadowed=0
while read -r name; do
  [[ -n "${name}" ]] || continue
  IFS=: read -ra prefixes <<< "${AMENT_PREFIX_PATH:-}"
  for p in "${prefixes[@]}"; do
    # A shell that already sourced the team overlay (every interactive one does)
    # still has the previous team install on the path; that is not shadowing.
    [[ "${p}" == "${WS}"/* ]] && continue
    if [[ -e "${p}/share/ament_index/resource_index/packages/${name}" ]]; then
      echo "[team_build] WARNING: team package '${name}' shadows the one in ${p}" >&2
      if [[ "${name}" == "px4_msgs" ]]; then
        echo "             px4_msgs MUST match the flight controller (PX4 v1.16.2). The base" >&2
        echo "             already carries release/1.16. Delete ${WS}/src/px4_msgs unless you" >&2
        echo "             know exactly why it is there -- a mismatch connects, lists every" >&2
        echo "             /fmu topic, and moves no data." >&2
      fi
      shadowed=1
      break
    fi
  done
done < <(colcon list --names-only --base-paths "${WS}/src" 2>/dev/null)

cd "${WS}" || exit 1
colcon build --symlink-install "$@"
rc=$?

if [[ -n "${HOST_UID:-}" ]]; then
  for d in build install log src; do
    [[ -e "${WS}/${d}" ]] && chown -R "${HOST_UID}:${HOST_GID:-${HOST_UID}}" "${WS}/${d}"
  done
fi

if [[ ${rc} -eq 0 ]]; then
  echo "[team_build] OK. New shells pick it up automatically; in this one: source skybeta_env.sh"
  [[ ${shadowed} -eq 1 ]] && echo "[team_build] (with shadowed packages -- see the warnings above)"
else
  echo "[team_build] colcon build FAILED (exit ${rc})" >&2
fi
exit "${rc}"
