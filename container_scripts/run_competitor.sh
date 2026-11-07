#!/usr/bin/env bash
# Inside the skybeta container -- the full stack's `competitor` window.
#
#   1. If the team workspace (/root/team_ws/src) holds any package, build it
#      (team_build.sh). A failed build drops to a shell showing the error.
#   2. Launch COMPETITOR_PKG / COMPETITOR_LAUNCH (from drone.env via host/run.sh;
#      default competitor_example / competitor.launch.py) with the team overlay
#      sourced, so a package in team_ws resolves before the built-in examples.
#   3. If that package exists nowhere, say so and keep a shell.
set -uo pipefail

PKG="${COMPETITOR_PKG:-competitor_example}"
LAUNCH="${COMPETITOR_LAUNCH:-competitor.launch.py}"

if find /root/team_ws/src -name package.xml -print -quit 2>/dev/null | grep -q .; then
  echo "[competitor] building the team workspace"
  if ! team_build.sh; then
    echo "[competitor] team_ws build failed -- fix it, then:  team_build.sh && ros2 launch ${PKG} ${LAUNCH}"
    exec bash
  fi
fi

# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

if ros2 pkg prefix "${PKG}" >/dev/null 2>&1; then
  echo "[competitor] ros2 launch ${PKG} ${LAUNCH}"
  exec ros2 launch "${PKG}" "${LAUNCH}"
fi

echo "[competitor] package '${PKG}' not found. Put it in ~/team_ws/src on the drone"
echo "             (/root/team_ws/src in here), set COMPETITOR_PKG in /etc/skybeta/drone.env,"
echo "             and restart the stack. Shell kept."
exec bash
