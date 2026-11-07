#!/usr/bin/env bash
# Inside the skybeta container -- start the full stack in a tmux session named
# 'skybeta' with four windows: agent, slam, bridge, competitor. Attaches at the
# end; Ctrl-b d to detach, `tmux kill-session -t skybeta` (or just exit the
# container) to tear everything down.
#
# Environment:
#   SESSION                tmux session name           (skybeta)
#   SLAM_DELAY_S           agent -> slam gap in s      (3)
#   BRIDGE_DELAY_S         slam -> bridge gap in s     (8)
#   COMPETITOR_DELAY_S     bridge -> competitor gap    (2)
#   CAMERA_X/Y/Z           camera translation, m FLU   (0.0)
#   CAMERA_ROLL/PITCH/YAW  camera rotation, rad FLU    (0.0)
#   COMPETITOR_PKG         competitor package          (competitor_example)
#   COMPETITOR_LAUNCH      competitor launch file      (competitor.launch.py)
#                          Both normally come from /etc/skybeta/drone.env via
#                          host/run.sh. The package may live in the team
#                          workspace, which run_competitor.sh builds first.
#   XRCE_DEV / XRCE_BAUD   agent transport             (/dev/ttyTHS1, 921600)
set -euo pipefail

SESSION="${SESSION:-skybeta}"
SLAM_DELAY_S="${SLAM_DELAY_S:-3}"           # let agent bind UART before SLAM starts
BRIDGE_DELAY_S="${BRIDGE_DELAY_S:-8}"       # let cuVSLAM init before bridge subscribes
COMPETITOR_DELAY_S="${COMPETITOR_DELAY_S:-2}"

CAMERA_X="${CAMERA_X:-0.0}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.0}"
CAMERA_ROLL="${CAMERA_ROLL:-0.0}"
CAMERA_PITCH="${CAMERA_PITCH:-0.0}"
CAMERA_YAW="${CAMERA_YAW:-0.0}"

COMPETITOR_PKG="${COMPETITOR_PKG:-competitor_example}"
COMPETITOR_LAUNCH="${COMPETITOR_LAUNCH:-competitor.launch.py}"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[stack] session '${SESSION}' already running -- attaching"
  exec tmux attach -t "${SESSION}"
fi

# remain-on-exit keeps dead panes visible with their exit code instead of
# silently disappearing -- failures are loud, not silent.
#
# It is a WINDOW option, so `set-option -t <session>` only ever reached the
# session's *current* window and the later windows died silently (bug #6).
# `-g` sets the global window option, which every window inherits.
#
# It is set in the same invocation that starts the server: a bare
# `tmux start-server` does not survive on its own (exit-empty defaults to on,
# so a server with no session exits immediately and the next tmux command
# fails). The first window is an idle shell until the send-keys below, and
# remain-on-exit is read when a pane's process exits, not when it is created,
# so setting it here covers window 0 as well.
tmux new-session -d -s "${SESSION}" -n agent \; set-option -g remain-on-exit on
tmux send-keys   -t "${SESSION}:agent" 'exec run_agent.sh' C-m
sleep "${SLAM_DELAY_S}"

tmux new-window  -t "${SESSION}" -n slam
tmux send-keys   -t "${SESSION}:slam" \
  "exec run_slam.sh ${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} ${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW}" C-m
sleep "${BRIDGE_DELAY_S}"

tmux new-window  -t "${SESSION}" -n bridge
tmux send-keys   -t "${SESSION}:bridge" 'exec run_bridge.sh' C-m
sleep "${COMPETITOR_DELAY_S}"

# The competitor package is what teams write. run_competitor.sh builds the team
# workspace (/root/team_ws) if it holds packages, then launches COMPETITOR_PKG
# with the team overlay sourced. A failed build or a missing package leaves a
# shell with a hint rather than a dead pane. tmux windows inherit this
# container's environment, so COMPETITOR_PKG/LAUNCH from drone.env reach it.
export COMPETITOR_PKG COMPETITOR_LAUNCH
tmux new-window  -t "${SESSION}" -n competitor
tmux send-keys   -t "${SESSION}:competitor" 'exec run_competitor.sh' C-m

cat <<EOM
[stack] tmux session '${SESSION}' is up (remain-on-exit on -- dead panes keep their exit code):
  0 agent       MicroXRCEAgent ${XRCE_TRANSPORT:-serial} ${XRCE_DEV:-/dev/ttyTHS1} @ ${XRCE_BAUD:-921600}
  1 slam        cuVSLAM + OAK-D; camera FLU xyz=${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m  rpy=${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad
  2 bridge      /visual_slam/tracking/odometry -> /fmu/in/vehicle_visual_odometry
  3 competitor  team_build.sh, then ros2 launch ${COMPETITOR_PKG} ${COMPETITOR_LAUNCH} (shell if not found)

  drone ${DRONE_ID:-?} / team ${TEAM:-?}   DDS domain ${ROS_DOMAIN_ID:-0}, scope ${SKYBETA_DDS_SCOPE:-network}

  check_topics.sh   prove the DDS link actually carries data
  record_bag.sh     mcap recording of the camera / SLAM / PX4 topics

Ctrl-b 0..3  switch window     Ctrl-b d  detach     exit  to tear down
EOM

exec tmux attach -t "${SESSION}"
