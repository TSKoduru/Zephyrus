# shellcheck shell=bash
# Inside the skybeta container -- SOURCE this, do not run it. Sets up ROS:
#
#   /opt/ros/humble                      ROS 2 Humble
#   /root/skybeta_ws/install             the skybeta underlay (px4_msgs @ release/1.16,
#                                        bringup, flight interface, examples)
#   /root/team_ws/install                the team overlay, if it has been built
#                                        (host/run.sh bind-mounts $TEAM_WS there)
#
# SKYBETA_UNDERLAY_ONLY=1 skips the team overlay. The flight-critical pipeline
# (run_slam.sh, run_bridge.sh, run_agent.sh) always sets it, so a team package
# that shadows an underlay one -- a px4_msgs cloned from PX4's main branch is the
# classic -- can break the team's own code but never the VIO -> FC path.
#
# Interactive shells get this from ~/.bashrc.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "skybeta_env.sh must be sourced:  source skybeta_env.sh" >&2
  exit 2
fi

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.); relax
# nounset just for the source, then restore whatever the caller had.
_skybeta_had_u=0
if [[ $- == *u* ]]; then _skybeta_had_u=1; fi
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
if [[ "${SKYBETA_UNDERLAY_ONLY:-0}" != "1" && -f /root/team_ws/install/local_setup.bash ]]; then
  source /root/team_ws/install/local_setup.bash
fi
if [[ ${_skybeta_had_u} -eq 1 ]]; then set -u; fi
unset _skybeta_had_u
