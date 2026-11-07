#!/usr/bin/env bash
# Inside the skybeta container -- launch cuVSLAM + OAK-D driver + the
# base_link -> oak_parent_frame static TF.
#
# Camera pose is FLU body frame: translation in metres, rotation in radians.
# Positional args win over the CAMERA_* environment variables.
#
#   run_slam.sh                              # camera at body origin
#   run_slam.sh 0.10 0.0 -0.02               # x y z
#   run_slam.sh 0.10 0.0 -0.02 0 0.35 0      # plus a 20 deg down-pitch
#   CAMERA_X=0.10 CAMERA_PITCH=0.35 run_slam.sh
#
# An unmodelled mount rotation couples vehicle rotation into apparent
# translation -- measure the real angles, do not leave them at 0 by default
# (CLAUDE.md, "Toilet bowl" suspect #4).
#
# Keep EKF2_EV_POS_X/Y/Z = 0 on the flight controller: this TF already carries
# the lever arm, and setting both applies it twice (bug #9).
set -euo pipefail

CAMERA_X="${1:-${CAMERA_X:-0.0}}"
CAMERA_Y="${2:-${CAMERA_Y:-0.0}}"
CAMERA_Z="${3:-${CAMERA_Z:-0.0}}"
CAMERA_ROLL="${4:-${CAMERA_ROLL:-0.0}}"
CAMERA_PITCH="${5:-${CAMERA_PITCH:-0.0}}"
CAMERA_YAW="${6:-${CAMERA_YAW:-0.0}}"

# Flight-critical path: the skybeta underlay only, never the team overlay, so a
# team package shadowing an underlay one cannot reach the VIO -> FC pipeline.
# skybeta_env.sh handles the nounset dance the ROS setup scripts need.
SKYBETA_UNDERLAY_ONLY=1
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

LAUNCH_ARGS=(
  camera_x:="${CAMERA_X}"
  camera_y:="${CAMERA_Y}"
  camera_z:="${CAMERA_Z}"
)

# camera_roll/pitch/yaw are WP3 launch arguments. Only pass them when set, so
# an image whose launch file predates them is not handed arguments it does not
# declare. `ros2 launch` ignores unknown top-level arguments silently, hence
# the explicit warning below rather than a crash.
#
# NB: `[[ cond ]] && cmd` as a bare statement makes the script exit 1 under
# `set -e` when the condition is false -- the same trap as bug #1. Use if.
ROT_ARGS=()
if [[ -n "${CAMERA_ROLL}"  ]]; then ROT_ARGS+=(camera_roll:="${CAMERA_ROLL}"); fi
if [[ -n "${CAMERA_PITCH}" ]]; then ROT_ARGS+=(camera_pitch:="${CAMERA_PITCH}"); fi
if [[ -n "${CAMERA_YAW}"   ]]; then ROT_ARGS+=(camera_yaw:="${CAMERA_YAW}"); fi

if [[ ${#ROT_ARGS[@]} -gt 0 ]]; then
  share="$(ros2 pkg prefix --share oakd_vslam_bringup 2>/dev/null || true)"
  launch_file="${share:+${share}/launch/oakd_vslam.launch.py}"
  if [[ -n "${launch_file}" && -f "${launch_file}" ]] \
     && ! grep -q "camera_roll" "${launch_file}"; then
    if [[ "${CAMERA_ROLL}${CAMERA_PITCH}${CAMERA_YAW}" =~ [1-9] ]]; then
      echo "[slam] WARNING: oakd_vslam.launch.py in this image declares no" >&2
      echo "       camera_roll/pitch/yaw -- the requested mount rotation will be" >&2
      echo "       IGNORED and the camera treated as axis-aligned with base_link." >&2
    fi
    ROT_ARGS=()
  fi
  if [[ ${#ROT_ARGS[@]} -gt 0 ]]; then
    LAUNCH_ARGS+=("${ROT_ARGS[@]}")
  fi
fi

echo "[slam] camera pose (FLU): xyz=${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m" \
     "rpy=${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad"
exec ros2 launch oakd_vslam_bringup oakd_vslam.launch.py "${LAUNCH_ARGS[@]}"
