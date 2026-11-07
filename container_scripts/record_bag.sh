#!/usr/bin/env bash
# Inside the skybeta container -- record the flight-debug topic set to an mcap
# bag. Everything needed to re-run the drift analysis offline: both camera
# streams + calibration, the IMU, cuVSLAM's odometry and tracking status, what
# the bridge sends PX4, and every /fmu/out topic PX4 is publishing.
#
#   record_bag.sh                       record until Ctrl-C
#   record_bag.sh --duration 60         record 60 s and stop
#   record_bag.sh --all                 every topic on the graph
#   record_bag.sh -o /bags/square-01    explicit output directory
#   record_bag.sh -- --compression-mode file --compression-format zstd
#
# Output goes to ${BAG_DIR:-/root/bags}/<timestamp> (host/run.sh bag mounts the
# repo's bags/ at /bags and sets BAGDIR, which is honoured too). Bind-mount it
# or the recording dies with the container.
set -euo pipefail

BAG_DIR="${BAG_DIR:-${BAGDIR:-/root/bags}}"
DURATION=""
RECORD_ALL=0
OUTPUT=""
EXTRA=()

 usage() { sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--duration) DURATION="${2:-}"; shift 2 ;;
    -a|--all)      RECORD_ALL=1; shift ;;
    -o|--output)   OUTPUT="${2:-}"; shift 2 ;;
    -h|--help)     usage; exit 0 ;;
    --)            shift; EXTRA=("$@"); break ;;
    *)             echo "record_bag.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "${DURATION}" && ! "${DURATION}" =~ ^[0-9]+$ ]]; then
  echo "record_bag.sh: --duration takes whole seconds, got '${DURATION}'" >&2
  exit 2
fi

# ROS + skybeta underlay + the team overlay (if built), so team-defined message
# types resolve. skybeta_env.sh handles the nounset dance the ROS setup scripts need.
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

BAG_PATH="${OUTPUT:-${BAG_DIR}/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$(dirname "${BAG_PATH}")"

# NB: rosbag2's own -d is --max-bag-duration (it SPLITS the bag every N
# seconds and keeps recording), so --duration is implemented below with a
# SIGINT timer instead -- SIGINT is rosbag2's clean shutdown path.
CMD=(ros2 bag record -s mcap -o "${BAG_PATH}")

if [[ ${RECORD_ALL} -eq 1 ]]; then
  CMD+=(-a)
  echo "[bag] recording ALL topics"
else
  # Fixed set first, then whatever PX4 is actually publishing. rosbag2 keeps
  # discovering, so a name that is not up yet is picked up when it appears.
  TOPICS=(
    /oak/left/image_raw
    /oak/right/image_raw
    /oak/left/camera_info
    /oak/right/camera_info
    /oak/imu/data
    /visual_slam/tracking/odometry
    /visual_slam/status
    /fmu/in/vehicle_visual_odometry
  )
  while IFS= read -r t; do
    if [[ -n "${t}" ]]; then TOPICS+=("${t}"); fi
  done < <(ros2 topic list 2>/dev/null | grep '^/fmu/out/' || true)

  echo "[bag] recording ${#TOPICS[@]} topics:"
  printf '        %s\n' "${TOPICS[@]}"
  CMD+=("${TOPICS[@]}")
fi

if [[ ${#EXTRA[@]} -gt 0 ]]; then
  CMD+=("${EXTRA[@]}")
fi

echo "[bag] ${CMD[*]}"
[[ -n "${DURATION}" ]] && echo "[bag] stopping after ${DURATION}s" || echo "[bag] Ctrl-C to stop"

# Ctrl-C is the documented way to stop an open-ended recording: catch it here
# so the summary below still prints, and do not report it as a failure. A
# trapped (not ignored) SIGINT is still delivered normally to `ros2 bag`.
trap ':' INT
rc=0
if [[ -n "${DURATION}" ]]; then
  "${CMD[@]}" &
  rec_pid=$!
  ( sleep "${DURATION}"; kill -INT "${rec_pid}" 2>/dev/null || true ) &
  timer_pid=$!
  wait "${rec_pid}" || rc=$?
  kill "${timer_pid}" 2>/dev/null || true
  wait "${timer_pid}" 2>/dev/null || true
else
  "${CMD[@]}" || rc=$?
fi
trap - INT
if [[ ${rc} -eq 130 || ${rc} -eq 2 ]]; then rc=0; fi   # SIGINT / rclpy KeyboardInterrupt

echo
if [[ -d "${BAG_PATH}" ]]; then
  echo "[bag] wrote ${BAG_PATH}  ($(du -sh "${BAG_PATH}" 2>/dev/null | cut -f1))"
  ls -1 "${BAG_PATH}" | sed 's/^/        /'
  echo "[bag] inspect with:  ros2 bag info ${BAG_PATH}"
else
  echo "[bag] no bag was written at ${BAG_PATH} (rc=${rc})" >&2
fi
exit "${rc}"
