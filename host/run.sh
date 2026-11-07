#!/usr/bin/env bash
# skybeta container launcher -- replaces 04_run_container.sh and run_pipeline.sh.
#
#   host/run.sh stack [cx cy cz [roll pitch yaw]]   full stack in a tmux session
#   host/run.sh shell                               interactive bash (or exec into a running one)
#   host/run.sh bag [args...]                       rosbag2 recording (container_scripts/record_bag.sh)
#   host/run.sh build [--init]                      build the team image from $TEAM_WS/Dockerfile
#   host/run.sh sim                                 Isaac Sim profile -- not yet implemented
#
# Per-drone settings come from /etc/skybeta/drone.env (host/drone.env.example):
# DRONE_ID, TEAM, ROS_DOMAIN_ID, SKYBETA_DDS_SCOPE, CAMERA_*, TEAM_WS,
# COMPETITOR_PKG/LAUNCH. The caller's environment overrides the file.
#
# Camera pose is the base_link -> oak_parent_frame static TF: translation in
# metres, rotation in radians, FLU body frame. Positional args win over the
# CAMERA_* environment variables, which win over drone.env.
#
#   host/run.sh stack                          camera pose from drone.env (else body origin)
#   host/run.sh stack 0.10 0 -0.02             translation only
#   host/run.sh stack 0.10 0 -0.02 0 0.35 0    plus a 20 deg down-pitch
#   CAMERA_X=0.10 CAMERA_PITCH=0.35 host/run.sh stack
#
# SKYBETA_DETACH=1 starts the stack container detached (docker run -d -t: the
# container gets its own pty, so the tmux session inside still works) and returns.
# That is how the boot-time service runs it (host/stack_service.sh,
# host/autostart.sh); attach with: docker exec -it skybeta-stack tmux attach -t skybeta
#
# Team code lives on the host in $TEAM_WS (default ~/team_ws), bind-mounted at
# /root/team_ws, so it survives the container exiting and image updates. If
# $TEAM_WS/Dockerfile exists and `host/run.sh build` has built it, the team image
# skybeta-team:<VERSION> is used instead of skybeta:<VERSION>.
#
# Image tag comes from the VERSION file at the repo root (falling back to
# 'latest'); override the whole reference with IMAGE=ghcr.io/.../skybeta:0.1.0.
#
# No sudo. The `sudo systemctl stop nvgetty` and `sudo chmod 666 $DEV` lines the
# old scripts carried are no-ops on this image (nvgetty.service does not exist)
# and unnecessary for a user in `dialout` -- see CLAUDE.md.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"
DOCKERFILE="${REPO}/docker/Dockerfile.jetson"

TAG="$(cat "${REPO}/VERSION" 2>/dev/null | tr -d '[:space:]')"
[[ -n "${TAG}" ]] || TAG="latest"
DEV="${DEV:-/dev/ttyTHS1}"

log()  { printf '[run] %s\n' "$*"; }
warn() { printf '[run] WARNING: %s\n' "$*" >&2; }
die()  { printf '[run] ERROR: %s\n' "$*" >&2; exit 1; }
 usage() { sed -n '2,38p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

MODE="${1:-}"
[[ $# -gt 0 ]] && shift || true
case "${MODE}" in
  stack|shell|bag|build|sim) ;;
  -h|--help|'') usage; exit 0 ;;
  *) usage >&2; die "unknown mode: ${MODE}" ;;
esac

# --- sim is a stub for now -------------------------------------------------
if [[ "${MODE}" == "sim" ]]; then
  cat <<'EOM'
[run] sim: not yet implemented -- see sim/

  The Isaac Sim profile (Isaac Sim 5.1.0 + Pegasus v5.1.0 + PX4 SITL v1.16.2,
  x86_64 only) is WP7. It will live in sim/ and build from docker/Dockerfile.sim.
  This Jetson cannot run it: there is no aarch64 Isaac Sim.
EOM
  exit 0
fi

# --- per-drone identity ----------------------------------------------------
# shellcheck source=host/drone_env.sh
source "${HERE}/drone_env.sh"
skybeta_load_drone_env
if [[ -z "${SKYBETA_DRONE_ENV_FILE}" ]]; then
  warn "no drone identity file (${SKYBETA_DRONE_ENV:-/etc/skybeta/drone.env}) -- running as an anonymous drone on DDS domain ${ROS_DOMAIN_ID:-0}. Fine for one drone on a bench; on a shared network every drone needs its own. See host/drone.env.example."
elif [[ -n "${SKYBETA_DRONE_ENV_IGNORED}" ]]; then
  warn "${SKYBETA_DRONE_ENV_FILE}: ignored unknown key(s): ${SKYBETA_DRONE_ENV_IGNORED}"
fi
domain_err="$(skybeta_check_domain)" || die "${domain_err}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SKYBETA_DDS_SCOPE="${SKYBETA_DDS_SCOPE:-local}"
case "${SKYBETA_DDS_SCOPE}" in
  local|network) ;;
  *) die "SKYBETA_DDS_SCOPE='${SKYBETA_DDS_SCOPE}' -- expected 'local' or 'network'" ;;
esac
export ROS_DOMAIN_ID DRONE_ID="${DRONE_ID:-}" TEAM="${TEAM:-}"

# --- team workspace ----------------------------------------------------------
# Created here, as the invoking user, before docker can create it as root.
TEAM_WS="${TEAM_WS:-${HOME}/team_ws}"
mkdir -p "${TEAM_WS}/src" || die "cannot create team workspace ${TEAM_WS}/src"
TEAM_WS="$(cd "${TEAM_WS}" && pwd)"

BASE_IMAGE="skybeta:${TAG}"
TEAM_IMAGE="skybeta-team:${TAG}"
if [[ -n "${IMAGE:-}" ]]; then
  :   # explicit IMAGE= always wins
elif [[ "${MODE}" != "build" && -f "${TEAM_WS}/Dockerfile" ]] \
     && docker image inspect "${TEAM_IMAGE}" >/dev/null 2>&1; then
  IMAGE="${TEAM_IMAGE}"
else
  IMAGE="${BASE_IMAGE}"
  if [[ "${MODE}" != "build" && -f "${TEAM_WS}/Dockerfile" ]]; then
    warn "${TEAM_WS}/Dockerfile exists but ${TEAM_IMAGE} is not built -- using ${BASE_IMAGE}. Run: host/run.sh build"
  fi
fi

# --- build: the team image ----------------------------------------------------
# A team's extra apt/pip packages go in $TEAM_WS/Dockerfile, layered on the
# pinned base. Never `apt install` inside a running container: every container
# here is --rm, so it is gone on exit.
if [[ "${MODE}" == "build" ]]; then
  if [[ "${1:-}" == "--init" ]]; then
    if [[ -e "${TEAM_WS}/Dockerfile" ]]; then
      die "${TEAM_WS}/Dockerfile already exists -- not overwriting it"
    fi
    cat > "${TEAM_WS}/Dockerfile" <<'EOD'
# Team image: your extra system and Python packages, layered on the pinned
# skybeta base. Build with `host/run.sh build`; host/run.sh then uses it
# automatically. Your ROS code does NOT go in here -- it lives in src/ and is
# bind-mounted live at /root/team_ws/src.
ARG BASE
FROM ${BASE}

# System / ROS packages:
# RUN apt-get update \
#  && apt-get install -y --no-install-recommends ros-humble-<package> \
#  && rm -rf /var/lib/apt/lists/*

# Python packages:
# RUN pip3 install --no-cache-dir <package>
EOD
    if [[ ! -e "${TEAM_WS}/.dockerignore" ]]; then
      printf '%s\n' build/ install/ log/ src/ > "${TEAM_WS}/.dockerignore"
    fi
    log "wrote ${TEAM_WS}/Dockerfile and .dockerignore -- edit it, then: host/run.sh build"
    exit 0
  fi
  [[ -f "${TEAM_WS}/Dockerfile" ]] \
    || die "no ${TEAM_WS}/Dockerfile. Start one with: host/run.sh build --init"
  docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1 \
    || die "base image ${BASE_IMAGE} is not on this drone. Pull it first: host/pull_image.sh ${TAG}"
  base_id="$(docker image inspect --format '{{.Id}}' "${BASE_IMAGE}")"
  log "building ${TEAM_IMAGE} from ${TEAM_WS}/Dockerfile on ${BASE_IMAGE} (${base_id:0:19})"
  exec docker build \
    -f "${TEAM_WS}/Dockerfile" \
    --build-arg "BASE=${BASE_IMAGE}" \
    --label "io.skybeta.base=${BASE_IMAGE}" \
    --label "io.skybeta.base-id=${base_id}" \
    -t "${TEAM_IMAGE}" \
    "${TEAM_WS}"
fi

# --- image -----------------------------------------------------------------
build_hint() {
  if [[ -f "${DOCKERFILE}" ]]; then
    printf 'Build it locally:  host/build_image.sh   (docker build -f %s -t %s %s)' \
      "${DOCKERFILE#"${REPO}/"}" "${IMAGE}" "${REPO}"
  else
    printf '%s is missing too -- WP0 has not landed; there is nothing to build from.' "${DOCKERFILE}"
  fi
}

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  PULLER="${HERE}/pull_image.sh"
  # pull_image.sh takes the bare tag and re-tags the result to skybeta:<tag>
  # locally, which is exactly the reference ${IMAGE} defaults to.
  PULL_TAG="${IMAGE##*:}"
  if [[ ! -f "${PULLER}" ]]; then
    die "image '${IMAGE}' not present and ${PULLER} does not exist yet (WP0 owns it).
       $(build_hint)"
  elif [[ "${PULL_TAG}" == "latest" ]]; then
    die "image '${IMAGE}' not present, and pull_image.sh refuses 'latest' on purpose -- a
       fleet of 10+ drones must run a byte-identical, pinned tag. Put a version in
       ${REPO}/VERSION, or pass IMAGE=skybeta:<tag>.
       $(build_hint)"
  else
    log "image ${IMAGE} not present locally -- pulling tag ${PULL_TAG} via host/pull_image.sh"
    bash "${PULLER}" "${PULL_TAG}" || die "pull of tag '${PULL_TAG}' failed (is REGISTRY set?).
       $(build_hint)"
  fi
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || die "image '${IMAGE}' still not present after the pull"
fi

# --- one UART, one owner ---------------------------------------------------
# Two MicroXRCEAgents on the same serial port silently corrupt each other's
# framing, so refuse rather than let it happen. Checks the actual device
# passthrough of every running container, not just our own container names.
uart_holder() {
  local c devs
  for c in $(docker ps -q 2>/dev/null); do
    devs="$(docker inspect -f '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$c" 2>/dev/null || true)"
    if [[ " ${devs} " == *" ${DEV} "* ]]; then
      docker inspect -f '{{.Name}}' "$c" 2>/dev/null | sed 's|^/||'
      return 0
    fi
  done
  return 1
}

# --- shared docker run arguments ------------------------------------------
DEVICE_ARG=()
if [[ -e "${DEV}" ]]; then
  DEVICE_ARG=(--device "${DEV}:${DEV}")
else
  warn "${DEV} not present -- the uXRCE agent will have nothing to talk to. Did the UART patch reboot happen? (host/doctor.sh)"
fi

# X11 so rviz2 / GUI tools inside the container can render on the host display.
xhost +local:root >/dev/null 2>&1 || true
X11_ARG=()
if [[ -n "${DISPLAY:-}" ]]; then
  X11_ARG+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [[ -n "${XAUTHORITY:-}" ]] && X11_ARG+=(-v "${XAUTHORITY}:${XAUTHORITY}" -e "XAUTHORITY=${XAUTHORITY}")
fi

COMMON_ARG=(
  --privileged
  --runtime nvidia
  --network host
  --ipc host
  -v /dev:/dev
  -v /dev/bus/usb:/dev/bus/usb
)

# --- DDS isolation -----------------------------------------------------------
# --network host puts every container straight on the drone's WiFi. Two layers
# keep one drone's DDS traffic (including /fmu/in/* setpoints the agent forwards
# to the FC) away from every other drone:
#   1. its own ROS_DOMAIN_ID, matched by UXRCE_DDS_DOM_ID on its FC;
#   2. SKYBETA_DDS_SCOPE=local (default): every DDS participant on loopback.
#      Two files, because the agent does not take the default profile:
#        dds/fastdds_localhost.xml   default profile for the ROS nodes (SHM + UDP lo)
#        dds/agent_refs_local.xml    the agent's PX4 participant, built by
#                                    reference -- needs UXRCE_DDS_PTCFG=2 on the FC
#      Measured on the bench: with PTCFG=0 the agent builds PX4's own XML on
#      every interface, and the loopback-only ROS nodes never discover it.
DDS_PROFILE="${HERE}/dds/fastdds_localhost.xml"
AGENT_REFS="${HERE}/dds/agent_refs_${SKYBETA_DDS_SCOPE}.xml"

# Fast DDS rejects a malformed profile with nothing but a log line and carries
# on with its built-in transports -- straight onto the WiFi. Refuse instead.
xml_ok() {
  ! command -v python3 >/dev/null 2>&1 \
    || python3 -c 'import sys, xml.etree.ElementTree as E; E.parse(sys.argv[1])' "$1" 2>/dev/null
}
[[ -f "${AGENT_REFS}" ]] || die "${AGENT_REFS} missing -- the agent cannot build PX4's participant"
xml_ok "${AGENT_REFS}" || die "${AGENT_REFS} is not well-formed XML"

DDS_ARG=(
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  -e "SKYBETA_DDS_SCOPE=${SKYBETA_DDS_SCOPE}"
  -e DRONE_ID
  -e TEAM
  -v "${AGENT_REFS}:/etc/skybeta/agent_refs.xml:ro"
  -e XRCE_REFS=/etc/skybeta/agent_refs.xml
)
case "${SKYBETA_DDS_SCOPE}" in
  local)
    [[ -f "${DDS_PROFILE}" ]] || die "${DDS_PROFILE} missing -- cannot keep DDS on this drone"
    xml_ok "${DDS_PROFILE}" \
      || die "${DDS_PROFILE} is not well-formed XML -- Fast DDS would ignore it and put DDS on every interface"
    DDS_ARG+=(
      -v "${DDS_PROFILE}:/etc/skybeta/fastdds.xml:ro"
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml
      -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml
    )
    ;;
  network)
    warn "SKYBETA_DDS_SCOPE=network: DDS domain ${ROS_DOMAIN_ID} is on every interface, camera topics included. Debugging only."
    ;;
esac
if [[ "${ROS_DOMAIN_ID}" == "0" && -n "${DRONE_ID}" && "${DRONE_ID}" != "0" ]]; then
  warn "DRONE_ID=${DRONE_ID} but ROS_DOMAIN_ID=0 -- every drone left on 0 shares a domain. Set ROS_DOMAIN_ID=${DRONE_ID} in drone.env and UXRCE_DDS_DOM_ID=${DRONE_ID} on the FC."
fi

# --- team workspace mount ------------------------------------------------------
# HOST_UID/GID let team_build.sh hand files created as root back to the host user.
TEAM_ARG=(
  -v "${TEAM_WS}:/root/team_ws"
  -e "HOST_UID=$(id -u)"
  -e "HOST_GID=$(id -g)"
  -e COMPETITOR_PKG
  -e COMPETITOR_LAUNCH
)

# --- camera pose -----------------------------------------------------------
CAMERA_X="${1:-${CAMERA_X:-0.0}}"
CAMERA_Y="${2:-${CAMERA_Y:-0.0}}"
CAMERA_Z="${3:-${CAMERA_Z:-0.0}}"
CAMERA_ROLL="${4:-${CAMERA_ROLL:-0.0}}"
CAMERA_PITCH="${5:-${CAMERA_PITCH:-0.0}}"
CAMERA_YAW="${6:-${CAMERA_YAW:-0.0}}"

CAMERA_ENV=(
  -e "CAMERA_X=${CAMERA_X}"
  -e "CAMERA_Y=${CAMERA_Y}"
  -e "CAMERA_Z=${CAMERA_Z}"
  -e "CAMERA_ROLL=${CAMERA_ROLL}"
  -e "CAMERA_PITCH=${CAMERA_PITCH}"
  -e "CAMERA_YAW=${CAMERA_YAW}"
)

case "${MODE}" in

  # -----------------------------------------------------------------------
  shell)
    NAME="${NAME:-skybeta}"
    if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
      log "attaching a new shell to the running container '${NAME}'"
      exec docker exec -it "${NAME}" /bin/bash
    fi
    if holder="$(uart_holder)"; then
      # Usually the running stack. A second container cannot take the UART, but
      # a shell inside the holder sees the same DDS domain and team workspace.
      log "container '${holder}' holds ${DEV} -- attaching a new shell to it"
      exec docker exec -it "${holder}" /bin/bash
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    log "starting '${NAME}' from ${IMAGE} (interactive; drone ${DRONE_ID:-?}, domain ${ROS_DOMAIN_ID}, DDS ${SKYBETA_DDS_SCOPE})"
    exec docker run -it --rm \
      --name "${NAME}" \
      "${COMMON_ARG[@]}" \
      "${DEVICE_ARG[@]}" \
      "${X11_ARG[@]}" \
      "${DDS_ARG[@]}" \
      "${TEAM_ARG[@]}" \
      -e "XRCE_DEV=${DEV}" \
      "${CAMERA_ENV[@]}" \
      "${IMAGE}" \
      /bin/bash
    ;;

  # -----------------------------------------------------------------------
  stack)
    NAME="${NAME:-skybeta-stack}"
    if holder="$(uart_holder)"; then
      if [[ "${holder}" == "skybeta-stack" ]] \
         && systemctl is-active --quiet skybeta-stack.service 2>/dev/null; then
        die "the stack is already running -- started at boot by skybeta-stack.service (host/autostart.sh).
       Attach to it:   docker exec -it skybeta-stack tmux attach -t skybeta   (Ctrl-b d to detach)
       Restart it (picks up drone.env changes):   docker rm -f skybeta-stack   (the service restarts it in ~10 s)
       Stop it until next boot:   sudo systemctl stop skybeta-stack
       Turn autostart off:        sudo host/autostart.sh disable"
      fi
      die "container '${holder}' already holds ${DEV} -- two MicroXRCEAgents on one UART will corrupt each other. Stop it first: docker rm -f ${holder}"
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true

    cat <<EOM
[run] full stack
        drone        ${DRONE_ID:-<no drone.env>}   team ${TEAM:-?}
        image        ${IMAGE}
        container    ${NAME}
        DDS          domain ${ROS_DOMAIN_ID}, scope ${SKYBETA_DDS_SCOPE}
        team code    ${TEAM_WS} -> /root/team_ws   launches ${COMPETITOR_PKG:-competitor_example}
        UART         ${DEV} @ ${XRCE_BAUD:-921600}
        camera FLU   xyz = ${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m
                     rpy = ${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad

      Flight-controller prerequisites (QGroundControl, then reboot the FC):
        MAV_1_CONFIG=0   UXRCE_DDS_CFG=102 (TELEM2)   SER_TEL2_BAUD=921600
        UXRCE_DDS_DOM_ID=${ROS_DOMAIN_ID}   (must equal ROS_DOMAIN_ID on this Jetson)
        UXRCE_DDS_PTCFG=2    (agent builds PX4's participant from dds/agent_refs_${SKYBETA_DDS_SCOPE}.xml;
                              with 0, /fmu/* never reaches the stack in local scope)
        EKF2_EV_CTRL=9 (horiz pos + yaw)   EKF2_HGT_REF=2 (range)  EKF2_RNG_CTRL=1  EKF2_BARO_CTRL=1
        EKF2_EV_DELAY = measured, not assumed   EKF2_EVP/EVV_NOISE=0.1  EKF2_EVA_NOISE=0.05
      See CLAUDE.md "Target PX4 parameter sets" -- MAV_1_CONFIG and UXRCE_DDS_CFG
      must never both point at 102.

EOM
    STACK_ARG=(
      --name "${NAME}"
      "${COMMON_ARG[@]}"
      "${DEVICE_ARG[@]}"
      "${X11_ARG[@]}"
      "${DDS_ARG[@]}"
      "${TEAM_ARG[@]}"
      -e "XRCE_DEV=${DEV}"
      -e "XRCE_BAUD=${XRCE_BAUD:-921600}"
      "${CAMERA_ENV[@]}"
    )
    if [[ "${SKYBETA_DETACH:-0}" == "1" ]]; then
      # -t without -i: no host TTY needed, but run_full_stack.sh's final
      # `tmux attach` still gets a terminal (the container's own pty) and keeps
      # the container alive. Its redraws go to the docker log, hence the cap.
      docker run -d -t --rm \
        --log-opt max-size=1m --log-opt max-file=1 \
        "${STACK_ARG[@]}" \
        "${IMAGE}" \
        run_full_stack.sh >/dev/null || die "docker run of ${NAME} failed"
      log "stack container '${NAME}' started detached -- attach: docker exec -it ${NAME} tmux attach -t skybeta"
      exit 0
    fi
    exec docker run -it --rm \
      "${STACK_ARG[@]}" \
      "${IMAGE}" \
      run_full_stack.sh
    ;;

  # -----------------------------------------------------------------------
  bag)
    NAME="${NAME:-skybeta-bag}"
    BAGDIR="${BAGDIR:-${REPO}/bags}"
    mkdir -p "${BAGDIR}"
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    log "recording into ${BAGDIR} (mounted at /bags)"
    # No --device: recording joins the running stack over --network host DDS,
    # so it neither needs nor may take the UART. It must carry the stack's DDS
    # domain and scope, or it records an empty domain. Team workspace read-only,
    # so team-defined message types resolve.
    exec docker run -it --rm \
      --name "${NAME}" \
      "${COMMON_ARG[@]}" \
      "${DDS_ARG[@]}" \
      -v "${TEAM_WS}:/root/team_ws:ro" \
      -v "${BAGDIR}:/bags" \
      -e "BAGDIR=/bags" \
      "${IMAGE}" \
      bash -lc 'command -v record_bag.sh >/dev/null || {
                  echo "[run] record_bag.sh is not in this image." >&2
                  echo "      container_scripts/record_bag.sh is WP6 and has not landed yet." >&2
                  echo "      Meanwhile, from inside host/run.sh shell:" >&2
                  echo "        ros2 bag record -s mcap -o /bags/\$(date +%Y%m%d-%H%M%S) \\\\" >&2
                  echo "          /oak/left/image_raw /oak/right/image_raw /oak/imu/data \\\\" >&2
                  echo "          /visual_slam/tracking/odometry /fmu/in/vehicle_visual_odometry" >&2
                  exit 127; }
                exec record_bag.sh "$@"' _ "$@"
    ;;
esac
