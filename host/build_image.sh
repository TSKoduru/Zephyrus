#!/usr/bin/env bash
# Build the skybeta Jetson image from docker/Dockerfile.jetson.
#
# Three tags, all pointing at the same image id:
#   skybeta:<semver>                              <- from the repo-root VERSION file
#   skybeta:<git-sha>                             <- exactly what was in the tree
#   skybeta:<semver>-px4_<fw>-isaacros_<isaac>    <- human-readable compatibility tag
#
# `latest` is NOT written unless you pass --tag-latest, and a drone must never
# pull `latest` — see host/pull_image.sh.
#
# First build pulls the ~40 GB NGC base if it is not cached, then ~30-40 min of
# apt + colcon on an Orin Nano. Subsequent builds are layer-cached.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-skybeta}"
DOCKERFILE="${DOCKERFILE:-docker/Dockerfile.jetson}"

# Compatibility coordinates baked into the third tag and into image labels.
PX4_FIRMWARE="${PX4_FIRMWARE:-1.16.2}"
ISAAC_ROS_VERSION="${ISAAC_ROS_VERSION:-3.2}"

# Build args. Defaults live in the Dockerfile; override here to pin a fleet
# build to exact SHAs or a different ROS snapshot.
SNAPSHOT_DATE="${SNAPSHOT_DATE:-}"
PX4_MSGS_REF="${PX4_MSGS_REF:-}"
PX4_ROS2_LIB_REF="${PX4_ROS2_LIB_REF:-}"
XRCE_AGENT_TAG="${XRCE_AGENT_TAG:-}"

TAG_LATEST=0
NO_CACHE=0

usage() {
  cat <<EOF
Usage: host/build_image.sh [--tag-latest] [--no-cache] [-h|--help]

Builds ${IMAGE_NAME} from ${DOCKERFILE} with the repo root as build context.

Options:
  --tag-latest   also tag the result ${IMAGE_NAME}:latest (do this only after
                 the acceptance checks pass — drones pull a pinned tag)
  --no-cache     docker build --no-cache
  -h, --help     this text

Environment overrides:
  IMAGE_NAME=${IMAGE_NAME}
  DOCKERFILE=${DOCKERFILE}
  PX4_FIRMWARE=${PX4_FIRMWARE}          ISAAC_ROS_VERSION=${ISAAC_ROS_VERSION}
  SNAPSHOT_DATE=       ROS apt snapshot date (default: Dockerfile ARG)
  PX4_MSGS_REF=        px4_msgs branch or SHA (default: Dockerfile ARG)
  PX4_ROS2_LIB_REF=    px4-ros2-interface-lib branch or SHA
  XRCE_AGENT_TAG=      Micro-XRCE-DDS-Agent tag

Version comes from ${REPO_ROOT}/VERSION.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag-latest) TAG_LATEST=1; shift ;;
    --no-cache)   NO_CACHE=1;   shift ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "build_image.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

command -v docker >/dev/null || { echo "build_image.sh: docker not on PATH" >&2; exit 1; }

[[ -f "${REPO_ROOT}/VERSION" ]] \
  || { echo "build_image.sh: missing ${REPO_ROOT}/VERSION" >&2; exit 1; }
VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION")"
[[ -n "${VERSION}" ]] || { echo "build_image.sh: VERSION is empty" >&2; exit 1; }

[[ -f "${REPO_ROOT}/${DOCKERFILE}" ]] \
  || { echo "build_image.sh: missing ${REPO_ROOT}/${DOCKERFILE}" >&2; exit 1; }

# Git sha, with -dirty when the tree does not match HEAD. A dirty tag is a
# warning, not an error: it says "this image is not reproducible from a commit".
GIT_SHA="nogit"
if git -C "${REPO_ROOT}" rev-parse --git-dir >/dev/null 2>&1; then
  GIT_SHA="$(git -C "${REPO_ROOT}" rev-parse --short=12 HEAD)"
  if ! git -C "${REPO_ROOT}" diff --quiet HEAD -- 2>/dev/null; then
    GIT_SHA="${GIT_SHA}-dirty"
    echo "[build] WARNING: working tree is dirty; tagging ${GIT_SHA}"
  fi
fi

TAG_SEMVER="${IMAGE_NAME}:${VERSION}"
TAG_GITSHA="${IMAGE_NAME}:${GIT_SHA}"
TAG_COMPAT="${IMAGE_NAME}:${VERSION}-px4_${PX4_FIRMWARE}-isaacros_${ISAAC_ROS_VERSION}"

BUILD_ARGS=()
[[ -n "${SNAPSHOT_DATE}"    ]] && BUILD_ARGS+=(--build-arg "SNAPSHOT_DATE=${SNAPSHOT_DATE}")
[[ -n "${PX4_MSGS_REF}"     ]] && BUILD_ARGS+=(--build-arg "PX4_MSGS_REF=${PX4_MSGS_REF}")
[[ -n "${PX4_ROS2_LIB_REF}" ]] && BUILD_ARGS+=(--build-arg "PX4_ROS2_LIB_REF=${PX4_ROS2_LIB_REF}")
[[ -n "${XRCE_AGENT_TAG}"   ]] && BUILD_ARGS+=(--build-arg "XRCE_AGENT_TAG=${XRCE_AGENT_TAG}")

CACHE_ARGS=()
[[ ${NO_CACHE} -eq 1 ]] && CACHE_ARGS+=(--no-cache)

echo "[build] repo      ${REPO_ROOT}"
echo "[build] dockerfile ${DOCKERFILE}"
echo "[build] tags      ${TAG_SEMVER}"
echo "[build]           ${TAG_GITSHA}"
echo "[build]           ${TAG_COMPAT}"

docker build \
  -f "${REPO_ROOT}/${DOCKERFILE}" \
  -t "${TAG_SEMVER}" \
  -t "${TAG_GITSHA}" \
  -t "${TAG_COMPAT}" \
  --label "io.skybeta.git-sha=${GIT_SHA}" \
  "${BUILD_ARGS[@]+"${BUILD_ARGS[@]}"}" \
  "${CACHE_ARGS[@]+"${CACHE_ARGS[@]}"}" \
  "${REPO_ROOT}"

if [[ ${TAG_LATEST} -eq 1 ]]; then
  docker tag "${TAG_SEMVER}" "${IMAGE_NAME}:latest"
  echo "[build] also tagged ${IMAGE_NAME}:latest"
fi

echo
echo "[build] done."
docker image inspect "${TAG_SEMVER}" \
  --format '[build] image id   {{.Id}}
[build] size       {{.Size}} bytes' 2>/dev/null || true
docker images "${IMAGE_NAME}" --format 'table {{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.ID}}'
echo
echo "[build] next: host/push_image.sh   (registry)   or   host/run.sh stack"
