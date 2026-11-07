#!/usr/bin/env bash
# Push the three skybeta tags built by host/build_image.sh to a registry.
#
#   REGISTRY=ghcr.io/<org> host/push_image.sh
#
# Pushes:
#   $REGISTRY/skybeta:<semver>
#   $REGISTRY/skybeta:<git-sha>
#   $REGISTRY/skybeta:<semver>-px4_<fw>-isaacros_<isaac>
#
# `latest` is never pushed. Drones pull a pinned tag (host/pull_image.sh) so a
# fleet cannot silently split across image versions mid-competition.
#
# Layer dedup: the NGC base (40 GB unpacked, 12.6 GB compressed) is a public image
# the drones pull straight from nvcr.io. The registry still stores the full image
# (~13 GB), but the per-drone transfer is only the ~0.6 GB compressed skybeta delta
# because the base layers are already on disk. Do NOT strip the base out of the
# push: the manifest needs it, and an offline drone gets the base from here.
# Any registry works; docs/distribution.md compares them.
#
# Log in first:
#   echo "$GHCR_PAT" | docker login ghcr.io -u <user> --password-stdin
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-skybeta}"
REGISTRY="${REGISTRY:-ghcr.io/YOUR-ORG}"
PX4_FIRMWARE="${PX4_FIRMWARE:-1.16.2}"
ISAAC_ROS_VERSION="${ISAAC_ROS_VERSION:-3.2}"

DRY_RUN=0

usage() {
  cat <<EOF
Usage: REGISTRY=ghcr.io/<org> host/push_image.sh [--dry-run] [-h|--help]

Tags and pushes the local ${IMAGE_NAME} build to \$REGISTRY. Requires that
host/build_image.sh has already produced the local tags.

Options:
  --dry-run    print the docker tag / docker push commands, do not run them
  -h, --help   this text

Environment:
  REGISTRY=${REGISTRY}
  IMAGE_NAME=${IMAGE_NAME}
  PX4_FIRMWARE=${PX4_FIRMWARE}   ISAAC_ROS_VERSION=${ISAAC_ROS_VERSION}
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)  DRY_RUN=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "push_image.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

command -v docker >/dev/null || { echo "push_image.sh: docker not on PATH" >&2; exit 1; }

if [[ "${REGISTRY}" == *YOUR-ORG* ]]; then
  echo "push_image.sh: set REGISTRY, e.g. REGISTRY=ghcr.io/myorg host/push_image.sh" >&2
  exit 2
fi
REGISTRY="${REGISTRY%/}"

[[ -f "${REPO_ROOT}/VERSION" ]] \
  || { echo "push_image.sh: missing ${REPO_ROOT}/VERSION" >&2; exit 1; }
VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION")"
[[ -n "${VERSION}" ]] || { echo "push_image.sh: VERSION is empty" >&2; exit 1; }

# Everything is pushed from the semver tag's image id, so all three remote tags
# are guaranteed to be the same image even if a local tag drifted.
SRC="${IMAGE_NAME}:${VERSION}"
if ! docker image inspect "${SRC}" >/dev/null 2>&1; then
  echo "push_image.sh: local image ${SRC} not found — run host/build_image.sh first" >&2
  exit 1
fi
IMAGE_ID="$(docker image inspect "${SRC}" --format '{{.Id}}')"
echo "[push] source ${SRC} (${IMAGE_ID})"

# The git-sha tag names the commit the image was BUILT from, not whatever is
# checked out now (the checkout moves on; a param or doc commit does not rebuild
# the image). Source, in order: the io.skybeta.git-sha label build_image.sh
# stamps, then the <sha> tag build_image.sh put on the same image id locally.
# If neither exists the sha tag is skipped rather than guessed.
GIT_SHA="$(docker image inspect "${SRC}" --format '{{index .Config.Labels "io.skybeta.git-sha"}}' 2>/dev/null || true)"
[[ "${GIT_SHA}" == "<no value>" ]] && GIT_SHA=""
if [[ -z "${GIT_SHA}" ]]; then
  # (bash regex, not awk: Ubuntu's default mawk has no {12} intervals)
  while read -r tag id; do
    if [[ "${IMAGE_ID#sha256:}" == "${id}"* && "${tag}" =~ ^[0-9a-f]{12}(-dirty)?$ ]]; then
      GIT_SHA="${tag}"; break
    fi
  done < <(docker image ls "${IMAGE_NAME}" --format '{{.Tag}} {{.ID}}' 2>/dev/null)
fi

TAGS=("${VERSION}")
if [[ -n "${GIT_SHA}" ]]; then
  echo "[push] built from commit ${GIT_SHA}"
  TAGS+=("${GIT_SHA}")
else
  echo "[push] WARNING: cannot tell which commit ${SRC} was built from (no io.skybeta.git-sha" >&2
  echo "[push]          label, no local <sha> tag) -- not pushing a git-sha tag" >&2
fi
TAGS+=("${VERSION}-px4_${PX4_FIRMWARE}-isaacros_${ISAAC_ROS_VERSION}")

run() {
  if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "[push] DRY-RUN: $*"
  else
    echo "[push] \$ $*"
    "$@"
  fi
}

for t in "${TAGS[@]}"; do
  remote="${REGISTRY}/${IMAGE_NAME}:${t}"
  run docker tag "${SRC}" "${remote}"
done

for t in "${TAGS[@]}"; do
  remote="${REGISTRY}/${IMAGE_NAME}:${t}"
  run docker push "${remote}"
done

if [[ ${DRY_RUN} -eq 0 ]]; then
  echo
  echo "[push] pushed:"
  for t in "${TAGS[@]}"; do echo "         ${REGISTRY}/${IMAGE_NAME}:${t}"; done
  echo
  echo "[push] on a drone:"
  echo "         REGISTRY=${REGISTRY} host/pull_image.sh ${VERSION}"
fi
