#!/usr/bin/env bash
# Pull a pinned skybeta image onto a drone.
#
#   REGISTRY=ghcr.io/<org> host/pull_image.sh 0.1.0
#
# Order matters. The NGC Isaac ROS base is ~40 GB and public; the skybeta delta
# on top of it is ~2.3 GB (measured on skybeta:0.1.0). Pulling the base FIRST from nvcr.io means the
# subsequent registry pull finds every base layer already on disk and transfers
# only the delta. Pull skybeta first and you drag the whole 40 GB across the
# venue network, per drone.
#
# `latest` is refused on purpose: 10+ drones must run a byte-identical image,
# and a moving tag is how a fleet silently splits mid-competition.
#
# LAN mirror (venue with bad uplink): stand up a pull-through cache and set
#   MIRROR=<host:5000>
# Both refs are then fetched through $MIRROR and re-tagged to their canonical
# names locally, so nothing downstream has to know the mirror exists. The mirror
# must proxy (or hold a copy of) both nvcr.io and $REGISTRY — a single
# `registry:2` in proxy mode only fronts one upstream, so either run two, or
# push copies into a plain registry:2 with the same repository paths:
#   docker run -d -p 5000:5000 --restart=always --name mirror registry:2
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-skybeta}"
REGISTRY="${REGISTRY:-ghcr.io/YOUR-ORG}"
MIRROR="${MIRROR:-}"

# Must match the FROM line in docker/Dockerfile.jetson.
BASE_IMAGE="${BASE_IMAGE:-nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c}"

SKIP_BASE=0

usage() {
  cat <<EOF
Usage: REGISTRY=ghcr.io/<org> host/pull_image.sh <tag> [--skip-base] [-h|--help]

Pulls the NGC base first, then \$REGISTRY/${IMAGE_NAME}:<tag>, so only the
skybeta delta crosses the network.

Arguments:
  <tag>          pinned tag, e.g. 0.1.0 or 0.1.0-px4_1.16.2-isaacros_3.2 or a
                 git sha. 'latest' is refused.

Options:
  --skip-base    do not pull the NGC base (it is already on this Jetson)
  -h, --help     this text

Environment:
  REGISTRY=${REGISTRY}
  IMAGE_NAME=${IMAGE_NAME}
  MIRROR=${MIRROR:-<unset>}   LAN pull-through cache, e.g. 192.168.1.10:5000
  BASE_IMAGE=${BASE_IMAGE}
EOF
}

TAG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-base) SKIP_BASE=1; shift ;;
    -h|--help)   usage; exit 0 ;;
    -*) echo "pull_image.sh: unknown option '$1'" >&2; usage >&2; exit 2 ;;
    *)  if [[ -n "${TAG}" ]]; then
          echo "pull_image.sh: unexpected extra argument '$1'" >&2; exit 2
        fi
        TAG="$1"; shift ;;
  esac
done

command -v docker >/dev/null || { echo "pull_image.sh: docker not on PATH" >&2; exit 1; }

if [[ -z "${TAG}" ]]; then
  echo "pull_image.sh: a tag is required" >&2
  usage >&2
  exit 2
fi
if [[ "${TAG}" == "latest" ]]; then
  echo "pull_image.sh: refusing 'latest' — pull a pinned tag (0.1.0, a git sha, ...)" >&2
  exit 2
fi

if [[ "${REGISTRY}" == *YOUR-ORG* ]]; then
  echo "pull_image.sh: set REGISTRY, e.g. REGISTRY=ghcr.io/myorg host/pull_image.sh ${TAG}" >&2
  exit 2
fi
REGISTRY="${REGISTRY%/}"
MIRROR="${MIRROR%/}"

# Rewrite a fully qualified ref onto $MIRROR, dropping the upstream registry
# host. nvcr.io/nvidia/isaac/ros:t -> $MIRROR/nvidia/isaac/ros:t
via_mirror() {
  local ref="$1"
  [[ -z "${MIRROR}" ]] && { printf '%s' "${ref}"; return; }
  printf '%s/%s' "${MIRROR}" "${ref#*/}"
}

# Pull $2 (possibly through the mirror) and make sure the canonical ref $1 ends
# up on this machine, so `docker run` and the compat tag both keep working.
pull_as() {
  local canonical="$1" source_ref="$2"
  if docker image inspect "${canonical}" >/dev/null 2>&1; then
    echo "[pull] already present: ${canonical}"
    return 0
  fi
  echo "[pull] docker pull ${source_ref}"
  docker pull "${source_ref}" || return 1   # explicit: set -e is off when called from an if
  if [[ "${source_ref}" != "${canonical}" ]]; then
    docker tag "${source_ref}" "${canonical}" || return 1
    echo "[pull] retagged ${source_ref} -> ${canonical}"
  fi
}

if [[ ${SKIP_BASE} -eq 1 ]]; then
  echo "[pull] skipping NGC base (--skip-base)"
else
  echo "[pull] step 1/2: NGC base (public, ~40 GB unpacked / ~12.6 GB transferred — cached after the first drone)"
  # Not fatal: the skybeta image in $REGISTRY carries every base layer too (push_image.sh
  # pushes the full manifest), so a drone with no route to nvcr.io -- an offline venue --
  # still gets everything from $REGISTRY in step 2, just all of it from there.
  if ! pull_as "${BASE_IMAGE}" "$(via_mirror "${BASE_IMAGE}")"; then
    echo "[pull] WARNING: could not pull the NGC base; step 2 will fetch its layers from ${MIRROR:-${REGISTRY}} instead" >&2
  fi
fi

REMOTE="${REGISTRY}/${IMAGE_NAME}:${TAG}"
echo "[pull] step 2/2: skybeta delta"
pull_as "${REMOTE}" "$(via_mirror "${REMOTE}")"

# Local convenience tag so host/run.sh finds it by the VERSION-derived
# reference. Deliberately NOT 'latest'.
docker tag "${REMOTE}" "${IMAGE_NAME}:${TAG}"

echo
echo "[pull] done."
docker image inspect "${REMOTE}" --format \
  '[pull] image id  {{.Id}}
[pull] size      {{.Size}} bytes
[pull] labels    {{json .Config.Labels}}' 2>/dev/null || true
echo
echo "[pull] record this digest in the fleet log:"
docker image inspect "${REMOTE}" --format '{{range .RepoDigests}}         {{.}}
{{end}}' 2>/dev/null || true
echo "[pull] run it with:  IMAGE=${IMAGE_NAME}:${TAG} host/run.sh stack"
