# shellcheck shell=bash
# Per-drone identity -- sourced by host/run.sh and host/doctor.sh, never executed.
#
# Every drone carries one file, /etc/skybeta/drone.env (override the path with
# SKYBETA_DRONE_ENV), written once at provisioning time. host/drone.env.example
# documents the keys. It is what makes ten identical images behave as ten
# separate drones on one WiFi network: above all, the DDS domain.
#
# The file is PARSED, not sourced. Only the whitelisted KEY=VALUE lines below are
# read, values are taken literally (no $(...), no $VAR expansion), and a key is
# exported only when the caller has not already set it. Precedence is therefore
#
#     positional args  >  caller environment  >  drone.env  >  built-in defaults
#
# which is the order host/run.sh's existing ${1:-${CAMERA_X:-0.0}} pattern expects.

SKYBETA_DRONE_ENV_KEYS=(
  DRONE_ID TEAM
  ROS_DOMAIN_ID SKYBETA_DDS_SCOPE
  CAMERA_X CAMERA_Y CAMERA_Z CAMERA_ROLL CAMERA_PITCH CAMERA_YAW
  COMPETITOR_PKG COMPETITOR_LAUNCH
  TEAM_WS
)

# Highest domain ID whose RTPS ports (7400 + 250*domain + offsets) stay below
# Linux's ephemeral port range, which starts at 32768. ROS 2 documents the same
# 0-101 limit.
SKYBETA_DOMAIN_MAX=101

_skybeta_is_drone_env_key() {
  local k
  for k in "${SKYBETA_DRONE_ENV_KEYS[@]}"; do [[ "$k" == "$1" ]] && return 0; done
  return 1
}

# skybeta_load_drone_env [file]
#   Sets SKYBETA_DRONE_ENV_FILE to the file actually read ("" when none) and
#   SKYBETA_DRONE_ENV_IGNORED to any keys it skipped as unknown. Always returns 0:
#   a missing file is the caller's decision to warn about, not an error here.
skybeta_load_drone_env() {
  local file="${1:-${SKYBETA_DRONE_ENV:-/etc/skybeta/drone.env}}"
  local line key value
  SKYBETA_DRONE_ENV_FILE=""
  SKYBETA_DRONE_ENV_IGNORED=""
  [[ -r "${file}" ]] || return 0
  # shellcheck disable=SC2034  # read by the scripts that source this library
  SKYBETA_DRONE_ENV_FILE="${file}"

  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%$'\r'}"
    # strip leading whitespace and an optional `export `
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line#export }"
    [[ -z "${line}" || "${line}" == \#* ]] && continue
    [[ "${line}" == *=* ]] || continue
    key="${line%%=*}"
    value="${line#*=}"
    # trailing whitespace off the value, then one layer of matching quotes
    value="${value%"${value##*[![:space:]]}"}"
    if [[ "${value}" =~ ^\"(.*)\"$ || "${value}" =~ ^\'(.*)\'$ ]]; then
      value="${BASH_REMATCH[1]}"
    fi
    if ! _skybeta_is_drone_env_key "${key}"; then
      SKYBETA_DRONE_ENV_IGNORED+="${SKYBETA_DRONE_ENV_IGNORED:+ }${key}"
      continue
    fi
    # `~/` is the only expansion offered, for TEAM_WS=~/team_ws. The quotes are
    # deliberate: match a literal tilde in the value.
    # shellcheck disable=SC2088
    if [[ "${value}" == "~/"* ]]; then
      value="${HOME}/${value#"~/"}"
    fi
    # Caller already set it (even to empty): the caller wins.
    if [[ -n "${!key+x}" ]]; then
      continue
    fi
    printf -v "${key}" '%s' "${value}"
    export "${key?}"
  done < "${file}"
  return 0
}

# skybeta_check_domain
#   Validates ROS_DOMAIN_ID (unset counts as 0). Prints a one-line reason and
#   returns 1 when it is unusable; returns 0 otherwise.
skybeta_check_domain() {
  local d="${ROS_DOMAIN_ID:-0}"
  if [[ ! "${d}" =~ ^[0-9]+$ ]]; then
    echo "ROS_DOMAIN_ID='${d}' is not a non-negative integer"
    return 1
  fi
  if (( 10#${d} > SKYBETA_DOMAIN_MAX )); then
    echo "ROS_DOMAIN_ID=${d} is above ${SKYBETA_DOMAIN_MAX}; DDS ports would collide with the ephemeral range"
    return 1
  fi
  return 0
}
