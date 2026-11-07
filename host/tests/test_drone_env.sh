#!/usr/bin/env bash
# Unit test for host/drone_env.sh -- the /etc/skybeta/drone.env parser that gives
# every drone its own DDS domain, camera pose and team workspace.
#
# Pure bash on temp files. No root, no docker, no Jetson.
#
#   bash host/tests/test_drone_env.sh
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="${HERE}/../drone_env.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

pass=0
fail=0
ok()   { printf '  ok   %s\n' "$*"; pass=$((pass + 1)); }
bad()  { printf '  FAIL %s\n' "$*"; fail=$((fail + 1)); }
check() { if [[ "$2" == "$3" ]]; then ok "$1 ($2)"; else bad "$1: expected '$3', got '$2'"; fi; }

# Each case runs in a fresh `env -i` bash so nothing leaks between cases or in
# from the caller's environment. Prints KEY=VALUE lines for the asserts.
load() {   # load <file> [VAR=value ...]   (VARs are pre-set in the caller env)
  local file="$1"; shift
  env -i HOME="${TMP}/home" PATH="${PATH}" "$@" bash -c '
    source "$1"
    skybeta_load_drone_env "$2"
    for k in DRONE_ID TEAM ROS_DOMAIN_ID SKYBETA_DDS_SCOPE CAMERA_X CAMERA_PITCH \
             TEAM_WS COMPETITOR_PKG EVIL; do
      printf "%s=%s\n" "$k" "${!k-<unset>}"
    done
    printf "FILE=%s\n" "${SKYBETA_DRONE_ENV_FILE}"
    printf "IGNORED=%s\n" "${SKYBETA_DRONE_ENV_IGNORED}"
    # exported, not just set: docker -e NAME needs it in the environment
    printf "EXPORTED_DOMAIN=%s\n" "$(bash -c "echo \${ROS_DOMAIN_ID-<unset>}")"
  ' _ "${LIB}" "${file}"
}
get() { sed -n "s/^$1=//p" <<<"$2"; }

# --- fixtures -------------------------------------------------------------------
cat > "${TMP}/drone.env" <<'EOF'
# a comment
   # an indented comment

DRONE_ID=3
TEAM="Team Falcon"
export ROS_DOMAIN_ID=3
SKYBETA_DDS_SCOPE='local'
CAMERA_X=0.10
CAMERA_PITCH=0.35
TEAM_WS=~/team_ws
COMPETITOR_PKG=$(touch /tmp/skybeta_pwned)falcon_nav
EVIL=rm -rf /
not a key value line
EOF
printf 'DRONE_ID=4\r\nROS_DOMAIN_ID=4\r\n' > "${TMP}/crlf.env"

echo "[test] basic parse"
out="$(load "${TMP}/drone.env")"
check "DRONE_ID"                    "$(get DRONE_ID "${out}")"          "3"
check "double quotes stripped"      "$(get TEAM "${out}")"              "Team Falcon"
check "export prefix accepted"      "$(get ROS_DOMAIN_ID "${out}")"     "3"
check "single quotes stripped"      "$(get SKYBETA_DDS_SCOPE "${out}")" "local"
check "trailing whitespace trimmed" "$(get CAMERA_X "${out}")"          "0.10"
# shellcheck disable=SC2088  # a label, not a path
check "~/ expands to HOME"         "$(get TEAM_WS "${out}")"           "${TMP}/home/team_ws"
check "value is exported"           "$(get EXPORTED_DOMAIN "${out}")"   "3"
check "FILE records the path"       "$(get FILE "${out}")"              "${TMP}/drone.env"

echo "[test] the file is data, not code"
check "\$(...) kept literal"  "$(get COMPETITOR_PKG "${out}")" '$(touch /tmp/skybeta_pwned)falcon_nav'
if [[ -e /tmp/skybeta_pwned ]]; then bad "command substitution executed"; rm -f /tmp/skybeta_pwned
else ok "command substitution not executed"; fi
check "unknown key not set"   "$(get EVIL "${out}")"    "<unset>"
check "unknown key reported"  "$(get IGNORED "${out}")" "EVIL"

echo "[test] precedence: caller environment beats the file"
out="$(load "${TMP}/drone.env" ROS_DOMAIN_ID=9 CAMERA_PITCH=)"
check "env ROS_DOMAIN_ID wins"            "$(get ROS_DOMAIN_ID "${out}")" "9"
check "env set-but-empty still wins"      "$(get CAMERA_PITCH "${out}")"  ""
check "keys not in env still come from file" "$(get DRONE_ID "${out}")"   "3"

echo "[test] CRLF file (edited on Windows)"
out="$(load "${TMP}/crlf.env")"
check "CR stripped" "$(get ROS_DOMAIN_ID "${out}")" "4"

echo "[test] missing file"
out="$(load "${TMP}/nope.env")"
check "FILE empty"        "$(get FILE "${out}")"          ""
check "nothing set"       "$(get ROS_DOMAIN_ID "${out}")" "<unset>"

echo "[test] skybeta_check_domain"
domain() { env -i PATH="${PATH}" ${1:+ROS_DOMAIN_ID="$1"} bash -c 'source "$0"; skybeta_check_domain >/dev/null && echo ok || echo bad' "${LIB}"; }
check "unset -> ok (0)"  "$(domain '')"    "ok"
check "0 ok"             "$(domain 0)"     "ok"
check "101 ok"           "$(domain 101)"   "ok"
check "102 rejected"     "$(domain 102)"   "bad"
check "-1 rejected"      "$(domain -1)"    "bad"
check "text rejected"    "$(domain three)" "bad"
check "008 ok (decimal, not octal)" "$(domain 008)" "ok"

echo "[test] the Fast DDS profiles are well-formed"
# A malformed profile is silently ignored by Fast DDS, which then puts DDS on the
# WiFi. `--` inside an XML comment is the easy way to break one (done twice).
if command -v python3 >/dev/null 2>&1 && python3 -c 'print()' >/dev/null 2>&1; then PY=python3; else PY=python; fi
for xml in "${HERE}"/../dds/*.xml; do
  if "${PY}" -c 'import sys, xml.etree.ElementTree as E; E.parse(sys.argv[1])' "${xml}" 2>/dev/null; then
    ok "$(basename "${xml}") parses"
  else
    bad "$(basename "${xml}") does not parse"
  fi
done
for scope in local network; do
  if grep -q 'profile_name="px4_participant"' "${HERE}/../dds/agent_refs_${scope}.xml" 2>/dev/null; then
    ok "agent_refs_${scope}.xml defines px4_participant (the name PX4 asks for with PTCFG=2)"
  else
    bad "agent_refs_${scope}.xml missing or lacks profile px4_participant"
  fi
done

echo
echo "[test] ${pass} passed, ${fail} failed"
[[ "${fail}" -eq 0 ]] || exit 1
exit 0
