#!/usr/bin/env bash
# Inside the skybeta container -- prove the uXRCE-DDS link is not just
# advertising topics but actually carrying data.
#
# A topic *name* appearing in `ros2 topic list` only proves the agent built the
# session. The bench gate is an --once echo returning a sample (CLAUDE.md #8).
#
# PX4 v1.16 versions its messages: a versioned topic gains a _v<N> suffix
# (/fmu/out/vehicle_status_v1), an unversioned one keeps the bare name. Nothing
# below is hard-coded -- each topic is resolved against the live topic list.
set -euo pipefail

TIMEOUT_S="${CHECK_TIMEOUT_S:-10}"

# ROS + skybeta underlay + the team overlay (if built), so team-defined message
# types resolve. skybeta_env.sh handles the nounset dance the ROS setup scripts need.
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

TOPICS="$(ros2 topic list 2>/dev/null || true)"
FMU_OUT="$(printf '%s\n' "${TOPICS}" | grep '^/fmu/out/' || true)"
FMU_ALL="$(printf '%s\n' "${TOPICS}" | grep '^/fmu/' || true)"
n_out="$(printf '%s' "${FMU_OUT}" | grep -c . || true)"
n_all="$(printf '%s' "${FMU_ALL}" | grep -c . || true)"

echo "[check] /fmu topics: ${n_all} total, ${n_out} under /fmu/out/"
if [[ "${n_out}" -gt 0 ]]; then
  printf '%s\n' "${FMU_OUT}" | sed 's/^/         /'
fi
echo

# Resolve a bare PX4 topic name against what is actually advertised: prefer the
# highest _v<N> variant, fall back to the unversioned name, and if neither is
# there hand back the bare name so the echo below reports a real FAIL.
resolve() {
  local base="$1" versioned
  versioned="$(printf '%s\n' "${FMU_OUT}" | grep -E "^${base}_v[0-9]+$" | sort -V | tail -n 1 || true)"
  if [[ -n "${versioned}" ]]; then printf '%s\n' "${versioned}"; return 0; fi
  printf '%s\n' "${base}"
}

fails=0
sample() {
  local topic="$1" out rc=0
  printf '[check] echo --once %-38s ' "${topic}"
  out="$(timeout "${TIMEOUT_S}" ros2 topic echo "${topic}" --once \
           --qos-reliability best_effort \
           --qos-durability transient_local 2>&1)" || rc=$?
  if [[ ${rc} -eq 0 && -n "${out}" ]]; then
    echo "PASS"
    printf '%s\n' "${out}" | head -n 6 | sed 's/^/           /'
  elif [[ ${rc} -eq 124 ]]; then
    echo "FAIL (no sample in ${TIMEOUT_S}s -- topic advertised but silent)"
    fails=$((fails + 1))
  else
    echo "FAIL (rc=${rc})"
    printf '%s\n' "${out}" | head -n 3 | sed 's/^/           /'
    fails=$((fails + 1))
  fi
}

sample "$(resolve /fmu/out/vehicle_status)"
sample "$(resolve /fmu/out/timesync_status)"
sample "$(resolve /fmu/out/vehicle_attitude)"

echo
if [[ ${fails} -eq 0 ]]; then
  echo "[check] PASS -- ${n_out} /fmu/out topics, all three sampled."
  echo "        Bench gate wants >= 50 /fmu topics; ${n_all} seen."
  exit 0
fi

cat >&2 <<EOM
[check] FAIL -- the link is not carrying data.

  1. Is run_agent.sh running, and did it print a session/client id?
  2. FC params: UXRCE_DDS_CFG=102, MAV_1_CONFIG=0, SER_TEL2_BAUD=921600,
     UXRCE_DDS_DOM_ID == ${ROS_DOMAIN_ID:-0} (ROS_DOMAIN_ID here, from drone.env).
     UXRCE_DDS_PTCFG == 2. Reboot the FC after changing them. A domain mismatch,
     or PTCFG=0 in local DDS scope, looks exactly like a dead link: the agent
     connects, and its topics land where nothing in this container can see them.
  3. The TELEM2 JST-GH connector on the dev unit is physically damaged
     (CLAUDE.md) -- a negative result here proves nothing until it is replaced.
  4. Topics present but silent => px4_msgs generation vs firmware mismatch
     (bug #8): the image is pinned to release/1.16 for PX4 v1.16.2.
EOM
exit 1
