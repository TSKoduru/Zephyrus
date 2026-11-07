#!/usr/bin/env bash
# Tests for the boot-time stack: host/autostart.sh render, the unit template, and
# host/run.sh's detached (SKYBETA_DETACH=1) docker invocation. No root, no real
# docker: a shim on PATH records what run.sh would have run.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

n_ok=0; n_fail=0
ok()   { printf '  ok   %s\n' "$1"; n_ok=$((n_ok+1)); }
bad()  { printf '  FAIL %s\n' "$1"; n_fail=$((n_fail+1)); }
check() { if eval "$2"; then ok "$1"; else bad "$1"; fi; }

# --- render ------------------------------------------------------------------
unit="$(SKYBETA_STACK_USER=pilot bash "${REPO}/host/autostart.sh" render)"
check "render exits 0"                         '[[ -n "${unit}" ]]'
check "no @PLACEHOLDER@ left"                  '! grep -q "@[A-Z]*@" <<<"${unit}"'
check "User= is the requested user"            'grep -qx "User=pilot" <<<"${unit}"'
check "ExecStart runs this repo's service"     'grep -qx "ExecStart=/bin/bash ${REPO}/host/stack_service.sh" <<<"${unit}"'
check "WorkingDirectory is this repo"          'grep -qx "WorkingDirectory=${REPO}" <<<"${unit}"'
check "restarts on failure"                    'grep -qx "Restart=on-failure" <<<"${unit}"'
check "no start-rate limit"                    'grep -qx "StartLimitIntervalSec=0" <<<"${unit}"'
check "ordered after docker"                   'grep -qx "After=docker.service" <<<"${unit}" && grep -qx "Requires=docker.service" <<<"${unit}"'
check "wanted by multi-user.target"            'grep -qx "WantedBy=multi-user.target" <<<"${unit}"'

if command -v systemd-analyze >/dev/null 2>&1; then
  printf '%s\n' "${unit}" > "${TMP}/skybeta-stack.service"
  # verify warns about the not-yet-installed paths only if they are missing;
  # the repo paths exist, so any output naming the unit is a real problem.
  va="$(systemd-analyze verify "${TMP}/skybeta-stack.service" 2>&1 | grep 'skybeta-stack.service' \
        | grep -v 'Unit docker.service not found' || true)"   # a laptop without docker
  check "systemd-analyze verify is clean"      '[[ -z "${va}" ]]'
  [[ -z "${va}" ]] || printf '%s\n' "${va}" | sed 's/^/       /'
else
  ok "systemd-analyze not installed -- verify skipped"
fi

out="$(bash "${REPO}/host/autostart.sh" enable 2>&1)"; rc=$?
check "enable without root refuses"            '[[ ${rc} -ne 0 ]] && grep -q "sudo host/autostart.sh enable" <<<"${out}"'
out="$(bash "${REPO}/host/autostart.sh" frobnicate 2>&1)"; rc=$?
check "unknown command fails"                  '[[ ${rc} -ne 0 ]]'
out="$(SKYBETA_UNIT_DIR="${TMP}/none" bash "${REPO}/host/autostart.sh" status 2>&1)"
check "status without the unit says so"        'grep -q "not installed" <<<"${out}"'

# --- run.sh SKYBETA_DETACH=1 ---------------------------------------------------
mkdir -p "${TMP}/bin"
cat > "${TMP}/bin/docker" <<EOF
#!/usr/bin/env bash
echo "\$*" >> "${TMP}/docker.log"
case "\$1" in
  run)   echo deadbeef ;;
  ps)    : ;;
  *)     : ;;
esac
exit 0
EOF
chmod +x "${TMP}/bin/docker"
cat > "${TMP}/drone.env" <<'EOF'
DRONE_ID=16
ROS_DOMAIN_ID=16
SKYBETA_DDS_SCOPE=local
CAMERA_X=0.1053
EOF
out="$(PATH="${TMP}/bin:${PATH}" SKYBETA_DRONE_ENV="${TMP}/drone.env" TEAM_WS="${TMP}/team_ws" \
       SKYBETA_DETACH=1 DEV="${TMP}/ttyFAKE" bash "${REPO}/host/run.sh" stack </dev/null 2>&1)"; rc=$?
runline="$(grep '^run ' "${TMP}/docker.log" 2>/dev/null | tail -1)"
check "detached run.sh exits 0"                '[[ ${rc} -eq 0 ]]'
check "docker run was called"                  '[[ -n "${runline}" ]]'
check "uses -d -t"                             'grep -q -- "^run -d -t --rm " <<<"${runline}"'
check "never -i / -it (no host TTY at boot)"   '! grep -qE -- " -it? " <<<" ${runline} "'
check "docker log is capped"                   'grep -q -- "--log-opt max-size=1m" <<<"${runline}"'
check "container named skybeta-stack"          'grep -q -- "--name skybeta-stack" <<<"${runline}"'
check "runs run_full_stack.sh"                 'grep -q "run_full_stack.sh$" <<<"${runline}"'
check "drone.env reaches the container"        'grep -q -- "-e ROS_DOMAIN_ID=16" <<<"${runline}" && grep -q -- "-e CAMERA_X=0.1053" <<<"${runline}"'
check "prints how to attach"                   'grep -q "tmux attach -t skybeta" <<<"${out}"'

: > "${TMP}/docker.log"
out="$(PATH="${TMP}/bin:${PATH}" SKYBETA_DRONE_ENV="${TMP}/drone.env" TEAM_WS="${TMP}/team_ws" \
       DEV="${TMP}/ttyFAKE" bash -c 'exec "$0" stack' "${REPO}/host/run.sh" </dev/null 2>&1)"
runline="$(grep '^run ' "${TMP}/docker.log" 2>/dev/null | tail -1)"
check "without SKYBETA_DETACH it stays -it"    'grep -q -- "^run -it --rm " <<<"${runline}"'

# --- stack_service.sh clock-step detection ----------------------------------------
eval "$(sed -n '/^clock_offset() {/,/^}/p' "${REPO}/host/stack_service.sh")"
o1="$(clock_offset)"; sleep 1; o2="$(clock_offset)"
check "clock offset steady without a step"     'awk -v a="${o1}" -v b="${o2}" "BEGIN { d = a - b; if (d < 0) d = -d; exit !(d < 0.1) }"'
check "service waits for NTP before starting"  'grep -q "NTPSynchronized" "${REPO}/host/stack_service.sh"'
check "service restarts on a clock step"       'grep -q "system clock stepped" "${REPO}/host/stack_service.sh"'

# --- stack_service.sh syntax -----------------------------------------------------
check "stack_service.sh parses"                'bash -n "${REPO}/host/stack_service.sh"'
check "autostart.sh parses"                    'bash -n "${REPO}/host/autostart.sh"'

echo
echo "[test] ${n_ok} passed, ${n_fail} failed"
[[ ${n_fail} -eq 0 ]]
