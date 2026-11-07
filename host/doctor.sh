#!/usr/bin/env bash
# skybeta host health check.
#
# Every check here runs UNPRIVILEGED. On this fleet `sudo` prompts for a
# password and `dmesg` is restricted (kernel.dmesg_restrict=1), so nothing may
# depend on either -- the UART/IOMMU state is read from /proc/device-tree and
# /sys instead. See CLAUDE.md, "Verified facts about this Jetson".
#
# Usage:
#   host/doctor.sh                 # all checks except the camera boot
#   host/doctor.sh --boot-camera   # also start a container and open the OAK-D
#   host/doctor.sh --no-uart       # skip the 3 s serial sniff
#   host/doctor.sh --json          # one JSON object on stdout instead of the table
#                                  # (schema 1: date host image version args checks[]
#                                  #  {name result detail} pass warn fail ok)
#
# Exit status: 0 if no check FAILed (WARNs do not fail), 1 otherwise.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"

DEV="${DEV:-/dev/ttyTHS1}"
BAUD="${XRCE_BAUD:-921600}"
SNIFF_S="${SNIFF_S:-3}"
TAG="$(cat "${REPO}/VERSION" 2>/dev/null | tr -d '[:space:]')"
[[ -n "${TAG}" ]] || TAG="latest"
IMAGE="${IMAGE:-skybeta:${TAG}}"

BOOT_CAMERA=0
DO_UART=1
JSON=0
for arg in "$@"; do
  case "${arg}" in
    --boot-camera) BOOT_CAMERA=1 ;;
    --no-uart)     DO_UART=0 ;;
    --json)        JSON=1 ;;
    -h|--help)     sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown flag: ${arg}" >&2; exit 2 ;;
  esac
done

n_pass=0; n_warn=0; n_fail=0
C_NAME=(); C_RES=(); C_DET=()
record() { C_RES+=("$1"); C_NAME+=("$2"); C_DET+=("${3-}"); }
show() { [[ "${JSON}" -eq 1 ]] || printf "  \033[$1m%s\033[0m  %-26s %s\n" "$2" "$3" "${4-}"; }
pass() { record PASS "$@"; n_pass=$((n_pass+1)); show 32 PASS "$@"; }
warn() { record WARN "$@"; n_warn=$((n_warn+1)); show 33 WARN "$@"; }
fail() { record FAIL "$@"; n_fail=$((n_fail+1)); show 31 FAIL "$@"; }

# A JSON string literal. C0 control characters other than \t \n \r are dropped.
json_str() {
  local s
  s="$(printf '%s' "$1" | tr -d '\000-\010\013\014\016-\037')"
  s="${s//\\/\\\\}"
  s="${s//\"/\\\"}"
  s="${s//$'\t'/\\t}"
  s="${s//$'\n'/\\n}"
  s="${s//$'\r'/\\r}"
  printf '"%s"' "${s}"
}

RUN_DATE="$(date -Is)"
RUN_ARGS=("$@")
if [[ "${JSON}" -eq 0 ]]; then
  echo "skybeta doctor -- ${RUN_DATE}  host=$(hostname)  image=${IMAGE}"
  echo
fi

# 1 -- L4T release --------------------------------------------------------
REL_FILE=/etc/nv_tegra_release
if [[ ! -r "${REL_FILE}" ]]; then
  fail "l4t-release" "${REL_FILE} missing -- this is not a JetPack rootfs"
else
  REL_LINE="$(head -1 "${REL_FILE}")"
  REV="$(sed -n 's/.*REVISION: \([0-9.]*\).*/\1/p' <<<"${REL_LINE}")"
  if [[ "${REL_LINE}" == *"R36"* ]]; then
    pass "l4t-release" "R36, REVISION ${REV:-?}"
  else
    fail "l4t-release" "expected R36.x, got: ${REL_LINE}"
  fi
fi

# 2 -- UART DMA/IOMMU device-tree patch is live ---------------------------
DT="/proc/device-tree/bus@0/serial@3100000"
if [[ ! -d "${DT}" ]]; then
  fail "uart-devicetree" "${DT} absent -- no UARTA node in the live DT"
else
  missing=()
  for prop in iommus dmas dma-names; do
    [[ -e "${DT}/${prop}" ]] || missing+=("${prop}")
  done
  if [[ ${#missing[@]} -eq 0 ]]; then
    pass "uart-devicetree" "iommus + dmas + dma-names present"
  else
    fail "uart-devicetree" "missing ${missing[*]} -- run host/provision.sh --from uart_patch, then reboot"
  fi
fi

# 3 -- the platform device is bound and owns ttyTHS1 ----------------------
PDEV=/sys/bus/platform/devices/3100000.serial
if [[ ! -d "${PDEV}" ]]; then
  fail "uart-driver-bind" "${PDEV} absent"
else
  drv="$(basename "$(readlink -f "${PDEV}/driver" 2>/dev/null)" 2>/dev/null)"
  tty="$(ls "${PDEV}/tty" 2>/dev/null | tr '\n' ' ' | sed 's/ $//')"
  grp="$(basename "$(readlink -f "${PDEV}/iommu_group" 2>/dev/null)" 2>/dev/null)"
  if [[ "${drv}" == "serial-tegra" && "${tty}" == *ttyTHS1* ]]; then
    pass "uart-driver-bind" "driver=serial-tegra tty=${tty} iommu_group=${grp:-none}"
  else
    fail "uart-driver-bind" "driver='${drv:-none}' tty='${tty:-none}' (want serial-tegra / ttyTHS1)"
  fi
fi

# 4 -- exactly one FDT line in extlinux.conf (bug #2) ---------------------
EXTLINUX=/boot/extlinux/extlinux.conf
if [[ ! -r "${EXTLINUX}" ]]; then
  warn "extlinux-fdt" "${EXTLINUX} unreadable"
else
  n_fdt="$(grep -cE '^[[:space:]]*FDT[[:space:]]' "${EXTLINUX}")"
  fdt_val="$(grep -E '^[[:space:]]*FDT[[:space:]]' "${EXTLINUX}" | head -1 | awk '{print $2}')"
  if [[ "${n_fdt}" -eq 1 ]]; then
    pass "extlinux-fdt" "1 FDT line -> ${fdt_val}"
  elif [[ "${n_fdt}" -eq 0 ]]; then
    warn "extlinux-fdt" "no FDT line -- booting the stock DTB, not ${fdt_val:-the patched one}"
  else
    warn "extlinux-fdt" "${n_fdt} duplicate FDT lines (bug #2) -- fix: sudo host/jetson_uart_dma_fix.sh apply  (rewrites extlinux.conf only, no reboot; preview with --dry-run)"
  fi
fi

# 5 -- group membership (no sudo needed for UART or docker) ---------------
GROUPS_NOW="$(id -nG)"
missing=()
for g in dialout docker; do
  [[ " ${GROUPS_NOW} " == *" ${g} "* ]] || missing+=("${g}")
done
if [[ ${#missing[@]} -eq 0 ]]; then
  pass "user-groups" "$(id -un) is in dialout and docker"
else
  fail "user-groups" "$(id -un) missing group(s): ${missing[*]} -- sudo usermod -aG ${missing[*]// /,} \$USER, then re-login"
fi

# 6 -- docker nvidia runtime ----------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
  fail "docker-nvidia-runtime" "docker not installed"
elif ! docker info >/dev/null 2>&1; then
  fail "docker-nvidia-runtime" "cannot talk to the docker daemon (group not active in this shell?)"
else
  runtimes="$(docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null)"
  default_rt="$(docker info --format '{{.DefaultRuntime}}' 2>/dev/null)"
  if [[ " ${runtimes} " == *" nvidia "* ]]; then
    pass "docker-nvidia-runtime" "registered (default=${default_rt}; every run passes --runtime nvidia explicitly)"
  else
    fail "docker-nvidia-runtime" "nvidia runtime not registered; have: ${runtimes:-none}"
  fi
fi

# 7 -- OAK-D ---------------------------------------------------------------
# The MyriadX enumerates twice: 03e7:2485 @ 480 Mbps unbooted, then 03e7:f63b
# @ 5000 Mbps once depthai has pushed firmware. A pre-boot lsusb reading says
# nothing about the link speed -- see CLAUDE.md, "Hardware status".
if ! command -v lsusb >/dev/null 2>&1; then
  warn "oakd-usb" "lsusb not installed (apt install usbutils)"
else
  usbline="$(lsusb | grep -i '03e7:' | head -1)"
  if [[ -z "${usbline}" ]]; then
    fail "oakd-usb" "no 03e7:* device -- OAK-D not enumerated; check the USB3 cable/hub"
  elif [[ "${usbline}" == *"03e7:f63b"* ]]; then
    pass "oakd-usb" "booted: ${usbline#*ID }"
  else
    pass "oakd-usb" "unbooted: ${usbline#*ID } (normal; re-enumerates as 03e7:f63b once depthai loads firmware)"
  fi
fi

if [[ "${BOOT_CAMERA}" -eq 1 ]]; then
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    warn "oakd-usb-speed" "image ${IMAGE} absent -- cannot boot the camera"
  else
    speed="$(docker run --rm --privileged --runtime nvidia \
               -v /dev/bus/usb:/dev/bus/usb "${IMAGE}" \
               python3 -c 'import depthai as dai
with dai.Device() as d:
    print("USBSPEED=" + str(d.getUsbSpeed()).split(".")[-1])' 2>/dev/null \
             | sed -n 's/^USBSPEED=//p' | tail -1)"
    case "${speed}" in
      SUPER|SUPER_PLUS) pass "oakd-usb-speed" "getUsbSpeed() -> ${speed}" ;;
      HIGH)             warn "oakd-usb-speed" "getUsbSpeed() -> HIGH (USB 2.0). Move the camera to the USB3 hub; a pre-boot lsusb at 480 Mbps is NOT evidence of this -- the device re-enumerates on boot" ;;
      "")               warn "oakd-usb-speed" "could not open the device (busy? another container or the WP-camera agent may hold it)" ;;
      *)                warn "oakd-usb-speed" "getUsbSpeed() -> ${speed}" ;;
    esac
  fi
fi

# 8 -- UART traffic from the flight controller ----------------------------
# A container that passes ${DEV} through is running the agent: sniffing the port
# now would steal its bytes and corrupt the live session. With autostart on
# (host/autostart.sh) the stack always holds it, so ask the stack instead.
uart_holder=""
if command -v docker >/dev/null 2>&1; then
  for c in $(docker ps -q 2>/dev/null); do
    devs="$(docker inspect -f '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$c" 2>/dev/null || true)"
    if [[ " ${devs} " == *" ${DEV} "* ]]; then
      uart_holder="$(docker inspect -f '{{.Name}}' "$c" 2>/dev/null | sed 's|^/||')"
      break
    fi
  done
fi
if [[ "${DO_UART}" -eq 0 ]]; then
  warn "uart-traffic" "skipped (--no-uart)"
elif [[ -n "${uart_holder}" ]]; then
  warn "uart-traffic" "skipped: container '${uart_holder}' holds ${DEV} (the agent is using it; sniffing would corrupt its session). Check the link from inside: docker exec ${uart_holder} bash -lc 'source skybeta_env.sh; check_topics.sh'"
elif [[ ! -e "${DEV}" ]]; then
  fail "uart-traffic" "${DEV} does not exist"
elif [[ ! -r "${DEV}" ]]; then
  fail "uart-traffic" "${DEV} not readable by $(id -un) (dialout group not active in this shell?)"
elif ! python3 -c 'import serial' 2>/dev/null; then
  warn "uart-traffic" "pyserial not installed (pip3 install pyserial) -- cannot sniff ${DEV}"
else
  sniff="$(python3 - "${DEV}" "${BAUD}" "${SNIFF_S}" <<'PY' 2>/dev/null
import sys, time, serial
dev, baud, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
try:
    s = serial.Serial(dev, baud, timeout=0.2)
except Exception as e:
    print("ERR", e); raise SystemExit(0)
s.reset_input_buffer()
buf = bytearray(); t0 = time.time()
while time.time() - t0 < secs:
    buf += s.read(4096)
s.close()
print("N", len(buf), "XRCE", int(0x7E in buf), "MAVLINK", int(0xFD in buf),
      "HEAD", buf[:16].hex(" "))
PY
)"
  if [[ "${sniff}" == ERR* ]]; then
    fail "uart-traffic" "cannot open ${DEV}: ${sniff#ERR }"
  else
    nbytes="$(awk '{print $2}' <<<"${sniff}")"
    is_xrce="$(awk '{print $4}' <<<"${sniff}")"
    is_mav="$(awk '{print $6}' <<<"${sniff}")"
    if [[ "${nbytes:-0}" -eq 0 ]]; then
      fail "uart-traffic" "0 bytes in ${SNIFF_S}s @ ${BAUD} on ${DEV}. (1) If the link worked earlier in this FC power cycle and the stack/agent has since been stopped: PX4 v1.16.2 TX DMA wedge (CLAUDE.md bug #15) -- only an FC power cycle fixes it, a reseat does not. (2) Otherwise: the TELEM2 JST-GH 1.25 mm 6-pin connector/cable (on the reference airframe it is damaged and does not seat; CLAUDE.md, Hardware status), FC power, TX/RX swapped"
    elif [[ "${is_xrce}" == "1" ]]; then
      pass "uart-traffic" "${nbytes} bytes @ ${BAUD}, 0x7E framing -> uXRCE-DDS client is talking"
    elif [[ "${is_mav}" == "1" ]]; then
      warn "uart-traffic" "${nbytes} bytes @ ${BAUD}, 0xFD framing -> TELEM2 is still on MAVLink. Set MAV_1_CONFIG=0 and UXRCE_DDS_CFG=102, then reboot the FC (CLAUDE.md, State B)"
    else
      warn "uart-traffic" "${nbytes} bytes but neither 0x7E nor 0xFD seen -- baud mismatch? (${sniff#*HEAD })"
    fi
  fi
fi

# 8b -- boot-time stack (host/autostart.sh) -----------------------------------
# Informational: autostart is a per-drone choice, so "not installed" is a PASS.
AUTOSTART_UNIT="${SKYBETA_UNIT_DIR:-/etc/systemd/system}/skybeta-stack.service"
if [[ ! -f "${AUTOSTART_UNIT}" ]]; then
  pass "stack-autostart" "not installed -- the stack starts by hand (host/run.sh stack)"
else
  as_en="$(systemctl is-enabled skybeta-stack.service 2>/dev/null || true)"
  as_act="$(systemctl is-active skybeta-stack.service 2>/dev/null || true)"
  if [[ "${as_en}" == "enabled" && "${as_act}" == "active" ]]; then
    pass "stack-autostart" "enabled, running (attach: docker exec -it skybeta-stack tmux attach -t skybeta)"
  elif [[ "${as_en}" == "enabled" ]]; then
    warn "stack-autostart" "enabled but ${as_act:-not active} -- journalctl -u skybeta-stack -n 50"
  else
    warn "stack-autostart" "unit installed but ${as_en:-not enabled} -- sudo host/autostart.sh enable (or disable to remove it)"
  fi
fi

# 9 -- container image present ---------------------------------------------
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  warn "image-present" "docker unavailable -- skipped"
elif docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  digest="$(docker image inspect --format '{{.Id}}' "${IMAGE}" 2>/dev/null | cut -c1-19)"
  pass "image-present" "${IMAGE} (${digest})"
else
  others="$(docker image ls --format '{{.Repository}}:{{.Tag}}' 2>/dev/null | grep '^skybeta:' | tr '\n' ' ')"
  fail "image-present" "${IMAGE} not present -- run host/provision.sh --from pull (or host/pull_image.sh).${others:+ Present locally: ${others}}"
fi

# 10 -- power mode ----------------------------------------------------------
PM_STATUS=/var/lib/nvpmodel/status
PM_CONF="$(cat /var/lib/nvpmodel/conf_file_path 2>/dev/null || echo /etc/nvpmodel.conf)"
if [[ ! -r "${PM_STATUS}" ]]; then
  warn "power-mode" "${PM_STATUS} unreadable -- cannot confirm the power model"
else
  pm_id="$(sed -n 's/.*pmode:0*\([0-9]\+\).*/\1/p' "${PM_STATUS}")"
  [[ -n "${pm_id}" ]] || pm_id="$(tr -dc '0-9' < "${PM_STATUS}" | sed 's/^0*//')"
  pm_name="$(grep -oE "< POWER_MODEL ID=${pm_id:-x} NAME=[^ ]+" "${PM_CONF}" 2>/dev/null | head -1 | sed 's/.*NAME=//')"
  if [[ "${pm_name}" == MAXN* ]]; then
    pass "power-mode" "id=${pm_id} ${pm_name}"
  elif [[ -z "${pm_name}" ]]; then
    warn "power-mode" "id=${pm_id:-?} (name not found in ${PM_CONF})"
  else
    warn "power-mode" "id=${pm_id} ${pm_name} -- not MAXN. A low power profile produces exactly the frame jitter in CLAUDE.md #3b. Fix: host/provision.sh --from power"
  fi
fi

# 11 -- per-drone identity: /etc/skybeta/drone.env ---------------------------
# Without it every drone sits on DDS domain 0, and on a shared network every
# agent can see every other drone's /fmu/in/* setpoints.
# shellcheck source=host/drone_env.sh
source "${HERE}/drone_env.sh"
skybeta_load_drone_env
ID_FILE="${SKYBETA_DRONE_ENV:-/etc/skybeta/drone.env}"
scope="${SKYBETA_DDS_SCOPE:-local}"
if [[ -z "${SKYBETA_DRONE_ENV_FILE}" ]]; then
  warn "drone-identity" "no ${ID_FILE} -- anonymous drone on domain ${ROS_DOMAIN_ID:-0}. Fine alone on a bench; required before a second drone joins the network (host/drone.env.example)"
elif ! derr="$(skybeta_check_domain)"; then
  fail "drone-identity" "${ID_FILE}: ${derr}"
elif [[ "${scope}" != "local" && "${scope}" != "network" ]]; then
  fail "drone-identity" "${ID_FILE}: SKYBETA_DDS_SCOPE='${scope}' -- expected local or network"
elif [[ "${ROS_DOMAIN_ID:-0}" == "0" ]]; then
  warn "drone-identity" "drone ${DRONE_ID:-?} is on ROS_DOMAIN_ID 0, the shared default. Set ROS_DOMAIN_ID=${DRONE_ID:-<n>} in ${ID_FILE} and UXRCE_DDS_DOM_ID to match on the FC"
else
  id_msg="drone ${DRONE_ID:-?} team ${TEAM:-?} domain ${ROS_DOMAIN_ID} DDS ${scope} -- FC UXRCE_DDS_DOM_ID must be ${ROS_DOMAIN_ID}"
  [[ -n "${SKYBETA_DRONE_ENV_IGNORED}" ]] && id_msg+="; ignored unknown keys: ${SKYBETA_DRONE_ENV_IGNORED}"
  if [[ "${scope}" == "network" ]]; then
    warn "drone-identity" "${id_msg}. Scope 'network' puts the camera streams on the WiFi -- debugging only"
  else
    pass "drone-identity" "${id_msg}"
  fi
fi

# 12 -- team workspace ---------------------------------------------------------
TEAM_WS="${TEAM_WS:-${HOME}/team_ws}"
if [[ ! -d "${TEAM_WS}" ]]; then
  warn "team-ws" "${TEAM_WS} does not exist yet (host/run.sh creates it on first start)"
else
  n_pkg="$(find "${TEAM_WS}/src" -name package.xml 2>/dev/null | wc -l | tr -d ' ')"
  overlay="not built"
  [[ -f "${TEAM_WS}/install/local_setup.bash" ]] && overlay="built"
  # The container runs as root; anything it created that team_build.sh has not
  # chowned back is uneditable from the host shell.
  foreign="$(find "${TEAM_WS}" ! -user "$(id -u)" -print -quit 2>/dev/null)"
  team_img=""
  if [[ -f "${TEAM_WS}/Dockerfile" ]]; then
    if docker image inspect "skybeta-team:${TAG}" >/dev/null 2>&1; then
      team_img=", team image skybeta-team:${TAG}"
    else
      team_img=", team image NOT built for ${TAG} (host/run.sh build)"
    fi
  fi
  if [[ -n "${foreign}" ]]; then
    warn "team-ws" "${TEAM_WS} has files not owned by $(id -un), e.g. ${foreign#"${TEAM_WS}/"} -- created as root in a container. Fix: sudo chown -R $(id -un): ${TEAM_WS}"
  elif [[ "${team_img}" == *"NOT built"* ]]; then
    warn "team-ws" "${TEAM_WS}: ${n_pkg} package(s), overlay ${overlay}${team_img}"
  else
    pass "team-ws" "${TEAM_WS}: ${n_pkg} package(s), overlay ${overlay}${team_img}"
  fi
fi

# --------------------------------------------------------------------------
if [[ "${JSON}" -eq 1 ]]; then
  args_json=""
  for a in ${RUN_ARGS[@]+"${RUN_ARGS[@]}"}; do
    args_json+="${args_json:+,}$(json_str "${a}")"
  done
  checks_json=""
  for i in "${!C_NAME[@]}"; do
    checks_json+="${checks_json:+,}"$'\n  '
    checks_json+="{\"name\":$(json_str "${C_NAME[$i]}"),\"result\":\"${C_RES[$i]}\",\"detail\":$(json_str "${C_DET[$i]}")}"
  done
  ok=true; [[ "${n_fail}" -eq 0 ]] || ok=false
  printf '{"schema":1,"date":%s,"host":%s,"image":%s,"version":%s,"args":[%s],\n"checks":[%s\n],\n"pass":%d,"warn":%d,"fail":%d,"ok":%s}\n' \
    "$(json_str "${RUN_DATE}")" "$(json_str "$(hostname)")" "$(json_str "${IMAGE}")" \
    "$(json_str "${TAG}")" "${args_json}" "${checks_json}" \
    "${n_pass}" "${n_warn}" "${n_fail}" "${ok}"
else
  echo
  printf 'doctor: %d PASS  %d WARN  %d FAIL\n' "${n_pass}" "${n_warn}" "${n_fail}"
fi
[[ "${n_fail}" -eq 0 ]] || exit 1
exit 0
