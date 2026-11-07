#!/usr/bin/env bash
# Unit test for the extlinux.conf FDT rewrite in host/jetson_uart_dma_fix.sh
# (bug #2: every `apply` used to append one more duplicate FDT line -- the dev
# unit had accumulated four).
#
# Runs entirely on a copy in a temp dir. No root, no /boot writes, no reboot.
#
#   bash host/tests/test_extlinux_fix.sh [path-to-extlinux.conf]
#
# Default input is the live /boot/extlinux/extlinux.conf, which is world
# readable. If it is missing (CI, x86 box) a synthetic four-FDT fixture with
# the same shape is used instead.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIX="${HERE}/../jetson_uart_dma_fix.sh"
DTB="/boot/dtb/uart_dma_fix.dtb"
SRC="${1:-/boot/extlinux/extlinux.conf}"

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

pass=0
fail=0
ok()   { printf '  ok   %s\n' "$*"; pass=$((pass + 1)); }
bad()  { printf '  FAIL %s\n' "$*"; fail=$((fail + 1)); }
check() { if [[ "$2" == "$3" ]]; then ok "$1 ($2)"; else bad "$1: expected '$3', got '$2'"; fi; }

# --- fixture ----------------------------------------------------------------
if [[ -r "${SRC}" ]]; then
  cp "${SRC}" "${TMP}/extlinux.conf"
  echo "[test] fixture: ${SRC}"
else
  echo "[test] ${SRC} unreadable -- using synthetic 4x-FDT fixture"
  cat > "${TMP}/extlinux.conf" <<'FIXTURE'
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      FDT /boot/dtb/uart_dma_fix.dtb
      FDT /boot/dtb/uart_dma_fix.dtb
      FDT /boot/dtb/uart_dma_fix.dtb
      FDT /boot/dtb/uart_dma_fix.dtb
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=deadbeef rw rootwait

# LABEL backup
#    MENU LABEL backup kernel
#    LINUX /boot/Image.backup
#    INITRD /boot/initrd
#    APPEND ${cbootargs}
FIXTURE
fi

cp "${TMP}/extlinux.conf" "${TMP}/original"
before=$(grep -cE '^[[:space:]]*FDT[[:space:]]' "${TMP}/original" || true)
echo "[test] FDT lines before: ${before}"

# --- pass 1 -----------------------------------------------------------------
echo "[test] pass 1"
bash "${FIX}" rewrite-extlinux "${TMP}/extlinux.conf" "${DTB}" | sed 's/^/  /'
cp "${TMP}/extlinux.conf" "${TMP}/after1"

check "exactly one FDT line" \
  "$(grep -cE '^[[:space:]]*FDT[[:space:]]' "${TMP}/after1" || true)" "1"

check "FDT points at the patched dtb" \
  "$(grep -E '^[[:space:]]*FDT[[:space:]]' "${TMP}/after1" | sed 's/^[[:space:]]*//')" \
  "FDT ${DTB}"

# FDT must sit on the line immediately after LINUX, inside LABEL primary.
linux_no=$(grep -nE '^[[:space:]]*LINUX[[:space:]]' "${TMP}/after1" | head -1 | cut -d: -f1)
fdt_no=$(grep -nE '^[[:space:]]*FDT[[:space:]]'   "${TMP}/after1" | head -1 | cut -d: -f1)
check "FDT is the line right after LINUX" "${fdt_no}" "$((linux_no + 1))"

check "FDT is indented with six spaces" \
  "$(grep -E '^      FDT ' "${TMP}/after1" | wc -l | tr -d ' ')" "1"

# Every non-FDT line must survive byte-for-byte, in order.
grep -vE '^[[:space:]]*FDT[[:space:]]' "${TMP}/original" > "${TMP}/orig.nofdt"
grep -vE '^[[:space:]]*FDT[[:space:]]' "${TMP}/after1"   > "${TMP}/after1.nofdt"
if diff -u "${TMP}/orig.nofdt" "${TMP}/after1.nofdt" > "${TMP}/nofdt.diff"; then
  ok "all non-FDT lines unchanged"
else
  bad "non-FDT lines changed:"
  sed 's/^/       /' "${TMP}/nofdt.diff"
fi

# --- pass 2 (idempotency) ----------------------------------------------------
echo "[test] pass 2 (idempotency)"
bash "${FIX}" rewrite-extlinux "${TMP}/extlinux.conf" "${DTB}" | sed 's/^/  /'
cp "${TMP}/extlinux.conf" "${TMP}/after2"

if diff -u "${TMP}/after1" "${TMP}/after2" > "${TMP}/idem.diff"; then
  ok "second run is a byte-for-byte no-op"
else
  bad "second run changed the file:"
  sed 's/^/       /' "${TMP}/idem.diff"
fi

check "still exactly one FDT line after two runs" \
  "$(grep -cE '^[[:space:]]*FDT[[:space:]]' "${TMP}/after2" || true)" "1"

# --- a file with no FDT at all should gain exactly one -----------------------
echo "[test] pass 3 (file with no FDT line)"
grep -vE '^[[:space:]]*FDT[[:space:]]' "${TMP}/original" > "${TMP}/nofdt.conf"
bash "${FIX}" rewrite-extlinux "${TMP}/nofdt.conf" "${DTB}" | sed 's/^/  /'
check "one FDT inserted into a file that had none" \
  "$(grep -cE '^[[:space:]]*FDT[[:space:]]' "${TMP}/nofdt.conf" || true)" "1"

# --- result ------------------------------------------------------------------
echo
printf '[test] %d passed, %d failed\n' "${pass}" "${fail}"
[[ "${fail}" -eq 0 ]] || exit 1
echo "[test] PASS"
exit 0
