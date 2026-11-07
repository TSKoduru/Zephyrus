#!/usr/bin/env bash
# Build PX4 v1.16.2 for Pixhawk 6C (px4_fmu-v6c_default) = the pinned release plus
# exactly ONE fix: NuttX stm32h7 "UART TX DMA getting stuck".
#
#   Fix upstream:   apache/nuttx PR #17203
#   PX4 backport:   PX4/NuttX PR #353, commit d50ffd354495ce5d30814dba41d2eb70a32b8fa3
#                   on branch px4_firmware_nuttx-10.3.0+
#   In PX4 main:    PX4-Autopilot PR #25791 (NOT in v1.16.2, whose NuttX pin is 886acbbdb4)
#   Not applied:    PX4-Autopilot PR #26848 or anything else; no defconfig edits.
#
# Symptom fixed: uxrce_dds_client closes/reopens TELEM2 (/dev/ttyS3 = UART5, TX DMA)
# mid-transfer; up_dma_shutdown() leaves dmatx.length/nlength and the stream NDTR
# non-zero, so the UART never transmits again until reboot.
#
# Usage:  ./build_fmu-v6c_v1.16.2-txdma.sh [WORKDIR]      (default: directory of this script)
#         PREPARE_ONLY=1 ./build_fmu-v6c_v1.16.2-txdma.sh  (source prep only, no docker/build)
#         NATIVE=1 ./build_fmu-v6c_v1.16.2-txdma.sh        (no docker: user-space toolchain, see below)
# Needs: git, curl, make, cmake, gcc, /usr/bin/python3; plus docker usable without sudo
# (default mode) or nothing else (NATIVE=1: everything goes into WORKDIR/tools, no sudo).
# Idempotent: reuses an existing clone if it is in the expected state, re-verifies everything.
set -euo pipefail

SCRIPT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")
WORK=${1:-$(dirname "$SCRIPT")}
SRC=$WORK/PX4-Autopilot
OUT=$WORK/out
TAG=v1.16.2
TARGET=px4_fmu-v6c_default
NUTTX_PIN=886acbbdb4f061e5c0ce1a76afbcfa7cb7df9849   # v1.16.2's NuttX submodule pin
FIX=d50ffd354495ce5d30814dba41d2eb70a32b8fa3         # PX4/NuttX PR #353
# Image Tools/docker_run.sh picks for px4_fmu* at v1.16.2 (GCC 9.3.1 20200408, same as stock).
IMAGE=px4io/px4-dev-nuttx-focal:2022-08-12
# Fixed identity + dates => the cherry-pick and superproject commit get the same SHAs on
# every run, so git_identity (v1.16.2-1-g<sha>) is reproducible.
export GIT_AUTHOR_NAME="skybeta build" GIT_AUTHOR_EMAIL="skybeta-build@localhost"
export GIT_COMMITTER_NAME="skybeta build" GIT_COMMITTER_EMAIL="skybeta-build@localhost"
export GIT_COMMITTER_DATE="2026-10-20T00:00:00+0000"
PREPARE_ONLY=${PREPARE_ONLY:-0}
NATIVE=${NATIVE:-0}
# NATIVE=1 toolchain: the compiler inside the docker image above / used by the stock release,
# "GNU Arm Embedded Toolchain 9-2020-q2-update 9.3.1 20200408 (release)".
TOOLS=$WORK/tools
GCC_TAR=gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
GCC_URLS="https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/$GCC_TAR
https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/$GCC_TAR"
GCC_MD5=2b9eeccc33470f9d3cda26983b9d2dc6     # published by ARM
GCC_SHA256=5adc2ee03904571c2de79d5cfc0f7fe2a5c5f54f44da5b645c17ee57b217f11f  # observed 2026-10-20
GCC_DIR=$TOOLS/gcc-arm-none-eabi-9-2020-q2-update
GENROMFS_REV=e4225b49a7be0ae9d39e98f2175dd674c0d6b1ea   # github.com/chexum/genromfs (0.5.7)

log() { printf '\n=== %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --- 1. preflight -------------------------------------------------------------------------
if [ "$PREPARE_ONLY" != 1 ] && [ "$NATIVE" != 1 ]; then
  command -v docker >/dev/null || die "docker is not installed"
  docker info >/dev/null 2>&1 || die "docker info fails for $(id -un) (daemon down or needs sudo / docker group)"
fi
mkdir -p "$WORK" "$OUT"

# --- 2. clone v1.16.2 with submodules ------------------------------------------------------
if [ ! -d "$SRC/.git" ]; then
  log "clone PX4-Autopilot $TAG"
  git clone --branch "$TAG" --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git "$SRC"
fi
cd "$SRC"
[ "$(git rev-parse "$TAG^{commit}")" = 54f0455ffcd755534539a7cf33a09a20bf71d29d ] || die "tag $TAG is not the expected commit"
NX=platforms/nuttx/NuttX/nuttx
[ "$(git rev-parse "$TAG:$NX")" = "$NUTTX_PIN" ] || die "$TAG does not pin NuttX at $NUTTX_PIN"

# --- 3. cherry-pick the fix into the NuttX submodule ---------------------------------------
if [ "$(git rev-parse HEAD)" = "$(git rev-parse "$TAG^{commit}")" ]; then
  log "cherry-pick PX4/NuttX $FIX into $NX"
  git diff --quiet && git diff --cached --quiet || die "superproject has local changes"
  (
    cd "$NX"
    [ "$(git rev-parse HEAD)" = "$NUTTX_PIN" ] || die "NuttX is not at $NUTTX_PIN"
    # depth 2 so the fix commit has its parent (a depth-1 fetch makes cherry-pick see the whole tree as new)
    git fetch --depth 2 origin "$FIX"
    git -c user.name="$GIT_COMMITTER_NAME" -c user.email="$GIT_COMMITTER_EMAIL" cherry-pick -x "$FIX"
  )
  git add "$NX"
  GIT_AUTHOR_DATE="$GIT_COMMITTER_DATE" git -c user.name="$GIT_AUTHOR_NAME" -c user.email="$GIT_AUTHOR_EMAIL" \
    commit -q -m "NuttX: cherry-pick stm32h7 UART TX DMA stuck fix (PX4/NuttX ${FIX:0:7})

Moves the NuttX submodule from ${NUTTX_PIN:0:9} (the $TAG pin) to ${NUTTX_PIN:0:9} plus
cherry-pick -x of PX4/NuttX $FIX (px4_firmware_nuttx-10.3.0+, PX4/NuttX PR #353,
backport of apache/nuttx PR #17203; in PX4 main via PX4-Autopilot PR #25791).
Zeroes dmatx.length/nlength in up_dma_shutdown() and the stream NDTR/CNDTR in
stm32_dmafree(), so a TX-DMA UART (TELEM2 on fmu-v6c) keeps working after a
close/reopen during a transfer. No other change."
fi

# Verify the result is exactly v1.16.2 + this one change.
[ "$(git rev-parse HEAD~1)" = "$(git rev-parse "$TAG^{commit}")" ] || die "HEAD is not $TAG + 1 commit"
[ "$(git diff --name-only "$TAG" HEAD)" = "$NX" ] || die "superproject diff touches more than $NX"
git diff --quiet && git diff --cached --quiet || die "superproject is dirty"
NX_HEAD=$(git -C "$NX" rev-parse HEAD)
[ "$(git -C "$NX" rev-parse HEAD~1)" = "$NUTTX_PIN" ] || die "NuttX HEAD~1 is not $NUTTX_PIN"
[ "$(git -C "$NX" diff --name-only HEAD~1 HEAD | tr '\n' ' ')" = \
  "arch/arm/src/stm32h7/stm32_dma.c arch/arm/src/stm32h7/stm32_serial.c " ] || die "unexpected files in NuttX fix"
git -C "$NX" diff HEAD~1 HEAD -- arch/arm/src/stm32h7/stm32_serial.c | grep -q '^+ *dev->dmatx.nlength = 0;' \
  || die "fix content missing (dmatx.nlength)"
git -C "$NX" diff HEAD~1 HEAD -- arch/arm/src/stm32h7/stm32_dma.c | grep -q 'STM32_DMA_SNDTR_OFFSET, 0' \
  || die "fix content missing (SNDTR)"
# Same code change as upstream d50ffd3 (compare patch-ids).
[ "$(git -C "$NX" show HEAD | git patch-id --stable | cut -d' ' -f1)" = \
  "$(git -C "$NX" show "$FIX" | git patch-id --stable | cut -d' ' -f1)" ] || die "cherry-pick differs from $FIX"
git -C "$NX" format-patch -1 --full-index --stdout HEAD > "$OUT/nuttx-stm32h7-uart-txdma-stuck-${FIX:0:7}.patch"

# The version strings depend on git history/tags, which the shallow clone lacks:
#  - src/lib/version/px_update_git_header.py takes the last nuttx-X.Y.Z tag in the NuttX
#    submodule (IndexError if there are none -> build fails);
#  - nuttx/tools/version.sh (NuttX banner "NuttX-x.y.z", CONFIG_VERSION_*) uses describe/tags.
# Fetch full NuttX history + tags, then drop nuttx-12.12.0, which PX4/NuttX gained after the
# stock v1.16.2 build: the stock image embeds "v11.0.0" / "NuttX-11.0.0" (checked with
# `strings` on the official release .px4).
if [ -f "$(git -C "$NX" rev-parse --git-dir)/shallow" ]; then
  log "unshallow NuttX submodule (+tags) for version strings"
  git -C "$NX" fetch --unshallow --tags origin
fi
git -C "$NX" tag -d nuttx-12.12.0 >/dev/null 2>&1 || true
NX_TAG=$(git -C "$NX" -c versionsort.suffix=- tag --sort=v:refname | grep -oE 'nuttx-[0-9]+\.[0-9]+\.[0-9]+' | tail -1)
[ "$NX_TAG" = nuttx-11.0.0 ] || die "NuttX version tag would be $NX_TAG, stock is nuttx-11.0.0"
[ "$(git -C "$NX" rev-parse HEAD)" = "$NX_HEAD" ] || die "NuttX HEAD moved"

IDENT=$(git describe --exclude 'ext/*' --always --tags --dirty)
log "source ready: $IDENT  (NuttX $NX_HEAD)"
case "$IDENT" in v1.16.2-1-g*) ;; *) die "unexpected git describe: $IDENT";; esac
[ "$PREPARE_ONLY" = 1 ] && { echo "PREPARE_ONLY=1: stopping before the build"; exit 0; }

if [ "$NATIVE" = 1 ]; then
  # --- 4n. user-space toolchain in $TOOLS (no sudo) ----------------------------------------
  mkdir -p "$TOOLS/bin"
  if [ ! -x "$GCC_DIR/bin/arm-none-eabi-gcc" ]; then
    if [ ! -f "$TOOLS/$GCC_TAR" ]; then
      for u in $GCC_URLS; do
        log "download $u"
        curl -fL --retry 3 -o "$TOOLS/$GCC_TAR.part" "$u" && mv "$TOOLS/$GCC_TAR.part" "$TOOLS/$GCC_TAR" && break
      done
    fi
    [ -f "$TOOLS/$GCC_TAR" ] || die "could not download $GCC_TAR"
    echo "$GCC_MD5  $TOOLS/$GCC_TAR" | md5sum -c --quiet || die "md5 mismatch for $GCC_TAR"
    echo "$GCC_SHA256  $TOOLS/$GCC_TAR" | sha256sum -c --quiet || die "sha256 mismatch for $GCC_TAR"
    tar -xjf "$TOOLS/$GCC_TAR" -C "$TOOLS"
  fi
  "$GCC_DIR/bin/arm-none-eabi-gcc" --version | head -1 | grep -q '9.3.1 20200408 (release)' \
    || die "unexpected arm-none-eabi-gcc version"
  if [ ! -x "$TOOLS/bin/genromfs" ]; then
    [ -d "$TOOLS/genromfs/.git" ] || git clone https://github.com/chexum/genromfs "$TOOLS/genromfs"
    git -C "$TOOLS/genromfs" checkout -q "$GENROMFS_REV"
    make -C "$TOOLS/genromfs" genromfs
    cp "$TOOLS/genromfs/genromfs" "$TOOLS/bin/"
  fi
  # Ubuntu 22.04's /usr/bin/python3 (3.10) lacks ensurepip (python3-venv not installed),
  # so make the venv without pip and bootstrap pip with get-pip.py.
  if [ ! -x "$TOOLS/venv/bin/python3" ] || ! "$TOOLS/venv/bin/python3" -c 'import em, kconfiglib, jinja2, pymavlink' 2>/dev/null; then
    rm -rf "$TOOLS/venv"
    if ! /usr/bin/python3 -m venv "$TOOLS/venv" 2>/dev/null; then
      rm -rf "$TOOLS/venv"
      /usr/bin/python3 -m venv --without-pip "$TOOLS/venv"
      curl -fsSL -o "$TOOLS/get-pip.py" https://bootstrap.pypa.io/pip/get-pip.py
      "$TOOLS/venv/bin/python3" -I "$TOOLS/get-pip.py" -q
    fi
    "$TOOLS/venv/bin/python3" -m pip install -q -r "$SRC/Tools/setup/requirements.txt"   # empy>=3.3,<4
  fi
  # pydronecan (uavcan dsdl compiler) imports pkg_resources, removed in setuptools 81;
  # requirements.txt only says >=39.2.0, so cap it (the docker image's setuptools predates 81).
  "$TOOLS/venv/bin/python3" -W ignore -c 'import pkg_resources' 2>/dev/null \
    || "$TOOLS/venv/bin/python3" -m pip install -q "setuptools<81"
  # Same cmake/ninja as the CI image px4io/px4-dev-nuttx-focal:2022-08-12 (focal apt: cmake
  # 3.16.3, ninja-build 1.10.0). This matters: with cmake 3.22 (host) or Unix Makefiles the
  # image embeds absolute __FILE__ paths (/tmp/.../src/...) instead of stock's "../../src/...",
  # +648 bytes and different code layout.
  "$TOOLS/venv/bin/cmake" --version 2>/dev/null | grep -q 'version 3.16.3$' && \
    "$TOOLS/venv/bin/ninja" --version 2>/dev/null | grep -q '^1.10.0' \
    || "$TOOLS/venv/bin/python3" -m pip install -q "cmake==3.16.3.post1" "ninja==1.10.0"
  "$TOOLS/venv/bin/python3" -m pip freeze > "$TOOLS/pip-freeze.txt"

  # --- 4. native build ----------------------------------------------------------------------
  # Minimal PATH so no other python3/cmake/compiler (e.g. a project venv) is picked up; the
  # venv's cmake 3.16.3 + ninja 1.10.0 come before /usr/bin.
  rm -rf "build/$TARGET"
  START=$(date +%s)
  env PATH="$TOOLS/bin:$GCC_DIR/bin:$TOOLS/venv/bin:/usr/bin:/bin" \
    make "$TARGET" -j"$(nproc)" 2>&1 | tee "$WORK/build.log"
  END=$(date +%s)
  echo "native build took $((END - START)) s" | tee -a "$WORK/build.log"
  grep -q '^CMAKE_GENERATOR:INTERNAL=Ninja$' "build/$TARGET/CMakeCache.txt" || die "not built with Ninja"
else
  # --- 4. build in PX4's docker toolchain --------------------------------------------------
  # Same as Tools/docker_run.sh 'make px4_fmu-v6c_default', minus -it (no TTY needed) and the
  # MAVLink port; runs as the current uid so build files stay owned by the user. ccache is off
  # so nothing from earlier builds leaks in.
  log "docker pull $IMAGE"
  docker pull "$IMAGE"
  DIGEST=$(docker image inspect --format '{{index .RepoDigests 0}}' "$IMAGE")
  rm -rf "build/$TARGET"
  START=$(date +%s)
  docker run --rm -w "$SRC" --user "$(id -u):$(id -g)" \
    --env CCACHE_DISABLE=1 --env HOME=/tmp \
    --volume "$SRC:$SRC:rw" \
    "$IMAGE" /bin/bash -c "make $TARGET" 2>&1 | tee "$WORK/build.log"
  END=$(date +%s)
  echo "build took $((END - START)) s with $DIGEST" | tee -a "$WORK/build.log"
fi

# --- 5. verify the artifact ----------------------------------------------------------------
PX4=build/$TARGET/$TARGET.px4
[ -f "$PX4" ] || die "no $PX4"
/usr/bin/python3 -I - "$PX4" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
for k in ("summary", "board_id", "board_revision", "git_identity", "git_hash", "image_size", "image_maxsize"):
    print(f"{k}: {d.get(k)}")
assert d["board_id"] == 56, "board_id is not 56 (fmu-v6c)"
assert d["git_identity"].startswith("v1.16.2-1-g"), d["git_identity"]
assert d["image_size"] <= d["image_maxsize"]
EOF
/usr/bin/python3 -I - "$PX4" "$WORK/image.bin" <<'EOF'
import json, sys, zlib, base64
d = json.load(open(sys.argv[1])); img = zlib.decompress(base64.b64decode(d["image"]))
assert len(img) == d["image_size"]; open(sys.argv[2], "wb").write(img)
EOF
strings -n 6 "$WORK/image.bin" > "$WORK/image.strings"   # (no grep -q in a pipe: pipefail + SIGPIPE)
grep -qxF '9.3.1 20200408 (release)' "$WORK/image.strings" || die "image lacks the stock toolchain string"
grep -qxF 'NuttShell (NSH) NuttX-11.0.0' "$WORK/image.strings" || die "image NuttX banner differs from stock"
! grep -qF "$SRC" "$WORK/image.strings" || die "image embeds absolute source paths"
grep -m1 'The C compiler identification' "$WORK/build.log" || true

NAME=${TARGET}_${IDENT}_txdma.px4
cp "$PX4" "$OUT/$NAME"
(cd "$OUT" && sha256sum "$NAME" > "$NAME.sha256" && cat "$NAME.sha256")
cp "$WORK/build.log" "$OUT/build.log"
[ "$SCRIPT" -ef "$OUT/$(basename "$SCRIPT")" ] || cp "$SCRIPT" "$OUT/"
log "done: $OUT/$NAME"
