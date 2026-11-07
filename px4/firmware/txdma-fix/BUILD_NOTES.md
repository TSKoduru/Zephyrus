# PX4 v1.16.2 + NuttX stm32h7 UART TX DMA fix, px4_fmu-v6c_default

## Artifact (vendored)
| | |
|---|---|
| file | `px4_fmu-v6c_default_v1.16.2-1-g7a39c59_txdma.px4` as built; vendored in this repo as `px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` (same bytes, repo naming) |
| sha256 | `fb23d654a5c213e80231451c848f519194bc505ffecb35d41d9e13f957e8d9c9` |
| git_identity | `v1.16.2-1-g7a39c59` (git_hash 7a39c5954139c917d21e0c5cdf6cc10449b1c011) |
| board_id / summary | 56 / PX4FMUv6C |
| image_size | 1959884 of 1966080 bytes. It fits: 6196 bytes free. That is 72 bytes more than stock (1959812). |
| produced by | `NATIVE=1 ./build_fmu-v6c_v1.16.2-txdma.sh`. No docker on the laptop (not installed, no sudo). |
| build time | 201 s for the script's final build. Earlier full builds took 61-246 s at -j16. |
| toolchain | build log: `The C compiler identification is GNU 9.3.1`. `arm-none-eabi-gcc --version`: `arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)`. The image embeds "GNU GCC" and "9.3.1 20200408 (release)", the strings `ver all` prints, and they are identical to stock. The build log has no "Toolchain:" line; that line is printed on the FC by `ver all`. |

The sha256 is not reproducible from build to build, because PX4 embeds `__DATE__`/`__TIME__`
(stock has the same). Expect a different sha256 every rebuild.

## Source
- PX4-Autopilot v1.16.2 = 54f0455ffcd7. NuttX was pinned at 886acbbdb4f061e5c0ce1a76afbcfa7cb7df9849.
- `cherry-pick -x d50ffd354495ce5d30814dba41d2eb70a32b8fa3` (PX4/NuttX PR #353, backport of
  apache/nuttx PR #17203; in PX4 main via PX4-Autopilot PR #25791) applied cleanly.
  - Result: NuttX c21890c6c91d09a648308644094bd86817231682. The patch-id equals d50ffd3.
  - Files: arch/arm/src/stm32h7/stm32_serial.c (`dmatx.length/nlength = 0` in up_dma_shutdown)
    and stm32_dma.c (a dma_free op zeroes SNDTR/CNDTR, called from stm32_dmafree).
- The ELF has `stm32_sdma_free` (`str r2=0, [regs+4]`) and `stm32_bdma_free`.
  `stm32_mdma_free` is absent, so it was presumably dropped as unused; MDMA isn't used here.
- One superproject commit, 7a39c595, only bumps the submodule. Identity and dates are fixed
  ("skybeta build", 2026-10-20T00:00:00Z), so the SHAs are reproducible; two clean runs agreed.
- Patch: `nuttx-stm32h7-uart-txdma-stuck-d50ffd3.patch`.

## Native toolchain (all in px4build/tools, no sudo)
- ARM GCC 9-2020-q2-update, from the developer.arm.com URL (first try worked).
  - md5 2b9eeccc33470f9d3cda26983b9d2dc6, matching ARM's published value.
  - sha256 5adc2ee03904571c2de79d5cfc0f7fe2a5c5f54f44da5b645c17ee57b217f11f.
  - This is the same tarball the CI image installs (its Dockerfile history shows the URL).
- genromfs 0.5.7 from github.com/chexum/genromfs @ e4225b49a7be.
- Python 3.10.12 venv (/usr/bin/python3).
  - It had no ensurepip, so I used `venv --without-pip` + get-pip.py, which installed pip 26.2.2.
  - Then `pip install -r Tools/setup/requirements.txt`, which gave empy 3.3.4, plus:
    - **setuptools<81** (80.10.2): pydronecan imports pkg_resources, which was removed in
      setuptools 81. The build failed without this pin.
    - **cmake==3.16.3.post1 and ninja==1.10.0**, the versions in the CI image
      `px4io/px4-dev-nuttx-focal:2022-08-12` (focal apt; read from the image config on Docker Hub).
  - Full list: tools/pip-freeze.txt.
- Build PATH: `tools/bin:gcc/bin:tools/venv/bin:/usr/bin:/bin`. This is minimal so that the
  user's 3.13 project venv is not used.

## Deviations, and what was learned
1. **No docker, so native build.** The script now has both modes. Docker mode is unchanged
   but has never been run.
2. **The generator and CMake version DO change the image.**
   - Host cmake 3.22 + Unix Makefiles, and also cmake 3.22 + ninja 1.13: the image embedded
     absolute `/tmp/claude-1000/.../src/...` `__FILE__` paths and came out +648 bytes.
   - cmake 3.16.3 + ninja 1.10.0: the paths are relative `../../src/...` as in stock, and the
     size is +72 bytes.
   - The script asserts Ninja and that the image has no absolute source path.
3. **Fetched the fix at depth 2.** At depth 1 the commit has no parent.
4. **Unshallowed the NuttX submodule with tags, and deleted the local tag nuttx-12.12.0.**
   This was needed for px_update_git_header.py (IndexError without tags) and to match stock's
   "v11.0.0" / "NuttX-11.0.0". Both are confirmed in this image.
5. ccache is not installed or used.

## Control build (unpatched v1.16.2, same native toolchain) vs official release
I built exactly v1.16.2 the same way and compared it with the official
`px4_fmu-v6c_default.px4` (sha256 f23a3ed4...776f).
- **Matches:** image size 1959812 = 1959812. The set of strings is identical except the
  `__DATE__`/`__TIME__` stamps.
- **Differs:** bytes are not identical. Function placement and branch/address offsets differ
  (892 diff runs).
- So the code content matches, but the link order differs. My guess is the CI image's Python
  3.8 versus 3.10 here, or directory iteration order in code generators; not proven.
- Patched vs stock: +72 bytes, which fits the fix (3 small functions + 4 table pointers + calls
  + 2 stores) and the longer version string.
- parameter_xml and airframe_xml are byte-identical to stock.
- The .px4 JSON has the same keys as stock. Only image, build_time, image_size, git_identity
  and git_hash differ.

## Version the FC will announce (PX4's version_tag_to_number, compiled with host gcc and run)
| | stock v1.16.2 | this build |
|---|---|---|
| flight_sw_version | 0x011002FF (RELEASE) | 0x01100200 (DEV) |
| flight_custom_version (git hash bytes) | 54f0455ffc | 7a39c59541 |
| os_custom_version (NuttX hash) | 886acbbdb4f061e5 | c21890c6c91d09a6 |
| os_sw_version | v11.0.0 | v11.0.0 |

- The type is DEV because "-N-g<sha>" means 2 dashes and 2 dots.
- PX4_GIT_TAG_OR_BRANCH_NAME is "main" instead of "v1.16.2". This only changes the fallback
  metadata URL directory.

**/home/robotics/pixhawk_autoparams.py will NOT reflash a stock v1.16.2 board with this.**
- Both `firmware_version()` (regex on git_identity) and `autopilot_version()` (bytes 3..1 of
  flight_sw_version, type byte dropped) give (1,16,2).
- `update_firmware()` returns when `have == want`. There is no force flag.

Minimal change: also compare the hash.
- In `autopilot_version()`: `bytes(v.flight_custom_version)[3:8][::-1].hex()`. This is derived
  from mavlink_main.cpp and not yet seen on a live board.
- In `firmware_version()`: `json["git_hash"][:10]`.
- Compare (version, hash) both before and after flashing.

## Commands
    ./build_fmu-v6c_v1.16.2-txdma.sh            # docker mode (needs docker without sudo)
    NATIVE=1 ./build_fmu-v6c_v1.16.2-txdma.sh   # what produced this artifact
    PREPARE_ONLY=1 ./build_fmu-v6c_v1.16.2-txdma.sh
