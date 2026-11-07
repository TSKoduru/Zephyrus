# Distributing the image

How the `skybeta` container image gets from the build machine onto every drone, and how a
fleet is updated afterwards. Everything here works with **any** OCI registry; the
registry is a deployment choice and nothing in the scripts assumes one.

## What is in the image

`skybeta:<VERSION>` (aarch64 only) is everything a drone runs except the host-side bits
(`host/`, the UART patch, `/etc/skybeta/drone.env`). Layer by layer, from
`docker/Dockerfile.jetson`:

| Layer | Contents |
|---|---|
| **NVIDIA base** | `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dd...`: Ubuntu 22.04, CUDA / TensorRT for JetPack 6, ROS 2 Humble, the Isaac ROS 3.x apt channel. This is 95 % of the size |
| apt | OAK-D driver **depthai-ros v3** (3.3.0), **cuVSLAM** (`isaac-ros-visual-slam` 3.2.6), NITROS IMU/odometry types, `diagnostic-updater` 4.0.7, colcon / rosdep, tmux. Sources pinned to a dated ROS snapshot (`2026-08-07`) so rebuilds are identical |
| pip | `depthai==3.10.0`, `MAVProxy==1.8.74`, `pymavlink==2.4.49`, `numpy<2` |
| udev | the OAK-D (`03e7`) rule |
| agent | **Micro-XRCE-DDS-Agent v2.4.3**, built from source: the bridge between ROS 2 and PX4 over the TELEM2 UART |
| workspace | `/root/skybeta_ws`, built with colcon: `px4_msgs` and `px4-ros2-interface-lib` (both `release/1.16`), and the four packages in `src/`: `oakd_vslam_bringup` (camera + GPU mono conversion + cuVSLAM launch, and the odometry bridge to PX4), `skybeta_flight_interface` (the competitor-facing offboard wrapper: altitude and speed clamp, ENU/NED, **no arm path**; no geofence since 0.3.0), `competitor_example`, `competitor_advanced_cpp` |
| scripts | `container_scripts/` on `$PATH`: `run_agent.sh`, `run_slam.sh`, `run_bridge.sh`, `run_full_stack.sh`, `run_competitor.sh`, `team_build.sh`, `check_topics.sh`, `record_bag.sh`, `skybeta_env.sh` |
| labels | provenance: ROS snapshot date, PX4 firmware, git refs, agent tag, and (from builds after 2026-10-16) `io.skybeta.git-sha`, the commit it was built from |

What runs inside when `host/run.sh stack` starts it: a tmux session with four windows,
`agent` (MicroXRCEAgent on `/dev/ttyTHS1`), `slam` (OAK-D driver → GPU mono8 → cuVSLAM),
`bridge` (cuVSLAM odometry → `/fmu/in/vehicle_visual_odometry`) and `competitor` (the team's
package from `~/team_ws`, mounted, not baked in). The pins and the reasons behind them are
in the [README](../README.md#inside-the-image).

Not in the image, on purpose: team code (`~/team_ws`, bind-mounted), the per-drone
identity (`/etc/skybeta/drone.env`), any credential, and the fleet's admin tooling.

## Sizes (measured 2026-10-16, `skybeta:0.2.0`)

| | Unpacked on disk | Transferred (compressed) |
|---|---|---|
| NVIDIA base | 40.3 GB | 12.6 GB |
| skybeta layers on top | ~2.3 GB | **~0.6 GB** |
| Whole image | 42.6 GB | 13.2 GB |
| Registry storage for one version | | ~13 GB (later versions share the base: only changed layers are added) |

`host/pull_image.sh` pulls the NVIDIA base from `nvcr.io` **first**, so a drone with
internet fetches only the ~0.6 GB skybeta layers from your registry. A drone with **no**
route to `nvcr.io` (an offline venue) still works: the image in your registry carries every
base layer, so the pull falls back to fetching all 13 GB from your registry.

An update that only touches `container_scripts/` moves kilobytes; one that touches `src/`
moves the colcon layer (a few hundred MB). The base is never re-sent.

## The pieces

| Step | Script | Where it runs |
|---|---|---|
| Build | `host/build_image.sh` | a Jetson (aarch64), from a clean checkout |
| Publish | `REGISTRY=<reg> host/push_image.sh` | the build machine |
| Install on a drone | `REGISTRY=<reg> host/pull_image.sh <VERSION>` (`host/provision.sh` calls it) | the drone |
| No registry | `docker save skybeta:<V>` on one machine piped into `docker load` on the drone | anywhere with ssh to both |

Tags: `skybeta:<VERSION>`, `skybeta:<git-sha>` (the commit the image was **built from**),
and `skybeta:<VERSION>-px4_<fw>-isaacros_<v>`. `latest` is never pushed and never pulled:
a moving tag is how a fleet silently splits across versions. The **digest** the pull
prints is what proves two drones run the same bits; record it in the fleet table.

## Choosing a registry: the options

`REGISTRY` is `host[:port][/namespace]`, e.g. `192.168.1.10:5000`,
`registry.example.org/skybeta`, `ghcr.io/<org>`. All of these work:

| Option | Setup | On each drone | Notes |
|---|---|---|---|
| **A. `registry:2` on a LAN machine**, plain HTTP | `docker run -d -p 5000:5000 --restart=always -v /srv/registry:/var/lib/registry --name registry registry:2` | allow it as an insecure registry (below) | simplest; needs ~13 GB per version on its disk. Keep it on a network only the fleet uses: anyone who can reach it can pull (and, without auth, push) |
| **B. `registry:2` with TLS** (a reverse proxy, or `tailscale serve` on the registry machine) | as A, plus a certificate | nothing, if the certificate is trusted | `tailscale serve --bg --https=443 5000` gives a valid `*.ts.net` certificate; drones then need the optional `tag:registry` rule in the fleet's Tailscale policy |
| **C. A hosted private registry** (GHCR, Docker Hub, Harbor, a cloud provider's) | create a private repository | `docker login` with a **read-only pull token** | check the storage quota first: 13 GB per version. GHCR's free private storage is far below that |
| **D. No registry** | none | `docker save \| docker load` from a drone that has the image | ~13 GB per drone through the admin machine, and the full 13 GB again on every update. A fallback, not a fleet strategy |

Keep the registry **private** in every case. The image contains NVIDIA's base, and
NVIDIA's container licence does not allow redistributing it publicly.

### Plain-HTTP registry (option A): allow it on each drone

Docker refuses HTTP registries unless `/etc/docker/daemon.json` lists them under
`insecure-registries`. The fleet's provisioning script does this for you (its
`--insecure-registry HOST:PORT` option, which backs the file up, keeps the existing
`nvidia` runtime entry, and restarts docker only if it changed). By hand:

```bash
sudo python3 - 192.168.1.10:5000 <<'PY'
import json, os, sys
p = "/etc/docker/daemon.json"; d = json.load(open(p)) if os.path.exists(p) else {}
regs = d.setdefault("insecure-registries", [])
if sys.argv[1] not in regs: regs.append(sys.argv[1])
json.dump(d, open(p, "w"), indent=2)
PY
sudo systemctl restart docker
```

Pushing to it from the build machine needs the same entry there. `127.0.0.1` / `localhost`
registries are exempt.

### Credentials (option C)

Teams have root on their drone, so **anything on a drone is readable by its team**.
`docker login` stores the credential in `~/.docker/config.json`. Therefore:

- On drones: a **read-only (pull-only) token**, never a token that can push. Revoke and
  re-issue it between events if teams change.
- On the build machine: a push token, which never goes to a drone.
- Provisioning streams the pull token into RAM on the drone, runs `docker login`, and
  deletes the file; it is never in a repo, in `drone.env`, or in shell history.

## Publishing a version

On the build Jetson, from a **clean** checkout of the commit you want to ship:

```bash
host/build_image.sh                      # tags <VERSION>, <git-sha>, <VERSION>-px4_..-isaacros_..
REGISTRY=<reg> host/push_image.sh --dry-run
REGISTRY=<reg> host/push_image.sh        # docker login <reg> first if it needs auth
```

Bump `VERSION` whenever the image content changes, so a semver tag never names two builds.
`push_image.sh` reads the commit from the image itself (the `io.skybeta.git-sha` label, or
the `<sha>` tag `build_image.sh` left), not from the current checkout, which may have
moved on.

## Installing on a drone

```bash
REGISTRY=<reg> host/pull_image.sh 0.3.0            # NVIDIA base from nvcr.io, then the skybeta layers
REGISTRY=<reg> host/pull_image.sh 0.3.0 --skip-base  # everything from <reg> (offline venue)
MIRROR=<lan-host:5000> REGISTRY=<reg> host/pull_image.sh 0.3.0   # through a LAN pull-through cache
```

`host/provision.sh` runs the pull as one of its phases when `REGISTRY` is set. The fleet's
provisioning script sets up registry access and runs all of it for a new drone.

## Updating a fleet

A version change is code + image together: the checkout's `VERSION` names the image
`host/run.sh` starts. The order on each drone:

1. fast-forward `~/skybeta` to the release branch or tag (team code in `~/team_ws` is never
   touched; local edits to the stack's own files block the update, on purpose),
2. pull `skybeta:<new VERSION>` (or copy it in),
3. restart the stack **when the team is not flying**: the running container keeps the old
   image until then,
4. `host/doctor.sh`.

The organiser's fleet tooling does exactly this per drone (dry run first, never restarts a
stack under an armed vehicle). Roll out to **one** drone, fly it, then the rest.

## Verified (2026-10-16, bench Orin)

- `push_image.sh` into a `registry:2` on the bench: all three tags, and the registry serves
  the **same content digest** as the local image (`sha256:b378bb9b7bae...`); the `<git-sha>`
  tag correctly names the build commit `9267c55c2781`, not the newer checkout.
- `pull_image.sh` back out of it: digest verified, image id identical. 62 s end to end
  (both on one machine); 13 GB of registry storage.
- `docker save | docker load` streamed between two ssh sessions through the admin laptop:
  identical image id.
- Not yet: a cold pull onto a second drone that has none of the layers, and a pull across
  the venue network. The first new drone does both.

Bugs found and fixed by that test: `push_image.sh` tagged the image with the *current*
checkout's commit instead of the build's, and its fallback needed a regex that Ubuntu's
`mawk` does not support.
