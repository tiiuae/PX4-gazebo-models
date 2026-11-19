# Saluki FT HITL with Gazebo Harmonic

This procedure targets PX4 1.17, Gazebo Harmonic (Sim 8), Ubuntu 24.04 AMD64,
and the MAVSDK C++ version in `test/mavsdk_tests/MAVSDK_VERSION` (currently 3.7.0).
Run repository commands from the PX4 root unless stated otherwise. Installation,
controller preparation, simulator startup, flight, and failover are separate activities.
The dated [host setup record](../../../docs/en/simulation/saluki_ft_hitl_setup.md)
is historical evidence and may be absent from a clean checkout; it is not an installation prerequisite.

## Native installation and build

Start with a checkout whose submodules match its pinned revisions. Preserve local
work before updating submodules. Harmonic supports Ubuntu 22.04 and 24.04
([official installation instructions](https://gazebosim.org/docs/harmonic/install_ubuntu/));
this procedure and the container use 24.04 to match the MAVSDK package.

```bash
git submodule update --init --recursive
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo 'deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic build-essential cmake ninja-build pkg-config git \
  astyle file zip unzip rsync shellcheck libboost-system-dev libboost-thread-dev \
  libboost-filesystem-dev cppzmq-dev libprotobuf-dev protobuf-compiler libxml2-dev libxml2-utils \
  python3-dev python3-pip python3-venv libtinyxml-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
  libgl1-mesa-dri libegl1 mesa-utils
python3 -m venv .venv-hitl
source .venv-hitl/bin/activate
python -m pip install -r Tools/setup/requirements.txt
python -m pip check
version=$(cat test/mavsdk_tests/MAVSDK_VERSION)
curl -fL "https://github.com/mavlink/MAVSDK/releases/download/v${version}/libmavsdk-dev_${version}_ubuntu24.04_amd64.deb" \
  -o /tmp/saluki-mavsdk.deb
sudo apt-get install -y /tmp/saluki-mavsdk.deb
export GZ_DISTRO=harmonic
make px4_sitl_default gzsim-plugins px4_gz_plugins mavsdk_tests -j4
# Equivalent incremental build:
./packaging/build_px4_hitl_gzsim_mavsdk.sh -j4
```

`cppzmq-dev` supplies the Gazebo transport C++ headers/imported target when
recommended packages are disabled. GStreamer base/good/ugly plugins supply the
video conversion, UDP/RTP and software H.264 elements used by GstCameraSystem.
TinyXML (`tinyxml.h`) is required by the HITL bridge; TinyXML2 alone is insufficient.
OpenCV and GStreamer development packages are required by the standard PX4 plugins.
The three build targets produce:

- `build/px4_sitl_default/build_gz-sim_plugins/libmavlink_hitl_gazebosim.so`
- `build/px4_sitl_default/src/modules/simulation/gz_plugins/libOpticalFlowSystem.so`
- `build/px4_sitl_default/src/modules/simulation/gz_plugins/libGstCameraSystem.so`
- `build/px4_sitl_default/mavsdk_tests/mavsdk_tests` and `mavsdk_preparing`
- `build/px4_sitl_default/rootfs/gz_env.sh`

Building those targets does not build the PX4 SITL executable; also run
`GZ_DISTRO=harmonic make px4_sitl_default -j4` before ordinary SITL tests.

## Container installation and operation

The [Dockerfile](../../../packaging/Dockerfile.hitl_gzsim_mavsdk) uses the same
pinned Ubuntu 24.04 base for builder and runtime, HTTPS OSRF repositories and a
Python virtual environment. MAVSDK's version is read once from the repository,
and the Noble AMD64 package is shared by both stages. ROS is not needed by these
HITL/MAVSDK entry points. The image baseline is pinned; apt and Python packages
are resolved at build time, so this is not a bit-for-bit package lock.

Create a clean build context with a directory named `px4-firmware`, including its
initialized submodules and Git metadata. Exclude existing build outputs and local
Python environments. For example, from the repository root:

```bash
context=$(mktemp -d)
mkdir "$context/px4-firmware"
rsync -a --exclude=/build/ --exclude=/builds/ --exclude=/px4_sitl_default/ \
  --exclude=/.venv-hitl/ --exclude=__pycache__/ ./ "$context/px4-firmware/"
docker build -f "$context/px4-firmware/packaging/Dockerfile.hitl_gzsim_mavsdk" \
  -t saluki-hitl:harmonic "$context"
# A build is insufficient validation: run the loopback checks in the image.
docker run --rm --network host -v "$PWD/logs/container-smoke:/logs" \
  saluki-hitl:harmonic python3 Tools/simulation/gz/tests/hitl_smoke.py \
  --log-dir /logs/manual
# Separate camera check; see the rendering limitations below.
docker run --rm --network host -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/logs/container-smoke:/logs" saluki-hitl:harmonic \
  python3 Tools/simulation/gz/tests/hitl_smoke.py --camera --log-dir /logs/camera
```

The runtime layout remains `/px4-firmware`, matching absolute paths in `gz_env.sh`.
The server configuration is copied too. The image serves both single-controller
and FT HITL; select the model or test configuration at runtime. Use the smoke
checks above to verify startup, plugin loading and resource paths. Do not mount an
older host build tree over the image's build directory.

On Linux, host networking supplies the controller-facing host IP and UDP ports
without a container NAT mapping, and supports Gazebo discovery with a separate
GUI. Loopback in an isolated Docker network refers to that container, not the host.
If archive access is unavailable, the Dockerfile accepts `UBUNTU_MIRROR` and
`UBUNTU_SECURITY_MIRROR` build arguments for a reachable official Ubuntu mirror.
Container networking checks cover isolated loopback startup. Verify hardware
connectivity separately from a successful image build.

```bash
docker run --rm --network host --device /dev/dri \
  saluki-hitl:harmonic ./Tools/simulation/gz/hitl_run.sh \
  ssrc_holybro_x500/model_ft_hitl.sdf --headless
```

Use `--device /dev/dri` only when that render device exists. Software rendering
may be selected with `-e LIBGL_ALWAYS_SOFTWARE=1`; verify camera frames on the
actual host. SSH headless operation still needs EGL/OpenGL support for cameras.

## Ethernet and controller preparation

Use a dedicated wired network: host `192.168.202.254/24`, primary controller
`192.168.202.1`, secondary controller `192.168.202.65`. The interface name `eno1`
below is an example; select the actual interface with `ip -br link`. Preserve the
SSH/default route on another interface. Check address conflicts before assignment.

```bash
nmcli connection add type ethernet ifname eno1 con-name saluki-hitl \
  ipv4.method manual ipv4.addresses 192.168.202.254/24 ipv4.never-default yes \
  ipv6.method link-local ipv6.never-default yes
nmcli connection up saluki-hitl
ip -br address show eno1
ip route get 192.168.202.1
ping -c 3 192.168.202.1
ping -c 3 192.168.202.65
```

Review existing profiles before creating another. An unavailable interface or
inactive profile must be resolved before a hardware test; a saved profile is not
proof that the address is active. Allow the following UDP flows through any host
firewall, and keep telemetry separate from simulation:

| Endpoint | Local HITL port | Destination |
| --- | --- | --- |
| Host primary bridge | 14542 | 192.168.202.1:14543 |
| Primary controller | 14543 | 192.168.202.254:14542 |
| Host secondary bridge | 14562 | 192.168.202.65:14543 |
| Secondary controller | 14543 | 192.168.202.254:14562 |
| Host MAVSDK telemetry | 14540 | Controller telemetry/command connection |

Before hardware operation, separately prepare and inspect **both** controllers
using their maintenance/console procedure. Record firmware revision, `param show
SYS_AUTOSTART`, `param show SYS_HITL`, `param show MAV_SYS_ID`, `param show
MAV_COMP_ID`, `param show FT_N_SPARE_FCS`, `netconfig get_ipv4`, `mavlink status`
and `redundancy status` from each controller.

The hardware X500 HITL airframe is **1401**, with `SYS_HITL=1`. POSIX Gazebo SITL
uses **4401**; these IDs belong to different startup directories. Do not apply the
POSIX ID to the hardware. See the [hardware airframe](../../../ROMFS/px4fmu_common/init.d/airframes/1401_ssrc_holybro_x500.hil),
[board defaults](../../../boards/ssrc/saluki-ft/init/rc.board_defaults) and
[board MAVLink startup](../../../boards/ssrc/saluki-ft/init/rc.board_mavlink).

Board startup selects the primary channel with `MAV_COMP_ID=1`; the spare must
have its configured secondary identity (normally component 2). Verify identities
against the deployed redundancy configuration, including system identity; do not
blindly assign different MAVLink system IDs to a redundant vehicle. With one
spare, verify `FT_N_SPARE_FCS=1` and redundancy startup on both controllers.
Board startup derives the peer address with `netconfig get_ipv4 1/2`; redundancy
uses UDP ports 14544/14545 with reversed directions on the two controllers.
Only the primary starts the normal telemetry connection to host port 14540.
Secure telemetry (`SECU_MAV_ENA`) has its own proxy configuration; verify that
path separately when enabled.

For a two-controller FT test, the common preparation values on each controller
console are:

```text
param set SYS_AUTOSTART 1401
param set SYS_HITL 1
param set FT_N_SPARE_FCS 1
```

Set `param set MAV_COMP_ID 1` on the primary and `param set MAV_COMP_ID 2` on the
secondary, after verifying those roles and the provisioned network identities.
Use `param save`, then `reboot` on each controller and repeat the verification
above. Confirm `.1` / `.65` and the corresponding board-startup channels after
reboot. Preserve the intended common vehicle system identity and follow the
board's provisioning procedure if network identity differs. For single-controller
coverage, use its intended no-spare configuration and the single-controller SDF.

If other values differ, apply the controller-specific preparation procedure and
reboot/verify each controller before launch. The automated runner connects to the
primary, can set `SYS_AUTOSTART` and reboot it, and does **not** coordinate setup
of both FT controllers. Neither smoke mode invokes that preparation path.

## Manual simulation

```bash
source .venv-hitl/bin/activate
export GZ_DISTRO=harmonic
# One controller:
./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf --headless
# Or FT, after stopping the preceding process with Ctrl-C:
./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_ft_hitl.sdf --headless
```

Omit `--headless` for the integrated GUI. It expands to `-s --headless-rendering`
when present. The launcher owns `worlds/default.sdf` and the `default` world name.
It accepts `-s`, `--server-only`, `--headless-rendering`, `-v LEVEL`,
`--verbose LEVEL`, `--render-engine NAME` and `--render-engine-server NAME`
(including long `--option=value` forms). Other options, world filenames, GUI-only
mode, alternate server configurations and playback are rejected. Use `gz sim`
directly for other worlds. Model arguments may be relative to `PX4_GZ_MODELS` or
absolute (useful for temporary loopback copies).

The launcher preserves inherited plugin/resource directories, reads the generated
environment, checks the bridge and standard PX4 libraries, and prints
`GZ_PARTITION=...`. It waits up to 30 seconds for `/world/default/create`, submits
one request with a bounded timeout and requires a true Boolean response. An early
server exit, discovery timeout or failed spawn returns nonzero. Interruptions and
failures terminate only the process group started by this invocation.

Without an explicit partition, each launch gets a fresh UUID. To choose one:

```bash
GZ_PARTITION=saluki-lab-operator1 ./Tools/simulation/gz/hitl_run.sh \
  ssrc_holybro_x500/model_ft_hitl.sdf --headless
# In another terminal at the PX4 root, load resources and use the printed partition:
source build/px4_sitl_default/rootfs/gz_env.sh
GZ_PARTITION=saluki-lab-operator1 gz sim -g
```

An already advertised create service in the selected partition is rejected before
launch. This check is not a distributed lock: concurrent launches must use
**distinct partitions**. A partition isolates Gazebo discovery/services, not
MAVLink UDP sockets. Hardware models retain fixed addresses and ports, so concurrent
hardware sessions also require separate network/port assignments. An unrelated
GUI must use the same partition and compatible resource paths to view this world.

## Automated execution

Stop a manually launched simulator before running the automated flight runner.
After separately preparing both controllers:

```bash
source .venv-hitl/bin/activate
export GZ_DISTRO=harmonic
python3 test/mavsdk_tests/mavsdk_test_runner.py \
  test/mavsdk_tests/configs/hitl_ft_gz_harm.json --abort-early \
  --log-dir logs/saluki-ft --case 'Takeoff and hold position'
```

Omit `--case` to run cases selected by the JSON's multicopter/offboard filters;
these are ordinary flight cases, not dedicated failover tests. Add `--gui` for a
GUI client. The existing `hitl_gz_harm.json` remains available for single-controller
coverage (and includes other airframes). FT selects X500 only, `model_ft_hitl`,
hardware autostart 1401, `udp://:14540` telemetry and a ten-minute case timeout.
Logs are under the chosen directory, grouped by timestamp, model and case.

The Harmonic runner propagates one partition to the server, spawner, client and
PX4 SITL process. Each new session gets a new automatic partition; an explicitly
supplied value is retained and checked for an existing world. It preserves plugin
paths, includes both plugin directories, checks readiness and checks the model
create response. It does not retry an ambiguous create request.

## Validation procedures

Host-only checks require built plugins but no controllers and never change
checked-in SDFs or invoke `mavsdk_preparing`:

```bash
python3 -m unittest discover -s test/mavsdk_tests/tests -v
bash -n Tools/simulation/gz/hitl_run.sh packaging/build_px4_hitl_gzsim_mavsdk.sh
shellcheck Tools/simulation/gz/hitl_run.sh packaging/build_px4_hitl_gzsim_mavsdk.sh
python3 Tools/simulation/gz/tests/hitl_smoke.py --camera
python3 Tools/simulation/gz/tests/hitl_smoke.py --automated --camera
```

Each smoke run creates unique partitions even if `GZ_PARTITION` is inherited,
allocates separate loopback UDP ports, copies the real single/FT SDFs into temporary
files, and changes only MAVLink destinations/ports. It requires model creation,
one/two live UDP source channels and no required plugin-loading errors. It also
starts a comparison `default` world in another partition, checks that only the
smoke partition contains the drone, stops the smoke session and verifies that the
comparison remains alive. It then cleans up that comparison simulator. Logs and
port/partition records remain under `logs/hitl-smoke-*` (or `--log-dir`, which must
not already exist). `--camera` additionally requires a frame on each X500 camera
topic; image payloads are omitted from saved metadata. A successful UDP startup
check does not validate controller behavior or flight dynamics.

Hardware flight validation requires verified controller preparation, a connected
network, an approved test environment and recorded flight results. Run the
selected flight case and retain controller, simulator and MAVSDK logs. Explicit
FT validation additionally requires a defined primary failure/takeover scenario,
observed controller identities, actuator output continuity and recovery behavior.
Record those results separately; passing the filters above does not establish FT
failover. No hardware flight or failover result is claimed here.

## Troubleshooting and model limitations

- Missing `gz_env.sh`: initialize submodules and build the three listed targets.
  Do not source an environment generated in a different source/build location.
- Plugin lookup: inspect `GZ_SIM_SYSTEM_PLUGIN_PATH` as a colon-separated list.
  Keep inherited entries and both build directories. `libOpticalFlowSystem.so`
  and `libGstCameraSystem.so` come from `px4_gz_plugins`, not `gzsim-plugins`.
  Run `ldd` on a failing library and resolve every `not found` entry.
- Dependency failures: verify Harmonic/Sim 8, TinyXML, OpenCV and GStreamer
  development packages, the MAVSDK version file and `python -m pip check`.
  Use a MAVSDK package matching the OS and architecture. Container repository
  download timeouts are an environment issue until reproduced with working access.
- Spawn failure: read the startup log, check the selected model and
  `GZ_SIM_SERVER_CONFIG_PATH`, and run `GZ_PARTITION=... gz service -l` using the
  printed partition. The default create service needs the UserCommands system.
  Stop a conflicting simulator or choose a new partition; do not reuse its service.
- SSH/Qt errors: use `--headless`. EGL/render-engine errors are separate from GUI
  display initialization. Verify `/dev/dri` access and camera frames, not just spawn.
- The shared X500 had an unsupported `<empty/>` gimbal-pan visual. Omitting that
  invisible visual clears the reproduced Ogre2 geometry error without changing
  links, collisions, masses, inertias, poses or joints.
- `camera_link` still has zero mass/inertia and a fixed joint to `base_link`.
  Harmonic/DART warns about an invalid inertia matrix and a zero-mass BodyNode.
  The sensor is an offset third-person camera, not a measured physical payload.
  Assigning arbitrary positive mass could alter total mass and center of mass;
  a future fix must establish intended fixed-joint/frame behavior and compare
  dynamics. Reproduce with either `--camera` smoke mode. Camera frames and startup
  success do not validate model fidelity. Ogre2 visibility-mask and software H.264
  fallback warnings may also remain. Automated and software-rendered container
  camera checks have timed out despite passing manual/SITL frame checks. Reproduce
  with the `--automated --camera` smoke mode or the container camera command above.
