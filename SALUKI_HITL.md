# Saluki HITL with Gazebo Harmonic

This guide covers PX4 1.17, Gazebo Harmonic (Sim 8), Ubuntu 24.04 AMD64,
and the SSRC Holybro X500 with two Saluki FT flight controllers. Run commands
from the PX4 firmware root. Use the Gazebo models revision pinned by the firmware
checkout; initialize its nested HITL bridge submodule as well.

## Host installation and build

Preserve local work before initializing or updating submodules:

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
  astyle file zip unzip rsync libboost-system-dev libboost-thread-dev \
  libboost-filesystem-dev cppzmq-dev libprotobuf-dev protobuf-compiler libxml2-dev \
  libxml2-utils python3-dev python3-pip python3-venv libtinyxml-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgl1-mesa-dri libegl1 mesa-utils
python3 -m venv .venv-hitl
source .venv-hitl/bin/activate
python -m pip install -r Tools/setup/requirements.txt
export GZ_DISTRO=harmonic
make px4_sitl_default gzsim-plugins px4_gz_plugins -j4
```

`gzsim-plugins` builds the SSRC MAVLink HITL bridge. `px4_gz_plugins` builds
`libOpticalFlowSystem.so` and `libGstCameraSystem.so`, required by the PX4 1.17
server configuration. The generated `build/px4_sitl_default/rootfs/gz_env.sh`
contains absolute paths; rebuild after moving the checkout.

TinyXML (`tinyxml.h`) is needed by the bridge; TinyXML2 alone is insufficient.
OpenCV and GStreamer development packages are needed by the PX4 plugins.
GStreamer base/good/ugly packages supply conversion, RTP/UDP and software H.264
encoding. ROS and MAVSDK are not required for the manual FT simulator launch.
This procedure uses native host packages; it does not depend on the repository's
HITL Dockerfile or container build helper.

## Ethernet and controller preparation

Connect the host and both controllers to the dedicated HITL network:

| Endpoint | Address | Local HITL UDP port | Destination |
| --- | --- | --- | --- |
| Host primary bridge | 192.168.202.254/24 | 14542 | 192.168.202.1:14543 |
| Primary controller | 192.168.202.1/24 | 14543 | 192.168.202.254:14542 |
| Host secondary bridge | 192.168.202.254/24 | 14562 | 192.168.202.65:14543 |
| Secondary controller | 192.168.202.65/24 | 14543 | 192.168.202.254:14562 |

Select the actual Ethernet interface with `ip -br link`; `eno1` below is an
example. Review existing profiles and address usage before creating a profile.
Keep SSH and the internet default route on the management interface:

```bash
nmcli connection add type ethernet ifname eno1 con-name saluki-hitl \
  ipv4.method manual ipv4.addresses 192.168.202.254/24 ipv4.never-default yes \
  ipv6.method link-local ipv6.never-default yes
nmcli connection up saluki-hitl
ip -br address show eno1
ping -c 3 192.168.202.1
ping -c 3 192.168.202.65
```

Allow the listed UDP flows through the host firewall. The primary's ordinary
telemetry connection uses host port 14540 separately from the HITL sensor links.

On **each** controller console, verify firmware revision, network identity,
`SYS_AUTOSTART`, `SYS_HITL`, `MAV_SYS_ID`, `MAV_COMP_ID`, `FT_N_SPARE_FCS`,
`mavlink status`, and `redundancy status`. For the two-controller X500 setup:

```text
param set SYS_AUTOSTART 1401
param set SYS_HITL 1
param set FT_N_SPARE_FCS 1
```

After confirming the assigned roles, set `MAV_COMP_ID` to `1` on the primary and
`2` on the secondary. Preserve the intended common vehicle system identity.
Save with `param save`, reboot each controller, and verify parameters, `.1` / `.65`
addresses, MAVLink channels and redundancy again. Board startup derives peer
addresses with `netconfig get_ipv4 1/2`; redundancy uses ports 14544/14545 with
reversed directions on the two controllers. Secure telemetry, when enabled,
requires its existing proxy configuration.

Hardware X500 HITL uses airframe **1401**. POSIX Gazebo SITL uses **4401**;
the hardware airframe with ID 4401 is a different vehicle. Consult the
[hardware airframe](../../../ROMFS/px4fmu_common/init.d/airframes/1401_ssrc_holybro_x500.hil)
and [FT MAVLink startup](../../../boards/ssrc/saluki-ft/init/rc.board_mavlink).
The simulator scripts do not configure, flash or reboot controllers.

## Start and stop FT simulation

```bash
./Tools/simulation/gz/start_ft_hitl.sh
./Tools/simulation/gz/stop_ft_hitl.sh
```

Start runs the FT X500 headlessly in the background, waits for model creation
and ownership of both HITL sockets, and prints the log path and Gazebo partition.
Stop can be called from another SSH session; it signals the recorded launcher
only after checking the process start time. The launcher cleans up its Gazebo
process group. Repeated starts retain the current managed session, and repeated
stops succeed when it is already stopped.

Logs and PID state are kept in `${XDG_STATE_HOME:-$HOME/.local/state}/px4-ft-hitl`.
Use the same `PX4_FT_HITL_STATE_DIR` override for both scripts if changing this
location. With the default location:

```bash
tail -f ~/.local/state/px4-ft-hitl/latest.log
```

For foreground operation, including a single-controller model:

```bash
./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_ft_hitl.sdf --headless
# Or, after stopping the FT instance:
./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf --headless
```

Use Ctrl-C to stop a foreground instance. Omit `--headless` for the integrated
GUI. It expands to `-s --headless-rendering`; camera rendering still requires
working EGL/OpenGL even without a display. The launcher uses `worlds/default.sdf`
and accepts verbosity and render-engine options, for example `-v 3` or
`--render-engine-server ogre2`. It rejects alternate worlds and server options.
The Python helper is part of this launcher: it handles bounded discovery,
spawning, HITL socket readiness and process cleanup.

Each launch gets a new `GZ_PARTITION` unless explicitly supplied. To attach a GUI,
use the partition printed by startup and source the build environment first:

```bash
source build/px4_sitl_default/rootfs/gz_env.sh
GZ_PARTITION=<printed-partition> gz sim -g
```

Partitions isolate Gazebo discovery, not UDP sockets. Stop other simulators using
ports 14542/14562 before launching. Model arguments may be absolute paths or
relative to `PX4_GZ_MODELS`; `PX4_FT_HITL_MODEL` overrides the background launcher's
model. Neither option changes controller configuration.

## Single-FC quad tailsitter

Prepare FC1 using [airframe 13200 and the tailsitter overrides](models/quadtailsitter/hitl_params.txt)
before connecting the simulator to hardware. This is a four-motor tailsitter,
not the X500 airframe. Stop the current managed instance before changing models:

```bash
./Tools/simulation/gz/stop_ft_hitl.sh
PX4_FT_HITL_MODEL=quadtailsitter/model_hitl.sdf \
  ./Tools/simulation/gz/start_ft_hitl.sh
```

The existing script names and state directory are shared with FT operation.
This model uses only FC1 (`192.168.202.1:14543`) and host port 14542, with
`FT_N_SPARE_FCS=0`. The marked default world and network configuration are reused.
Its HITL wrapper sets the starting pose to `(1.01, 0.98, -0.19)` with zero roll,
pitch and yaw, upright on the home pad. The model origin is below its lowest
collision surface; this height leaves 1 cm ground clearance and avoids tipping
over after a drop from the generic X500 spawn height. The launcher honors an
explicit SDF model pose and retains its existing default for models without one.
The launcher selects `third-person-camera` for this model and streams it to the
saved QGC video destination. The sensor is fixed to `base_link`, 3 m behind and
1.5 m above it, looking at the aircraft. It follows both position and attitude,
so the horizon rotates with the aircraft during transitions. The camera adds no
links, joints or mass. The initial parameter profile uses airspeedless,
pitch-based transitions; flight validation remains separate from startup.

The tailsitter profile sets `FD_ESC_TOUT=1000` ms for simulated ESC arming
feedback. With the 300 ms default, HITL1 recorded an ESC arming failure followed
by a failsafe disarm about 0.5 s after arming. On 2026-11-24, the operator
confirmed that increasing this timeout to 1000 ms resolved the immediate motor
stop. ESC arming checks remain enabled (`FD_ESCS_EN=1`).

On HITL1, `~/hitl.sh` selects this model. An explicit `PX4_FT_HITL_MODEL` overrides
that host default, for example `ssrc_holybro_x500/model_hitl.sdf` to return to the
single-FC X500 after stopping the previous session. Stop with
`~/saluki-hitl/stop-ft-gazebo.sh`. Neither launcher applies the parameter file or
arms the controller.

## Visual flight references

The default world has ground lines at 10 m spacing, four coloured landmarks,
and a yellow H in a 4 m outlined pad centred on the simulator spawn `(1.01, 0.98)`.
These are visual references without additional collisions. The H marks the launch
location; it does not follow a home position subsequently set by the autopilot.
The existing ground collision, physics and GPS origin are unchanged.

## Camera streaming to QGroundControl

Set the QGroundControl computer's address when starting:

```bash
PX4_VIDEO_HOST_IP=192.168.50.70 ./Tools/simulation/gz/start_ft_hitl.sh
```

The address is saved in the state directory's `video-host` file. Without a saved
address, startup uses the SSH client's address, falling back to `127.0.0.1` for a
local session. Set an explicit reachable address when using an SSH jump host.
For X500, `PX4_VIDEO_CAMERA` defaults to `onboard-gimbal-camera`, selecting the
gimbal view pitched 45 degrees below horizontal. Its sensor sits 0.2 m ahead of
the gimbal origin to clear the central body; pan/tilt controls remain available.
For quadtailsitter, the launcher defaults to `third-person-camera`. An explicit
`PX4_VIDEO_CAMERA` overrides either choice. In QGroundControl,
select **UDP h.264 Video Stream**, port **5600**, and allow video while disarmed.
Allow UDP 5600 on the receiving computer. Stop and restart to apply video settings
or a rebuilt camera plugin. The camera plugin uses pipeline-relative timestamps
from GStreamer's `appsrc do-timestamp` for playback.

## Troubleshooting and validation limits

- Missing `gz_env.sh` or libraries: build both plugin targets above. The HITL
  bridge lives in `build/px4_sitl_default/build_gz-sim_plugins`; standard PX4
  plugins live in `build/px4_sitl_default/src/modules/simulation/gz_plugins`.
  The launcher preserves both directories in the colon-separated plugin path.
- Busy HITL ports: inspect `ss -lunp` and stop the existing owner. Changing the
  Gazebo partition does not resolve a socket conflict.
- Spawn failure: inspect the log and `GZ_PARTITION=<printed-partition> gz service -l`.
  The launcher requires `/world/default/create`, a successful create response,
  and the launched simulator's HITL UDP sockets before reporting readiness.
- SSH display failures: use `--headless`. For rendering failures, verify GPU/EGL
  availability and actual camera frames; successful spawning does not prove video
  delivery. Software rendering can be selected with `LIBGL_ALWAYS_SOFTWARE=1`.

The earlier host setup on `pmk-server-hitl2` used Ubuntu 24.04, Harmonic 8.15,
the same Ethernet addresses and hardware airframe 1401. Loopback model startup
was exercised there during 16–19 November 2026. That history does not validate
this proposal revision. Recheck startup, both controller links and camera delivery
on the target host. Hardware flight and primary-failure/takeover behavior require
separate validation; successful launch is not evidence of FT failover.
