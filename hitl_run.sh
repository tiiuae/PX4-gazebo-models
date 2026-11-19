#!/bin/bash
set -eo pipefail

# Run from any directory. Full procedure: SALUKI_FT_HITL.md beside this script.
# ./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_ft_hitl.sdf --headless
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
BUILD_DIR="$WORKSPACE/build/px4_sitl_default"
HITL_PLUGIN_DIR="$BUILD_DIR/build_gz-sim_plugins"
MODEL_PATH=${1:-}

if [ -z "$MODEL_PATH" ]; then
    echo "Usage: $0 model/model_hitl.sdf [--headless] [gz sim options]" >&2
    exit 1
fi
shift
GZ_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --headless) GZ_ARGS+=(-s --headless-rendering) ;;
        *) GZ_ARGS+=("$arg") ;;
    esac
done

if [ ! -f "$BUILD_DIR/rootfs/gz_env.sh" ]; then
    echo "Missing $BUILD_DIR/rootfs/gz_env.sh; build with GZ_DISTRO=harmonic make px4_sitl_default gzsim-plugins px4_gz_plugins mavsdk_tests" >&2
    exit 1
fi
# The generated environment appends standard PX4 plugins and preserves inherited paths.
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HITL_PLUGIN_DIR}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

if [ ! -f "$HITL_PLUGIN_DIR/libmavlink_hitl_gazebosim.so" ]; then
    echo "Missing HITL bridge in $HITL_PLUGIN_DIR; run GZ_DISTRO=harmonic make px4_sitl_default gzsim-plugins" >&2
    exit 1
fi
for plugin in libOpticalFlowSystem.so libGstCameraSystem.so; do
    if [ ! -f "$PX4_GZ_PLUGINS/$plugin" ]; then
        echo "Missing $PX4_GZ_PLUGINS/$plugin; run GZ_DISTRO=harmonic make px4_sitl_default px4_gz_plugins" >&2
        exit 1
    fi
done
if [ ! -f "$GZ_SIM_SERVER_CONFIG_PATH" ]; then
    echo "Missing server configuration: $GZ_SIM_SERVER_CONFIG_PATH; regenerate the build environment" >&2
    exit 1
fi
case "$MODEL_PATH" in
    /*) MODEL="$MODEL_PATH" ;;
    *) MODEL="$PX4_GZ_MODELS/$MODEL_PATH" ;;
esac
if [ ! -f "$MODEL" ] || [ ! -f "$PX4_GZ_WORLDS/default.sdf" ]; then
    echo "Model or default world missing: $MODEL; initialize Tools/simulation/gz submodules" >&2
    exit 1
fi
command -v gz >/dev/null || { echo "Install Gazebo Harmonic (gz-harmonic)" >&2; exit 1; }
exec python3 "$SCRIPT_DIR/hitl_session.py" launch --model "$MODEL" \
    --world "$PX4_GZ_WORLDS/default.sdf" -- "${GZ_ARGS[@]}"
