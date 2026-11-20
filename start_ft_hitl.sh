#!/bin/bash
# Start a HITL model in the background; stop it with stop_ft_hitl.sh.
set -euo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: $0 [gz sim options]"
    echo "Starts the selected HITL model headlessly (default: FT X500). GZ_PARTITION is optional."
    echo "PX4_FT_HITL_STATE_DIR overrides the PID/log directory."
    echo "PX4_FT_HITL_MODEL selects model/model_hitl.sdf or an absolute SDF."
    echo "For models with a camera, PX4_VIDEO_HOST_IP is saved between starts and defaults to the SSH client; UDP port is 5600."
    echo "PX4_VIDEO_CAMERA defaults to third-person-camera for quadtailsitter, otherwise onboard-gimbal-camera."
    exit 0
fi
SCRIPT_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
STATE_DIR=${PX4_FT_HITL_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/px4-ft-hitl}
PID_FILE="$STATE_DIR/launcher.pid"
MODEL=${PX4_FT_HITL_MODEL:-ssrc_holybro_x500/model_ft_hitl.sdf}
mkdir -p -- "$STATE_DIR"
exec 9>"$STATE_DIR/control.lock"
flock -w 90 9

process_alive() {
    local stat fields
    [[ $PID =~ ^[0-9]+$ && $PID -gt 1 && $START_TIME =~ ^[0-9]+$ ]] || return 1
    IFS= read -r stat 2>/dev/null < "/proc/$PID/stat" || return 1
    read -r -a fields <<< "${stat##*) }"
    [[ ${fields[0]:-Z} != Z && ${fields[19]:-} == "$START_TIME" ]]
}

# Remember the recipient so later SSH sessions keep the same QGC destination.
if [[ -z ${PX4_VIDEO_HOST_IP:-} && -s "$STATE_DIR/video-host" ]]; then
    IFS= read -r PX4_VIDEO_HOST_IP < "$STATE_DIR/video-host"
fi
if [[ -z ${PX4_VIDEO_HOST_IP:-} && -n ${SSH_CONNECTION:-} ]]; then
    read -r PX4_VIDEO_HOST_IP _ <<< "$SSH_CONNECTION"
fi
export PX4_VIDEO_HOST_IP=${PX4_VIDEO_HOST_IP:-127.0.0.1}
case "$MODEL" in
    quadtailsitter/*|*/quadtailsitter/*) DEFAULT_CAMERA=third-person-camera ;;
    *) DEFAULT_CAMERA=onboard-gimbal-camera ;;
esac
export PX4_VIDEO_CAMERA=${PX4_VIDEO_CAMERA:-$DEFAULT_CAMERA}
printf '%s\n' "$PX4_VIDEO_HOST_IP" > "$STATE_DIR/video-host"

if [[ -f "$PID_FILE" ]]; then
    read -r PID START_TIME < "$PID_FILE" || { echo "Invalid PID record: $PID_FILE" >&2; exit 1; }
    if process_alive; then
        echo "HITL Gazebo is already running (PID $PID). Log: $STATE_DIR/latest.log"
        echo "Saved video destination: $PX4_VIDEO_HOST_IP:5600. Stop/start to apply any changes."
        exit 0
    fi
    rm -- "$PID_FILE"
fi

export GZ_DISTRO=harmonic
export GZ_PARTITION=${GZ_PARTITION:-px4-ft-$(python3 -c 'import uuid; print(uuid.uuid4().hex)')}
LOG_FILE="$STATE_DIR/gazebo-$(date -u +%Y%m%dT%H%M%SZ)-$$.log"
ln -sfn -- "$LOG_FILE" "$STATE_DIR/latest.log"
printf '%s\n' "$GZ_PARTITION" > "$STATE_DIR/partition"
nohup "$SCRIPT_DIR/hitl_run.sh" \
    "$MODEL" --headless "$@" \
    >"$LOG_FILE" 2>&1 < /dev/null 9>&- &
PID=$!
START_TIME=''
READY=0
# Called indirectly by the EXIT trap.
# shellcheck disable=SC2329
cleanup_failed_start() {
    if [[ $READY == 0 ]]; then
        if process_alive; then
            kill -TERM "$PID" 2>/dev/null || true
            for ((attempt=0; attempt<100; attempt++)); do
                process_alive || break
                sleep 0.1
            done
        fi
        if ! process_alive; then
            rm -f -- "$PID_FILE"
            wait "$PID" 2>/dev/null || true
        else
            echo "Launcher still running; use $SCRIPT_DIR/stop_ft_hitl.sh. PID record retained." >&2
        fi
    fi
}
trap 'cleanup_failed_start' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if ! IFS= read -r stat 2>/dev/null < "/proc/$PID/stat"; then
    echo "HITL launcher exited. See $LOG_FILE" >&2
    exit 1
fi
read -r -a fields <<< "${stat##*) }"
START_TIME=${fields[19]}
printf '%s %s\n' "$PID" "$START_TIME" > "$PID_FILE"

for ((attempt=0; attempt<65; attempt++)); do
    if ! process_alive; then
        echo "HITL startup failed. Log: $LOG_FILE" >&2
        tail -n 20 -- "$LOG_FILE" >&2
        exit 1
    fi
    if grep -q '^HITL model created:' "$LOG_FILE"; then
        READY=1
        echo "HITL Gazebo started (PID $PID)."
        echo "Model: $MODEL"
        echo "GZ_PARTITION=$GZ_PARTITION"
        echo "Video target (models with a camera): $PX4_VIDEO_CAMERA -> $PX4_VIDEO_HOST_IP:5600 (QGC UDP H.264)."
        echo "Log: $LOG_FILE"
        exit 0
    fi
    sleep 1
done
echo "HITL model startup timed out. Log: $LOG_FILE" >&2
tail -n 20 -- "$LOG_FILE" >&2
exit 1
