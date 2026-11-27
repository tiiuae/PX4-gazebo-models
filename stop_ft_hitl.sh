#!/bin/bash
# Stop only the HITL launcher recorded by start_ft_hitl.sh; it cleans up Gazebo.
set -euo pipefail

if [[ $# -gt 0 ]]; then
    echo "Usage: $0"
    [[ $1 == --help || $1 == -h ]] && exit 0
    exit 1
fi
STATE_DIR=${PX4_FT_HITL_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/px4-ft-hitl}
PID_FILE="$STATE_DIR/launcher.pid"
if [[ ! -d "$STATE_DIR" ]]; then
    echo "Managed HITL Gazebo is not running."
    exit 0
fi
exec 9>"$STATE_DIR/control.lock"
flock -w 90 9
if [[ ! -f "$PID_FILE" ]]; then
    echo "Managed HITL Gazebo is not running."
    exit 0
fi
read -r PID START_TIME < "$PID_FILE" || { echo "Invalid PID record: $PID_FILE" >&2; exit 1; }
if [[ ! $PID =~ ^[0-9]+$ || $PID -le 1 || ! $START_TIME =~ ^[0-9]+$ ]]; then
    echo "Invalid PID record: $PID_FILE" >&2
    exit 1
fi
process_alive() {
    local stat fields
    IFS= read -r stat 2>/dev/null < "/proc/$PID/stat" || return 1
    read -r -a fields <<< "${stat##*) }"
    [[ ${fields[0]:-Z} != Z && ${fields[19]:-} == "$START_TIME" ]]
}
if ! process_alive; then
    rm -- "$PID_FILE"
    echo "Managed HITL Gazebo is not running; removed stale PID record."
    exit 0
fi
kill -TERM "$PID"
for ((attempt=0; attempt<100; attempt++)); do
    if ! process_alive; then
        rm -- "$PID_FILE"
        echo "HITL Gazebo stopped. Log: $STATE_DIR/latest.log"
        exit 0
    fi
    sleep 0.1
done
echo "HITL launcher did not stop within 10 seconds. PID record retained: $PID_FILE" >&2
exit 1
