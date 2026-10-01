#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: bash %s [NEW_SESSION_DIRECTORY]\n' "${0##*/}"
    printf 'Record all discovered ROS 2 topics until Ctrl+C. Source ROS and the workspace first.\n'
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
if [[ $# -gt 1 || "${1:-}" == -* ]]; then
    usage >&2
    exit 2
fi
if ! command -v ros2 >/dev/null 2>&1; then
    printf 'ros2 not found. Source /opt/ros/humble/setup.bash and ros_ws/install/setup.bash.\n' >&2
    exit 1
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)
session_dir=${1:-"$repo_dir/logs/vehicle_$(date -u +%Y%m%dT%H%M%SZ)_$$"}
mkdir -p -- "$(dirname -- "$session_dir")"
if ! mkdir -- "$session_dir"; then
    printf 'Choose a NEW session directory; existing captures are never overwritten.\n' >&2
    exit 1
fi
session_dir=$(cd -- "$session_dir" && pwd)

{
    printf 'Started UTC: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'Host: %s\nWorking directory: %s\nRepository: %s\n' "$(hostname)" "$PWD" "$repo_dir"
    printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\nROS_LOCALHOST_ONLY=%s\n' \
        "${ROS_DISTRO:-unset}" "${ROS_DOMAIN_ID:-unset (default 0)}" \
        "${RMW_IMPLEMENTATION:-unset (ROS default)}" "${ROS_LOCALHOST_ONLY:-unset}"
    printf 'SDSM_BIND_IP=%s\nSDSM_PORT=%s\n' "${SDSM_BIND_IP:-192.168.31.10}" "${SDSM_PORT:-9010}"
    printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-unset}"
    git -C "$repo_dir" log -1 --format='Commit: %H%nBranch commit: %s' || true
    git -C "$repo_dir" status --short --branch || true
    df -h "$session_dir"
} > "$session_dir/session.txt" 2>&1
git -C "$repo_dir" diff HEAD > "$session_dir/worktree.patch" 2>/dev/null || true
export ROS_LOG_DIR="$session_dir/recorder_ros_logs"
mkdir -p -- "$ROS_LOG_DIR"

record_command=(ros2 bag record --all --include-hidden-topics --storage sqlite3
    --max-bag-size 1073741824 --output "$session_dir/bag")
printf '%q ' "${record_command[@]}" > "$session_dir/record_command.txt"
printf '\n' >> "$session_dir/record_command.txt"
printf 'Session: %s\nRecording all topics, including late publishers and hidden topics.\n' "$session_dir"
printf 'Stop with Ctrl+C and wait for the recorder to exit BEFORE stopping the vehicle launcher.\n'
printf 'Run vehicle_diagnostics.sh in another terminal; see logger.md for capture and replay.\n'
exec "${record_command[@]}" > >(tee -i "$session_dir/recorder.log") 2>&1