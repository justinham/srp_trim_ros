#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: bash %s [NEW_OUTPUT_DIRECTORY] [VEHICLE_LAUNCH_DIRECTORY]\n' "${0##*/}"
    printf 'Snapshot ROS, network, topic samples, parameters, configuration, and existing logs.\n'
    printf 'Run on the Linux vehicle computer with ROS sourced. SAMPLE_SECONDS defaults to 10.\n'
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
if [[ $# -gt 2 || "${1:-}" == -* ]]; then
    usage >&2
    exit 2
fi
sample_seconds=${SAMPLE_SECONDS:-10}
if [[ ! "$sample_seconds" =~ ^[1-9][0-9]*$ ]]; then
    printf 'SAMPLE_SECONDS must be a positive integer.\n' >&2
    exit 2
fi
for required in ros2 timeout tar; do
    if ! command -v "$required" >/dev/null 2>&1; then
        printf 'Required command missing: %s (use the sourced Linux ROS 2 environment).\n' "$required" >&2
        exit 1
    fi
done

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)
launch_dir=$(cd -- "${2:-$PWD}" && pwd)
output_dir=${1:-"$repo_dir/logs/diagnostics_$(date -u +%Y%m%dT%H%M%SZ)_$$"}
mkdir -p -- "$(dirname -- "$output_dir")"
if ! mkdir -- "$output_dir"; then
    printf 'Choose a NEW output directory; existing diagnostics are never overwritten.\n' >&2
    exit 1
fi
output_dir=$(cd -- "$output_dir" && pwd)
mkdir -p -- "$output_dir/topics" "$output_dir/nodes" "$output_dir/parameters" "$output_dir/launch_logs"
printf 'Diagnostics: %s\n' "$output_dir"

run_check() {
    local destination=$1
    shift
    {
        printf 'UTC: %s\nCommand: ' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        printf '%q ' "$@"
        printf '\n'
        if timeout --kill-after=2s 8s "$@"; then
            printf '\nExit: 0\n'
        else
            printf '\nExit: %s (124 means timeout; command failure is diagnostic evidence)\n' "$?"
        fi
    } > "$output_dir/$destination" 2>&1
}

{
    printf 'Started UTC: %s\nHost: %s\nLaunch directory: %s\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(hostname)" "$launch_dir"
    for variable in ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY \
        ROS_LOG_DIR ROS_HOME AMENT_PREFIX_PATH CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE \
        FASTDDS_DEFAULT_PROFILES_FILE SDSM_BIND_IP SDSM_PORT; do
        printf '%s=%s\n' "$variable" "${!variable:-unset}"
    done
    git -C "$repo_dir" log -1 --format='Commit: %H%nSubject: %s' || true
    git -C "$repo_dir" status --short --branch || true
    df -h "$output_dir"
} > "$output_dir/session.txt" 2>&1
git -C "$repo_dir" diff HEAD > "$output_dir/worktree.patch" 2>/dev/null || true

run_check ip_addresses.txt ip address show
run_check ip_routes.txt ip route show table all
run_check link_counters.txt ip -details -statistics link show
run_check sockets.txt ss -tunap
run_check processes.txt ps -eo pid,ppid,lstart,args
run_check clock.txt timedatectl status
run_check clock_sources.txt chronyc tracking
run_check ros_packages.txt ros2 pkg list
run_check nodes.txt ros2 node list
run_check topics.txt ros2 topic list --include-hidden-topics -t

while IFS= read -r node; do
    [[ "$node" =~ ^/[a-zA-Z0-9_/]+$ ]] || continue
    node_tag=${node//\//_}
    run_check "nodes/$node_tag.txt" ros2 node info "$node"
    run_check "parameters/$node_tag.txt" ros2 param dump "$node"
done < "$output_dir/nodes.txt"

topics=(/infra_local_measures /infra_global_measures /gps /gps_can /ego_states
    /track /tracked_object_list /ego_local_measures /ego_global_measures
    /matched_ego_actors /matched_infra_actors /ego_actor_states /infra_actor_states
    /fused_actor_states /fg_fused_topic /app_cmd /selected_topic /ego_turn_signal
    /ego_waypoints /actors_waypoints /interpolated_wps /obstacles_id_data
    /ego_speed_cmd /vehicle_command /tf /tf_static /clock /rosout)
for topic in "${topics[@]}"; do
    run_check "topics/${topic#/}.info.txt" ros2 topic info "$topic" --verbose
done

sample_topics=(/infra_local_measures /infra_global_measures /gps /gps_can /ego_states
    /tracked_object_list /fused_actor_states /fg_fused_topic)
sample_pids=()
sample_files=()
stop_samples() {
    for sample_pid in "${sample_pids[@]}"; do
        kill -TERM "$sample_pid" 2>/dev/null || true
    done
    wait || true
    exit 130
}
trap stop_samples INT TERM
printf 'Sampling %s checkpoint topics for %ss (no sensor or control data is published).\n' \
    "${#sample_topics[@]}" "$sample_seconds"
for topic in "${sample_topics[@]}"; do
    for sample_kind in echo hz; do
        sample_file="$output_dir/topics/${topic#/}.$sample_kind.txt"
        if [[ "$sample_kind" == echo ]]; then
            sample_command=(ros2 topic echo "$topic" --once --qos-reliability best_effort)
        else
            sample_command=(ros2 topic hz "$topic" --window 100)
        fi
        printf 'Started UTC: %s; timeout: %ss\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$sample_seconds" > "$sample_file"
        PYTHONUNBUFFERED=1 timeout --kill-after=2s "${sample_seconds}s" "${sample_command[@]}" >> "$sample_file" 2>&1 &
        sample_pids+=("$!")
        sample_files+=("$sample_file")
    done
done
for sample_index in "${!sample_pids[@]}"; do
    sample_result=0
    wait "${sample_pids[$sample_index]}" || sample_result=$?
    printf '\nExit: %s (124 is expected for bounded hz sampling or no message before timeout)\n' \
        "$sample_result" >> "${sample_files[$sample_index]}"
done
trap - INT TERM

shopt -s nullglob
for log_file in "$launch_dir"/LOG_*.log "$launch_dir"/*gps*log* "$launch_dir"/log*.csv \
    "$launch_dir"/log*.txt "$launch_dir"/RAW-GPS.csv "$launch_dir"/fusion.csv "$launch_dir"/wpsfromjustin.log; do
    [[ -f "$log_file" ]] || continue
    cp -p -- "$log_file" "$output_dir/launch_logs/" 2>> "$output_dir/copy_errors.txt" || true
done
ros_log_root=${ROS_LOG_DIR:-"${ROS_HOME:-$HOME/.ros}/log"}
printf 'ROS log source: %s\n' "$ros_log_root" >> "$output_dir/session.txt"
if [[ -d "$ros_log_root" ]]; then
    (
        cd -- "$ros_log_root"
        find . -type f -mmin -120 -print0 | tar -czf "$output_dir/ros_logs_last_2h.tar.gz" --null -T -
    ) 2>> "$output_dir/copy_errors.txt" || true
fi
tar -czf "$output_dir/config_and_messages.tar.gz" -C "$repo_dir" \
    Data/Intersections Data/offline_path_files DBC-ARXML ros_ws/src/fusion/msg \
    2>> "$output_dir/copy_errors.txt" || true
printf 'Finished UTC: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$output_dir/session.txt"
printf 'Saved diagnostics to %s\nReview command Exit fields and copy_errors.txt for incomplete captures.\n' "$output_dir"