#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
test_dir=$(mktemp -d)
trap 'rm -rf -- "$test_dir"' EXIT
export MOCK_CALLS="$test_dir/calls.txt"

ros2() {
    printf '%s\n' "$*" >> "$MOCK_CALLS"
    case "$1 $2" in
        'bag record')
            if [[ "${MOCK_RECORD_FAIL:-0}" == 1 ]]; then
                printf 'simulated recorder failure\n' >&2
                return 23
            fi
            while [[ $# -gt 0 ]]; do
                if [[ "$1" == --output ]]; then
                    mkdir -- "$2"
                    printf 'mock bag\n' > "$2/metadata.yaml"
                    break
                fi
                shift
            done
            ;;
        'node list') printf '/test_node\n' ;;
        'param dump') printf '/test_node:\n  ros__parameters:\n    use_sim_time: false\n' ;;
        'topic echo') printf 'data: sample\n' ;;
        'topic hz') printf 'average rate: 10.0\n'; return 124 ;;
        'topic info') printf 'Publisher count: 1\n' ;;
        *) printf 'mock ROS output\n' ;;
    esac
}
timeout() {
    shift 2
    "$@"
}
tar() {
    printf 'mock archive\n' > "$2"
    if [[ " $* " == *' -T - '* ]]; then
        cat > /dev/null
    fi
}
export -f ros2 timeout tar
mkdir -- "$test_dir/bin"
printf '#!/usr/bin/env bash\nros2 "$@"\n' > "$test_dir/bin/ros2"
chmod +x "$test_dir/bin/ros2"
export PATH="$test_dir/bin:$PATH"

expect_exit() {
    local expected=$1
    shift
    local result=0
    "$@" > "$test_dir/last_output.txt" 2>&1 || result=$?
    if [[ "$result" -ne "$expected" ]]; then
        printf 'Expected exit %s, got %s: %s\n' "$expected" "$result" "$*" >&2
        cat "$test_dir/last_output.txt" >&2
        exit 1
    fi
}

for script in record_vehicle_bag.sh vehicle_diagnostics.sh; do
    bash -n "$script_dir/$script"
    expect_exit 0 bash "$script_dir/$script" --help
    expect_exit 2 bash "$script_dir/$script" --invalid
done

expect_exit 0 bash "$script_dir/record_vehicle_bag.sh" "$test_dir/session with spaces"
[[ -f "$test_dir/session with spaces/bag/metadata.yaml" ]]
[[ -f "$test_dir/session with spaces/session.txt" ]]
[[ -f "$test_dir/session with spaces/worktree.patch" ]]
grep -q -- '--all --include-hidden-topics --storage sqlite3 --max-bag-size 1073741824' "$MOCK_CALLS"
expect_exit 1 bash "$script_dir/record_vehicle_bag.sh" "$test_dir/session with spaces"
export MOCK_RECORD_FAIL=1
expect_exit 23 bash "$script_dir/record_vehicle_bag.sh" "$test_dir/failed_session"
unset MOCK_RECORD_FAIL

mkdir -- "$test_dir/launch" "$test_dir/ros_logs"
printf 'OBU fixture\n' > "$test_dir/launch/LOG_obu_interface_node2.log"
printf 'GPS fixture\n' > "$test_dir/launch/RAW-GPS.csv"
printf 'ROS fixture\n' > "$test_dir/ros_logs/launch.log"
export ROS_LOG_DIR="$test_dir/ros_logs"
export SAMPLE_SECONDS=1
expect_exit 0 bash "$script_dir/vehicle_diagnostics.sh" "$test_dir/diagnostics with spaces" "$test_dir/launch"
[[ -f "$test_dir/diagnostics with spaces/launch_logs/LOG_obu_interface_node2.log" ]]
[[ -f "$test_dir/diagnostics with spaces/launch_logs/RAW-GPS.csv" ]]
[[ -f "$test_dir/diagnostics with spaces/ros_logs_last_2h.tar.gz" ]]
[[ -f "$test_dir/diagnostics with spaces/config_and_messages.tar.gz" ]]
grep -q 'use_sim_time: false' "$test_dir/diagnostics with spaces/parameters/_test_node.txt"
grep -q 'data: sample' "$test_dir/diagnostics with spaces/topics/infra_local_measures.echo.txt"
grep -q 'Exit: 124' "$test_dir/diagnostics with spaces/topics/infra_local_measures.hz.txt"
grep -q 'topic info /fg_fused_topic --verbose' "$MOCK_CALLS"
grep -q 'topic info /fused_actor_states --verbose' "$MOCK_CALLS"
expect_exit 1 bash "$script_dir/vehicle_diagnostics.sh" "$test_dir/diagnostics with spaces" "$test_dir/launch"
export SAMPLE_SECONDS=0
expect_exit 2 bash "$script_dir/vehicle_diagnostics.sh" "$test_dir/invalid" "$test_dir/launch"
[[ ! -e "$test_dir/invalid" ]]

printf 'Vehicle logging smoke tests passed (mock ROS, timeout, and archive commands).\n'