#!/usr/bin/env bash
# Stop handover services.
#   ./stop_handover.sh        stop services recorded by start_handover.sh
#   ./stop_handover.sh --all  stop every handover process of this project,
#                             including ones started manually
set -u

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="$ROOT/logs/handover_services.pid"

MAIN_PATTERN='[z]edInhand_rsTohand|[r]obot2human_handover.py|[h]uman2robot_handover.py'
DRIVER_PATTERN='[u]r_control.launch.py'
SERVICE_PATTERN='[r]obotiq_2f_adapter_node.py|[e]ye_in_hand_zed2i.launch.py|[e]ye_to_hand_rs_d435i.launch.py|[s]tatic_transform_publisher.*zed_inhand_camera_frame_optical|[s]tatic_transform_publisher.*rs_tohand_color_optical_frame'
LEFTOVER_PATTERN='[u]r_control.launch.py|[u]r_ros2_control_node|[r]obotiq_2f_adapter_node.py|[e]ye_in_hand_zed2i.launch.py|[e]ye_to_hand_rs_d435i.launch.py|[s]tatic_transform_publisher.*zed_inhand_camera_frame_optical|[s]tatic_transform_publisher.*rs_tohand_color_optical_frame'
ALL_PATTERN="$MAIN_PATTERN|$LEFTOVER_PATTERN"

usage() {
    cat <<'EOF'
Usage: ./stop_handover.sh [--all]

  (no argument)  Stop the driver, gripper and TF publishers recorded in
                 logs/handover_services.pid by start_handover.sh.
  --all          Stop every handover process of this project, including
                 processes that were started manually.

Do not run either mode while the robot is holding an object: the gripper
adapter resets activation when it exits.
EOF
}

case "${1:-}" in
    "") ;;
    --all) ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

echo "WARNING: do not stop while the robot is holding an object;"
echo "the gripper adapter resets activation when it exits."

stop_one() {
    local name="$1"
    local pid="$2"
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "already stopped: $name (pid $pid)"
        return 0
    fi
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    local i
    for i in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || { echo "stopped: $name"; return 0; }
        sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    for i in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || { echo "terminated: $name"; return 0; }
        sleep 0.2
    done
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    echo "killed: $name"
}

stop_all() {
    # Main program first so it cannot keep commanding the robot or gripper.
    pkill -INT -f "$MAIN_PATTERN" 2>/dev/null
    sleep 1
    pkill -INT -f "$DRIVER_PATTERN" 2>/dev/null
    sleep 2
    pkill -INT -f "$SERVICE_PATTERN" 2>/dev/null
    sleep 1
    pkill -TERM -f "$LEFTOVER_PATTERN" 2>/dev/null
    sleep 1
    pkill -KILL -f "$LEFTOVER_PATTERN" 2>/dev/null
    sleep 1
    if command -v ros2 >/dev/null 2>&1; then
        ros2 daemon stop >/dev/null 2>&1
    fi
    if pgrep -f "$ALL_PATTERN" >/dev/null 2>&1; then
        echo "still running:" >&2
        pgrep -af "$ALL_PATTERN" >&2
        return 1
    fi
    echo "all handover processes stopped"
    return 0
}

if [ "${1:-}" = "--all" ]; then
    stop_all
    status=$?
    rm -f "$PIDFILE"
    exit "$status"
fi

if [ ! -f "$PIDFILE" ]; then
    echo "No pid file at $PIDFILE."
    echo "Services started manually must be stopped with Ctrl+C in their own terminals,"
    echo "or run ./stop_handover.sh --all to stop every handover process."
    exit 0
fi

tac "$PIDFILE" | while read -r name pid; do
    [ -n "${pid:-}" ] || continue
    stop_one "$name" "$pid"
done

rm -f "$PIDFILE"
