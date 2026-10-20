#!/usr/bin/env bash
# Start the four background services required by the handover main program.
# The main program itself is intentionally not started here.

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT" || exit 1
# ROS setup scripts reference unset variables, so enable nounset afterwards.
# shellcheck disable=SC1091
source ./activate_handover.bash
set -u

mkdir -p logs
PIDFILE="$ROOT/logs/handover_services.pid"

# Refuse to add a second copy while services from this script are alive.
if [ -f "$PIDFILE" ]; then
    active=0
    while read -r name pid; do
        [ -n "${pid:-}" ] || continue
        if kill -0 "$pid" 2>/dev/null; then
            echo "still running: $name (pid $pid)"
            active=1
        fi
    done < "$PIDFILE"
    if [ "$active" = 1 ]; then
        echo "Services from a previous ./start_handover.sh are still running." >&2
        echo "Run ./stop_handover.sh first." >&2
        exit 1
    fi
    rm -f "$PIDFILE"
fi
: > "$PIDFILE"

joint_states_ready() {
    timeout 6 ros2 topic echo --once --qos-reliability best_effort \
        /joint_states >/dev/null 2>&1
}

node_running() {
    ros2 node list 2>/dev/null | grep -q "/$1"
}

tf_chain_ready() {
    # PYTHONUNBUFFERED is required: the CLI buffers its output when piped.
    PYTHONUNBUFFERED=1 timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" \
        2>/dev/null | grep -q "Translation"
}

start_service() {
    local name="$1"
    shift
    local log="$ROOT/logs/${name}.log"
    local child_pidfile="$ROOT/logs/.${name}.pid"
    rm -f "$child_pidfile"
    # setsid gives the service its own process group so stop_handover.sh can
    # signal ros2 launch and ros2 run together with the processes they spawn.
    setsid bash -c 'echo $$ > "$1"; shift; exec "$@"' \
        bash "$child_pidfile" "$@" >"$log" 2>&1 &
    local i
    for i in $(seq 1 50); do
        [ -s "$child_pidfile" ] && break
        sleep 0.1
    done
    local pid
    pid="$(cat "$child_pidfile" 2>/dev/null || true)"
    rm -f "$child_pidfile"
    if [ -z "$pid" ]; then
        echo "failed to start $name (see $log)" >&2
        return 1
    fi
    printf '%s %s\n' "$name" "$pid" >> "$PIDFILE"
    echo "started: $name (pid $pid, log $log)"
}

wait_until() {
    local what="$1"
    local timeout_s="$2"
    shift 2
    local deadline=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if "$@"; then
            echo "ready: $what"
            return 0
        fi
        sleep 1
    done
    echo "timed out waiting for $what" >&2
    return 1
}

fail=0

if joint_states_ready; then
    echo "already running: UR driver"
elif start_service driver \
        ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur3e robot_ip:=192.168.1.10 launch_rviz:=false \
        kinematics_params_file:="$ROOT/config/ur3e_factory_calibration.yaml"; then
    wait_until "UR driver (/joint_states)" 90 joint_states_ready || fail=1
else
    fail=1
fi

if node_running robotiq_2f_urcap_adapter; then
    echo "already running: gripper adapter"
elif start_service gripper \
        ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py \
        --ros-args -p robot_ip:=192.168.1.10; then
    wait_until "gripper adapter" 30 node_running robotiq_2f_urcap_adapter || fail=1
else
    fail=1
fi

if tf_chain_ready tool0 zed_inhand_camera_frame_optical; then
    echo "already running: ZED extrinsic"
elif start_service zed-tf \
        ros2 launch calibration_result eye_in_hand_zed2i.launch.py; then
    wait_until "ZED extrinsic" 15 \
        tf_chain_ready tool0 zed_inhand_camera_frame_optical || fail=1
else
    fail=1
fi

if tf_chain_ready base_link rs_tohand_color_optical_frame; then
    echo "already running: D435i extrinsic"
elif start_service d435-tf \
        ros2 launch calibration_result eye_to_hand_rs_d435i.launch.py; then
    wait_until "D435i extrinsic" 15 \
        tf_chain_ready base_link rs_tohand_color_optical_frame || fail=1
else
    fail=1
fi

if [ "$fail" = 0 ]; then
    wait_until "TF base_link -> zed_inhand_camera_frame_optical" 15 \
        tf_chain_ready base_link zed_inhand_camera_frame_optical || fail=1
fi
if [ "$fail" = 0 ]; then
    wait_until "TF base_link -> rs_tohand_color_optical_frame" 15 \
        tf_chain_ready base_link rs_tohand_color_optical_frame || fail=1
fi

if [ "$fail" != 0 ]; then
    echo "startup incomplete; recent logs:" >&2
    for name in driver gripper zed-tf d435-tf; do
        [ -f "$ROOT/logs/$name.log" ] || continue
        echo "--- logs/$name.log (last 15 lines) ---" >&2
        tail -n 15 "$ROOT/logs/$name.log" >&2
    done
    exit 1
fi

cat <<'EOF'
All services are ready.

Run the main program in this terminal (load the environment once per terminal):

  source activate_handover.bash
  python -u src/graspnet-baseline/zedInhand_rsTohand/robot2human_handover.py \
    --manual --grasp-only --debug 2>&1 | tee logs/grasp_test.log

Remove --grasp-only for the full handover. Stop the services with ./stop_handover.sh
after the main program has exited. Only services started by this script are recorded
there; services started manually must be stopped in their own terminals.
EOF
