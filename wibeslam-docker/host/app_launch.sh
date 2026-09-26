#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /uros_ws/install/setup.bash 2>/dev/null || true
MODE=${MODE:-udp}
AGENT_PORT=${AGENT_PORT:-8090}
CFG_LUA=${CFG_LUA:-palmslam_2d_app}
PIDS=()

cleanup() {
    echo -e "\n=== Stopping app stack ==="
    for pid in "${PIDS[@]}"; do
        [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && kill "$pid" 2>/dev/null || true
    done
    echo "=== All stopped ==="
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[1/7] micro-ROS agent ($MODE, port $AGENT_PORT)..."
if [ "$MODE" = "serial" ]; then
    DEVICE=${DEVICE:-/dev/ttyACM0}
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -v2 > /tmp/agent.log 2>&1 &
else
    ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT" -v2 > /tmp/agent.log 2>&1 &
fi
PIDS+=($!)
sleep 3

echo "[2/7] host_time_publisher..."
python3 /root/host_time_publisher.py > /tmp/host_time.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[3/7] Static TFs..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 3.1416 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser > /tmp/tf.log 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id imu_link > /tmp/tf_imu.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[4/7] Cartographer ($CFG_LUA.lua)..."
ros2 run cartographer_ros cartographer_node \
    --configuration_directory /root/palmslam_cartographer \
    --configuration_basename palmslam_2d.lua > /tmp/cartographer.log 2>&1 &
PIDS+=($!)

echo "[5/7] Occupancy grid node..."
ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args -p publish_period_sec:=0.3 > /tmp/occgrid.log 2>&1 &
PIDS+=($!)

echo "[6/7] rosbridge_websocket :9090 + app topics bridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
PIDS+=($!)
python3 /root/app_topics_bridge.py > /tmp/app_bridge.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[7/7] Foxglove bridge :8765..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /tmp/foxglove.log 2>&1 &
PIDS+=($!)

echo -e "\n=== APP stack running ==="
echo "Phone app: connect ws://<host-ip>:9090 (ROS2)"
echo "Logs: tail -f /tmp/*.log   | Ctrl+C to stop"
wait