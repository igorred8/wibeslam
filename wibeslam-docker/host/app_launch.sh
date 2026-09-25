#!/bin/bash
# Полный APP-стек, повторяющий последовательность Yahboom PalmSLAM:
#   micro-ROS agent -> статические TF -> cartographer_node ->
#   occupancy_grid_node -> rosbridge_websocket (точка входа для
#   мобильного приложения «ROS Robot») -> foxglove bridge (:8765).
#
# Переменные окружения:
#   MODE           = udp (по умолчанию) | serial — транспорт до платы
#   ROSBRIDGE_PORT = порт websocket для мобильного приложения.
#       По умолчанию 8090 — тот же номер порта, что у micro-ROS агента
#       (как в стоке Yahboom/PalmSLAM: приложение подключается на :8090).
#       Конфликта нет: rosbridge слушает TCP:8090, агент — UDP:8090.
#   AGENT_PORT     = порт micro-ROS агента (по умолчанию 8090)
#   CFG_LUA        = palmslam_2d_app (по умолчанию) | palmslam_2d_stock | palmslam_2d
set -e
source /opt/ros/humble/setup.bash
source /uros_ws/install/setup.bash 2>/dev/null || true

MODE=${MODE:-udp}
AGENT_PORT=${AGENT_PORT:-8090}
ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-8090}
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

echo "[1/6] micro-ROS agent ($MODE, port $AGENT_PORT)..."
if [ "$MODE" = "serial" ]; then
    DEVICE=${DEVICE:-/dev/ttyACM0}
    micro_ros_agent serial --dev "$DEVICE" -v2 > /tmp/agent.log 2>&1 &
else
    micro_ros_agent udp4 --port "$AGENT_PORT" -v2 > /tmp/agent.log 2>&1 &
fi
PIDS+=($!)
sleep 3

echo "[2/6] Static TFs (base_link->laser roll=pi, base_link->imu_link)..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 3.1416 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser > /tmp/tf.log 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id imu_link > /tmp/tf_imu.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[3/6] Cartographer ($CFG_LUA.lua)..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory /root/palmslam_cartographer \
    -configuration_basename "$CFG_LUA" > /tmp/cartographer.log 2>&1 &
PIDS+=($!)

echo "[4/6] Occupancy grid node..."
ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args -p publish_period_sec:=0.3 > /tmp/occgrid.log 2>&1 &
PIDS+=($!)

echo "[5/6] rosbridge_websocket :$ROSBRIDGE_PORT (точка входа мобильного приложения)..."
ros2 run rosbridge_server rosbridge_websocket --ros-args \
    -p port:="$ROSBRIDGE_PORT" -p addresses:="['0.0.0.0']" \
    > /tmp/rosbridge.log 2>&1 &
PIDS+=($!)

echo "[6/6] Foxglove bridge :8765..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /tmp/foxglove.log 2>&1 &
PIDS+=($!)

echo -e "\n=== APP stack running ==="
echo "Phone app: connect ws://<host-ip>:$ROSBRIDGE_PORT (ROS2)"
echo "Save map:  docker exec <ctr> /root/save_map.sh <name>"
echo "Logs: tail -f /tmp/*.log   | Ctrl+C to stop"
wait
