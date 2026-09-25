#!/bin/bash
set -e
echo "=== wibeSlam Cartographer stack startup ==="
source /opt/ros/humble/setup.bash
source /uros_ws/install/setup.bash 2>/dev/null || true
PIDS=()

cleanup() {
    echo -e "\n=== Stopping all processes ==="
    for pid in "${PIDS[@]}"; do
        [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && kill "$pid" 2>/dev/null || true
    done
    echo "=== All stopped ==="
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[1/7] Starting micro-ROS agent (WiFi UDP)..."
micro_ros_agent udp4 --port ${AGENT_PORT:-8090} -v2 > /tmp/agent.log 2>&1 &
PIDS+=($!)
sleep 3

echo "[2/7] Starting host time publisher..."
python3 /root/host_time_publisher.py > /tmp/host_time.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[3/7] Starting static TF base_link -> laser..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 3.1416 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser > /tmp/tf.log 2>&1 &
PIDS+=($!)

echo "[4/7] Starting static TF base_link -> imu_link..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id imu_link > /tmp/tf_imu.log 2>&1 &
PIDS+=($!)
sleep 1

echo "[5/7] Starting Cartographer..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory /root/palmslam_cartographer \
    -configuration_basename palmslam_2d.lua > /tmp/cartographer.log 2>&1 &
PIDS+=($!)

echo "[6/7] Starting occupancy grid..."
ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args -p publish_period_sec:=0.3 > /tmp/occgrid.log 2>&1 &
PIDS+=($!)

echo "[7/7] Starting foxglove bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /tmp/foxglove.log 2>&1 &
PIDS+=($!)

echo -e "\n=== All components running ==="
echo "Logs: tail -f /tmp/*.log"
echo "Press Ctrl+C to stop everything\n"
wait
