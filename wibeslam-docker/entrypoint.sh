#!/bin/bash
set -e

# Инициализация ROS 2
source /opt/ros/humble/setup.bash

# Если собран micro-ROS agent
if [ -f /uros_ws/install/setup.bash ]; then
    source /uros_ws/install/setup.bash
fi

# Режим запуска: udp (WiFi) или serial (USB)
MODE=${MODE:-udp}

if [ "$MODE" = "udp" ]; then
    echo "=== Запуск micro-ROS agent (UDP/WiFi) на порту 8888 ==="
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
elif [ "$MODE" = "serial" ]; then
    DEVICE=${DEVICE:-/dev/ttyACM0}
    echo "=== Запуск micro-ROS agent (Serial/USB) на $DEVICE ==="
    exec ros2 run micro_ros_agent micro_ros_agent serial --dev $DEVICE -v6
elif [ "$MODE" = "full" ]; then
    echo "=== Запуск полного стека (agent + Cartographer) ==="
    exec /root/run_slam.sh
else
    echo "Неизвестный режим: $MODE"
    echo "Доступные: udp, serial, full"
    exit 1
fi