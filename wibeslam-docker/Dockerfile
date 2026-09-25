FROM ros:humble

# Обновление и установка зависимостей
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    ros-humble-cartographer-ros \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-rosbridge-server \
    ros-humble-foxglove-bridge \
    python3-yaml \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Установка micro-ROS agent
RUN mkdir -p /uros_ws/src && \
    cd /uros_ws/src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git && \
    cd /uros_ws && \
    . /opt/ros/humble/setup.sh && \
    apt-get update && apt-get install -y python3-rosdep && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --packages-select micro_ros_setup && \
    . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/*

# Копирование конфигов Cartographer
COPY host/palmslam_cartographer /root/palmslam_cartographer
COPY host/run_slam.sh /root/run_slam.sh
COPY host/app_launch.sh /root/app_launch.sh
COPY host/save_map.sh /root/save_map.sh
COPY host/host_time_publisher.py /root/host_time_publisher.py

RUN chmod +x /root/run_slam.sh /root/app_launch.sh /root/save_map.sh

WORKDIR /root

# Скрипт запуска
COPY entrypoint.sh /entrypoint.sh
# Нормализация окончаний строк (защита от CRLF при clone/редактировании на Windows:
# shebang вида "#!/bin/bash\r" ломает exec и даёт "no such file or directory")
RUN sed -i 's/\r$//g' /entrypoint.sh \
    && bash -n /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
