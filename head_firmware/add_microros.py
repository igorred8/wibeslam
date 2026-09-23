Import("env")
# Подключаем предсобранную библиотеку micro-ROS (Xtensa ESP32-S3 совместима)
env.Append(LIBPATH=["$PROJECT_DIR/lib/micro_ros_arduino/src/esp32"])
env.Append(LIBS=["microros"])
