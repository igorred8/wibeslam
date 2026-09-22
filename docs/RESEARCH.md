# Исследование похожих проектов и граблей

## Yahboom PalmSLAM (прямой аналог)
- Архитектура: плата-расширение ESP32 + 6-осевой IMU на устройстве, лидар и IMU
  уходят на хост, SLAM считается в Docker (курс: Gmapping + Cartographer).
- IMU используется как «сторож горизонта» (контроль позы удержания), основную
  позу даёт scan matching. Наш выбор тот же.

## Грабли сообществ (issue-трекеры)
| Проблема | Источник | Наше решение |
|---|---|---|
| 2D SLAM без IMU: tf map->base «плавает» | cartographer_ros #472, #623 | use_imu_data=true |
| 2D без одометрии работает (use_odometry=false) | cartographer_ros #1056 | используем |
| frame_id скана != tracking_frame -> карта молча не строится | cartographer_ros #623 | TF laser/imu_link фиксированы |
| slam_toolbox без одометрии не умеет (нужен внешний scan matcher) | slam_toolbox #221 | не используем |
| Связка madgwick+EKF+slam_toolbox на ESP32: пустой /map | stackoverflow 79614155 | не повторяем |
| micro-ROS: рассинхрон времени клиент/агент | micro_ros_stm32 #24, #131 | /host_time; в планах rmw_uros_sync_session |
| LaserScan по WiFi UDP: deserialization error на темпе | micro_ros_arduino #1427 | темп <=10 Гц, фикс. размер |
| UDP режется файрволом Windows | stackexchange 100142 | правило файрвола |
| CSM: CPU vs качество локализации | cartographer_ros #1073 | CSM включён, хост с запасом |

## Unitree 4D LiDAR (L1/L2)
- Встроенный IMU + per-ray deskewing: каждый луч получает свою позу по времени.
- Механика: противовес 5-6 г против горизонтальных вибраций (нам недоступно).
- Известная болезнь: duplicated walls / Z-oscillation при плохом deskew/калибровке.
- Наш эквивалент: честные scan_time/time_increment + sub-scans x2 в Cartographer
  + чистый IMU (bias + dual-notch + deadzone).

## Альтернативные бэкенды (резерв)
- KISS-ICP / KM-SAM (ROS2): лидарная одометрия «без тюнинга», loop closure.
  Вход: PointCloud2; наш /scan конвертируется тривиально (z=0).
- 2DLIW-SLAM: tight-coupling 2D LiDAR+IMU+колёса — не наш случай (колёс нет).

## Выводы
1. Cartographer остаётся основным бэкендом (как у PalmSLAM).
2. IMU обязан быть откалиброван и быстр (нулевая задержка по gyro).
3. Дескью и саб-сканы важнее сглаживания.
4. Оффлайн-полигон на rosbag — главный инструмент тюнинга без ходьбы по комнате.