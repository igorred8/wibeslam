# wibeSlam — handheld 2D LiDAR SLAM (ESP32-S3 + ROS 2 Humble)

DIY-аналог Yahboom PalmSLAM: handheld-сканер на Waveshare esp32-s3-touch-lcd-2 (ESP32-S3
IMU QMI8658, дисплей ST7789 320x240 с тачем), LiDAR LDROBOT LD14P + хост-стек в Docker на Windows 10
(micro-ROS agent + Cartographer + RViz через X11).

## Архитектура

ESP32-S3 (proшивка):
  LD14P -> /scan (LaserScan, ~10 Гц)
  QMI8658 -> imuTask 500 Гц: bias-калибровка по ROS-connect, dual-notch (f и 2f
  оборота), EMA accel, deadzone gyro -> /imu (Imu, 100 Гц)
  Транспорт: WiFi UDP (micro-ROS, порт 8888) или USB-serial; переключение кнопкой на экране.
  Синхронизация времени: подписка на /host_time от хоста.

Хост (Windows 10 + WSL2 + Docker Desktop):
  micro-ROS agent (udp4:8888) -> Cartographer 2D (use_imu_data=true,
  sub-scans x2 = deskew) -> /map, /tf; RViz2 через VcXsrv (X11) или Foxglove.

## Структура

- `firmware/` — скетч Arduino (ESP32-S3, core 2.0.11)
- `host/`     — Docker-образ, bat-скрипты запуска, конфиг Cartographer, RViz-конфиг
- `docs/`     — исследование похожих проектов и граблей (RESEARCH.md)

## Прошивка

1. Arduino IDE, плата ESP32-S3 Dev Module, core esp32 2.0.11.
2. Библиотеки: LDS 0.6.3, micro_ros_arduino 2.0.8-humble,
   GFX Library for Arduino 1.4.5, Adafruit GFX 1.12.6.
3. Настройки в `firmware/config.h` (WiFi, пины, режимы фильтров).
4. Первый старт: страница NETWORK -> ввести SSID/PASS/AGENT -> SAVE (хранится в NVS).
5. Калибровка IMU автоматическая: при первом подключении к ROS2 плата  с стоит
   неподвижно, снимаются bias gyro/acc; далее автокоррекция нуля в покое.

## Хост (Windows 10)

1. WSL2 + Ubuntu: `wsl --install -d Ubuntu`, пакет обновления ядра WSL2.
2. Docker Desktop (backend WSL2), интеграция с дистром Ubuntu.
3. VcXsrv (X-сервер): One window, display 0, Disable access control;
   правило файрвола для vcxsrv.exe.
4. Файрвол: `new-netfirewallrule -displayname "wibeslam udp 8888" -direction inbound -protocol udp -localport 8888 -action allow`
5. `host/build.bat` — сборка образа wibeslam:latest.
6. `host/run_wibeslam.bat` — режимы: 1 = agent UDP, 3 = полный стек (agent +
   Cartographer + occupancy grid + host_time).
7. `host/run_rviz.bat` — RViz2 через X11 из контейнера (DISPLAY=host.docker.internal:0).

## TF и калибровка

| Параметр | Значение |
|---|---|
| base_link -> laser | roll 3.1416 (лидар физически перевёрнут), yaw = итоговый подбор |
| base_link -> imu_link | identity |
| IMU bias | автосъём при ROS-connect + автокоррекция в покое |
| scan_time / time_increment | честные (1/scanFrequency, /360) — для deskew |

## Известные грабли (см. docs/RESEARCH.md)

- Docker Desktop на Windows: `--net=host` НЕ публикует порты в LAN — только `-p`.
- bat-файлы с кириллицей + `chcp 65001` ломают парсинг cmd — скрипты ASCII-only.
- Таймстампы micro-ROS должны быть монотонными и синхронными с хостом.
- Cartographer 2D без IMU «плывёт» по позе; без одометрии slam_toolbox не умеет.
- LaserScan по WiFi UDP: не поднимать темп выше ~10 Гц (десериализация агента).

## Roadmap

- [ ] bag-полигон: запись прохода и оффлайн-тюнинг бэкендов
- [ ] A/B: Cartographer vs KISS-ICP/KM-SAM на том же bag
- [ ] rmw_uros_sync_session вместо /host_time
- [ ] gimbal / tilt-gate для наклона головы

## Проект находится на стадии разработки и еще не готов к конечному использованию. Если вам интересно не стесняйтесь присоединяйтесь.
P.S. я не являюсь кодером, проект полностью написан с помощью открытых ИИ в режиме чата


## License

MIT