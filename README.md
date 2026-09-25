# wibeSlam — handheld 2D LiDAR SLAM (ESP32-S3 + ROS 2 Humble)

DIY-аналог Yahboom PalmSLAM: handheld-сканер на Waveshare esp32-s3-touch-lcd-2 (ESP32-S3,
IMU QMI8658, дисплей ST7789 320x240 с тачем), LiDAR LDROBOT LD14P + хост-стек в Docker
на Windows 10 (micro-ROS agent + Cartographer + RViz через X11 + rosbridge для мобильного приложения).

## Архитектура

**Прошивка (`head_firmware/src/`, ESP32-S3):**
- LD14P → `/scan` (LaserScan, ~10 Гц); QMI8658 → `/imu` (Imu, 100 Гц, dual-notch фильтр)
- Транспорт: WiFi UDP (micro-ROS) или USB-serial; переключение кнопкой на экране
- Синхронизация времени: подписка на `/host_time` (std_msgs/Int64, мс) от хоста
- Узел: `palmslam_diy` (у Yahboom `YB_PLAMSLAM_Node` — имя узла приложению не нужно,
  совпадают топики `/scan`, `/imu`, `/map`)

**Хост (`wibeslam-docker/`, Windows 10 + WSL2 + Docker Desktop):**
- micro-ROS agent (udp4, порт настраивается `AGENT_PORT`, по умолчанию 8090 — как в стоке Yahboom)
- Cartographer 2D (наш тюнинг `use_imu_data=true` + deskew x2; либо сток-Yahboom конфиги)
- `/map`, `/tf`; RViz2 через VcXsrv (X11) или Foxglove (:8765)
- rosbridge_websocket (:9090) — точка входа для приложения «ROS Robot» (режим APP)

## Структура

- `head_firmware/`, `head/` — скетч Arduino (ESP32-S3, core 2.0.11)
- `wibeslam-docker/` — Docker-образ, bat-скрипты запуска, конфиги Cartographer, RViz-конфиг
  - `host/run_slam.sh` — полный стек (agent + Cartographer + host_time + foxglove)
  - `host/app_launch.sh` — APP-стек как у Yahboom: agent → TF → cartographer →
    occupancy grid → **rosbridge :9090** → foxglove
  - `host/save_map.sh` — аналог `save_map_launch.py`: карта в `/root/maps/<имя>.pgm/.yaml`
  - `host/palmslam_cartographer/*.lua` — три конфига (см. таблицу соответствия)
- `docs/RESEARCH.md` — исследование похожих проектов и граблей

## Соответствие оригинальному Yahboom PalmSLAM

Параметры извлечены из официальной PDF-документации Yahboom («Cartographer»,
«Robot information release», «APP Mapping», «microROS Agent»). Исходники пакетов
`yahboomcar_*` закрыты — сверка возможна только по документации и публичным топикам.

| Компонент | Оригинал PalmSLAM | Наш хост |
|---|---|---|
| micro-ROS agent | `microros/micro-ros-agent:humble udp4 --port 8090 -v4` | ✅ идентичен: тот же агент (собран из micro_ros_setup humble), порт 8090 по умолчанию (`AGENT_PORT`) |
| Топики платы | pub `/scan`, `/imu`; sub `/beep` (UInt16) | pub `/scan`, `/imu` ✅; `/beep` в прошивке нет |
| SLAM | `cartographer_node` + `lds_2d.lua`: tracking/published=`base_footprint`, `use_imu_data=false`, min 0.1/max 30, missing 3, motion filter 0.3°, min_score 0.65 / global 0.7, `optimize_every_n_nodes=0` | `palmslam_2d_stock.lua` — дословный сток (A/B); `palmslam_2d_app.lua` — сток + оптимизация для APP; `palmslam_2d.lua` — наш тюнинг (IMU+deskew) |
| TF | base_link→laser_frame xyz(-0.0046,0,0.094); base_footprint→base_link z=0.05 | base_link→laser roll=π (LD14P физически перевёрнут — параметр нашего корпуса, менять нельзя) |
| Визуализация | `display_launch.py` (rviz2) | `run_rviz.bat` ✅ эквивалент |
| Сохранение карты | `save_map_launch.py` → `yahboom_map.pgm/.yaml` | `save_map.sh` → `/root/maps/<имя>.pgm/.yaml`, тот же формат yaml ✅ |
| Мобильное приложение | rosbridge_websocket :9090 + robot_pose_publisher + laserscan_to_pointcloud | режим APP: rosbridge :9090 (+ foxglove :8765) ✅ |
| ROS_DOMAIN_ID | 15 (согласован плата↔хост) | не задаётся (0 с обеих сторон — согласовано, работает) |

**Неустранимые отличия** (без закрытых пакетов Yahboom / другого железа):
имя узла (cosmetic, приложению не нужно), отсутствие `/beep` (не требуется),
TF laser roll=π (наше железо). Порт агента выровнен со стоком: 8090.

## Прошивка

1. Arduino IDE, плата ESP32-S3 Dev Module, core esp32 2.0.11.
2. Библиотеки: LDS 0.6.3, micro_ros_arduino 2.0.8-humble,
   GFX Library for Arduino 1.4.5, Adafruit GFX 1.12.6.
3. Настройки в `head_firmware/src/config.h` (WiFi, пины, режимы фильтров,
   `#define AGENT_PORT 8090` — совпадает со стоком Yahboom).
4. Первый старт: страница NETWORK → ввести SSID/PASS/AGENT → SAVE (хранится в NVS).
5. Калибровка IMU автоматическая: при первом подключении к ROS2 плата стоит
   неподвижно, снимаются bias gyro/acc; далее автокоррекция нуля в покое.

## Хост (Windows 10)

1. WSL2 + Ubuntu: `wsl --install -d Ubuntu`, пакет обновления ядра WSL2.
2. Docker Desktop (backend WSL2), интеграция с дистрибутивом Ubuntu.
3. VcXsrv (X-сервер): One window, display 0, Disable access control;
   правило файрвола для vcxsrv.exe.
4. Файрвол (для каждого используемого порта):
   `new-netfirewallrule -displayname "wibeslam udp" -direction inbound -protocol udp -localport 8090 -action allow`
   и отдельно `-localport 9090 -protocol tcp` для rosbridge.
5. `wibeslam-docker/build.bat` — сборка образа `wibeslam:latest`.
6. `wibeslam-docker/run_wifeslam.bat` — режимы:
   - **1** = agent UDP; **2** = agent Serial; **3** = полный стек (agent + Cartographer + host_time + foxglove);
   - **4** = APP-стек (agent + Cartographer + rosbridge :9090 + foxglove) — для телефона.
   Порт агента меняется переменной `AGENT_PORT` в начале bat-файла.
7. `wibeslam-docker/run_rviz.bat` — RViz2 через X11 из контейнера (DISPLAY=host.docker.internal:0).

## Подключение мобильного приложения Yahboom («ROS Robot»)

1. Пересобрать образ (`build.bat`), запустить `run_wifeslam.bat` → режим **4**.
2. Узнать IP компьютера в LAN (`ipconfig`, IPv4 Wi-Fi/Ethernet адаптера).
3. В приложении: адрес = IP компьютера, тип = **ROS2**, Connect → `ws://<IP>:9090`.
4. Строить карту, двигая сканер; Save Map в приложении. Либо сохранить вручную:
   `docker exec wibeslam_agent /root/save_map.sh mymap` → `/root/maps/mymap.pgm/.yaml`
   (скопировать: `docker cp wibeslam_agent:/root/maps ./maps`).
5. Для стокового поведения Yahboom (A/B-сравнение карт): добавить
   `-e CFG_LUA=palmslam_2d_stock` в docker run режима 4.

### Требования к сети

| Порт | Протокол | Назначение | Обязательный? |
|------|----------|------------|---------------|
| **9090** | TCP | rosbridge для приложения | ✅ Да |
| **8090** | UDP | micro-ROS агент (связь с платой) | ✅ Да |
| **8765** | TCP | Foxglove (опционально) | ⬜ Нет |

**Откройте порт 9090 в брандмауэре Windows:**
```powershell
New-NetFirewallRule -DisplayName "ROS Robot TCP 9090" -Direction Inbound -Protocol TCP -LocalPort 9090 -Action Allow

## TF и калибровка

| Параметр | Значение |
|---|---|
| base_link → laser | roll 3.1416 (лидар физически перевёрнут), yaw = итоговый подбор |
| base_link → imu_link | identity |
| IMU bias | автосъём при ROS-connect + автокоррекция в покое |
| scan_time / time_increment | честные (1/scanFrequency, /360) — для deskew |

## Известные грабли (см. docs/RESEARCH.md)

- Docker Desktop на Windows: `--net=host` НЕ публикует порты в LAN — только `-p`.
- bat-файлы с кириллицей + `chcp 65001` ломают парсинг cmd — скрипты ASCII-only.
- Таймстампы micro-ROS должны быть монотонными и синхронными с хостом.
- Cartographer 2D без IMU «плывёт» по позе; без одометрии slam_toolbox не умеет.
- LaserScan по WiFi UDP: не поднимать темп выше ~10 Гц (десериализация агента).
- Для полной проверки изменений пересоберите образ локально (`build.bat`) —
  Docker-сборка вне Windows/WSL недоступна.

## Roadmap

- [ ] bag-полигон: запись прохода и оффлайн-тюнинг бэкендов
- [ ] A/B: Cartographer stock vs our tuning vs KISS-ICP на том же bag
- [ ] rmw_uros_sync_session вместо /host_time
- [ ] gimbal / tilt-gate для наклона головы
- [ ] подписка /beep в прошивке (если приложение шлёт команды зуммера)

## Лицензия и статус

Проект находится на стадии разработки и ещё не готов к конечному использованию.
Если вам интересно — не стесняйтесь присоединяйтесь.
P.S. я не являюсь кодером, проект полностью написан с помощью открытых ИИ в режиме чата.

MIT
