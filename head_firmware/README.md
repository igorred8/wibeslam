# head_firmware — сборка через PlatformIO (ESP32-S3)

Готовый к сборке проект прошивки для платы `head` (скрипты скопированы из `../head/`).

## Что уже настроено
- `platformio.ini` — окружение `esp32s3` (board `esp32-s3-devkitc-1`, USB-CDC, 115200 monitor).
- `lib/micro_ros_arduino/` — micro-ROS v2.0.8-humble (precompiled, Xtensa ESP32; бинарно совместим с ESP32-S3).
- `add_microros.py` — подключает предсобранную `libmicroros.a` при линковке.
- Зависимости: GFX Library for Arduino 1.4.6 (пинкушена — новее требует `esp32-hal-periman.h`, которого нет в framework 2.0.17), Adafruit GFX, kaiaai/LDS.

## Сборка (успешно проверена)
```bash
cd head_firmware
pio run
# RAM: 24.2% (79 КБ / 320 КБ), Flash: 26.4% (882 КБ / 3.3 МБ)
# артефакт: .pio/build/esp32s3/firmware.bin
```

## Прошивка на COM3 (выполнять на машине с подключённой платой)
Из этого контейнера порт COM3 недоступен (нет проброса USB). На вашем ПК:
```bash
cd head_firmware
pio run -t upload --upload-port COM3
```
Если порт «занят» — закройте монитор Serial. Если устройство не входит в boot-mode:
зажмите BOOT, короткнно нажмите RST/EN, отпустите BOOT и повторите команду.

Альтернатива — прошить готовый образ напрямую:
```bash
esptool.py --chip esp32s3 --port COM3 --baud 921600 write_flash 0x0 .pio/build/esp32s3/firmware.bin
```

## Мониторинг
```bash
pio device monitor --port COM3 --baud 115200
```
