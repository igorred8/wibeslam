# wibeSlam Docker

Docker-контейнер для запуска micro-ROS агента и Cartographer для проекта wibeSlam.

## Быстрый старт

### 1. Сборка образа

Запустите `build.bat` двойным щелчком. Это соберёт Docker-образ со всеми зависимостями.

Первая сборка займёт 5-10 минут (скачивание ROS 2, установка пакетов, сборка micro-ROS agent).

> **Важно:** все `*.sh` должны сохраняться с окончаниями строк **LF**. В репозитории
> есть `.gitattributes`, но если у вас в git включён `core.autocrlf=true`, выполните
> один раз: `git config --global core.autocrlf input`. Иначе при запуске появится ошибка
> `exec /entrypoint.sh: no such file or directory` (Dockerfile дополнительно сам
> нормализует CRLF при сборке).

### 2. Запуск

Запустите `run_wifeslam.bat` двойным щелчком. Выберите режим:

> **Если при запуске видите `exec /entrypoint.sh: no such file or directory`** —
> локальный образ `wibeslam:latest` устарел/сломан. Если git недоступен в PATH,
> выполните один раз `fix_entrypoint.bat` — он починит образ на месте (docker cp +
> docker commit), без пересборки и без git. Либо добавьте Git в PATH
> (`C:\Program Files\Git\cmd`) и пересоберите: `docker rmi -f wibeslam:latest` → `build.bat`.

**Режим 1: WiFi/UDP**
- ESP32 подключена к WiFi
- Агент слушает UDP порт 8090 (как в стоке Yahboom; переопределяется AGENT_PORT)
- На ESP32 в поле AGENT укажите IP компьютера (узнать: `ipconfig` в cmd)

**Режим 2: USB/Serial**
- ESP32 подключена кабелем к компьютеру
- Требуется WSL2 с пробросом USB (сложнее, рекомендуется WiFi)

**Режим 3: Полный стек**
- Agent + Cartographer + occupancy grid
- Для тестирования без Orange Pi

### 3. Проверка

После запуска в окне должно появиться: