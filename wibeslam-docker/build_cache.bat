@echo off
echo === Сборка Docker-образа wibeslam ===
echo.

REM Проверка Docker
docker --version >nul 2>&1
if errorlevel 1 (
    echo ОШИБКА: Docker не установлен или не запущен
    echo Установите Docker Desktop: https://www.docker.com/products/docker-desktop
    pause
    exit /b 1
)

REM Сборка образа
echo Сборка образа wibeslam:latest...
docker build  -t wibeslam:latest .

if errorlevel 1 (
    echo ОШИБКА: Сборка не удалась
    pause
    exit /b 1
)

echo.
echo === Образ успешно собран ===
echo Теперь можете запустить run_wifeslam.bat
pause