@echo off
REM ============================================================
REM  Экстренный ремонт образа wibeslam:latest БЕЗ git и без
REM  пересборки. Работает так:
REM    1) берет контейнер из вашего старого образа;
REM    2) записывает в него правильный /entrypoint.sh (LF);
REM    3) "запекает" результат обратно в wibeslam:latest.
REM  После этого run_wifeslam.bat заработает.
REM  Если есть Dockerfile/entrypoint.sh от актуального кода -
REM  лучше просто пересобрать: build.bat
REM ============================================================

docker --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Docker not found or not running. Start Docker Desktop.
    pause
    exit /b 1
)

docker image inspect wibeslam:latest >nul 2>&1
if errorlevel 1 (
    echo ERROR: image wibeslam:latest not found. Run build.bat first.
    pause
    exit /b 1
)

echo Creating temp container from current image...
docker rm -f wibeslam_fix >nul 2>&1
docker create --name wibeslam_fix --entrypoint sh wibeslam:latest >nul
if errorlevel 1 (
    echo ERROR: docker create failed.
    pause
    exit /b 1
)

echo Writing correct /entrypoint.sh into the container...
docker cp entrypoint_fallback.sh wibeslam_fix:/entrypoint.sh
if errorlevel 1 (
    echo ERROR: docker cp failed. Is entrypoint_fallback.sh next to this script?
    docker rm -f wibeslam_fix >nul 2>&1
    pause
    exit /b 1
)

docker exec wibeslam_fix chmod +x /entrypoint.sh
if errorlevel 1 (
    echo ERROR: chmod failed.
    docker rm -f wibeslam_fix >nul 2>&1
    pause
    exit /b 1
)

echo Committing fixed image back to wibeslam:latest...
docker commit --change=ENTRYPOINT[/entrypoint.sh] wibeslam_fix wibeslam:latest
if errorlevel 1 (
    echo ERROR: docker commit failed.
    docker rm -f wibeslam_fix >nul 2>&1
    pause
    exit /b 1
)

docker rm -f wibeslam_fix >nul 2>&1

echo.
echo === DONE: image repaired. Verify: ===
docker run --rm --entrypoint ls wibeslam:latest -l /entrypoint.sh
if errorlevel 1 (
    echo Something is still wrong - rebuild with build.bat instead.
) else (
    echo OK - now run run_wifeslam.bat and choose mode 4.
)
pause
