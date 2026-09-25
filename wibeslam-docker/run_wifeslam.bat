@echo off
echo === wibeSlam launcher ===

docker --version >nul 2>&1
if errorlevel 1 goto nodocker

docker image inspect wibeslam:latest >nul 2>&1
if errorlevel 1 goto noimage

rem Проверка, что образ не устаревший: /entrypoint.sh должен быть внутри.
docker run --rm --entrypoint ls wibeslam:latest -l /entrypoint.sh >nul 2>&1
if errorlevel 1 goto staleimage


echo Select mode:
echo   1. WiFi/UDP agent
echo   2. USB/Serial agent
echo   3. Full stack (agent + Cartographer)
echo   4. APP stack (Cartographer + rosbridge :9090 for phone app)
set AGENT_PORT=8090
set /p choice=Enter 1, 2, 3 or 4: 

rem убить старый контейнер с тем же именем, если висит
docker rm -f wibeslam_agent >nul 2>&1

if "%choice%"=="1" goto mode1
if "%choice%"=="2" goto mode2
if "%choice%"=="3" goto mode3
if "%choice%"=="4" goto mode4
echo Bad choice.
goto end

:mode1
docker run -it --rm --name wibeslam_agent -p %AGENT_PORT%:%AGENT_PORT%/udp wibeslam:latest
goto end

:mode2
docker run -it --rm --name wibeslam_agent --privileged --device=/dev/ttyACM0 -e MODE=serial wibeslam:latest
goto end

:mode3
docker run -it --rm --name wibeslam_agent -p %AGENT_PORT%:%AGENT_PORT%/udp -p 8765:8765 -e MODE=full wibeslam:latest
goto end

:mode4
docker run -it --rm --name wibeslam_agent -p %AGENT_PORT%:%AGENT_PORT%/udp -p 9090:9090 -p 8765:8765 -e MODE=app wibeslam:latest
goto end

:nodocker
echo ERROR: Docker not found or not running.
pause
exit /b 1

:noimage
echo ERROR: image wibeslam:latest not found. Run build.bat first.
pause
exit /b 1

:staleimage
rem Entrypoint отсутствует внутри образа: образ собран из устаревшего кода
rem (например, entrypoint.sh был в CRLF/не скопирован). Нужна пересборка.
echo.
echo ============================================================
echo ERROR: /entrypoint.sh not found inside the image.
echo The local image "wibeslam:latest" is STALE - it was built
echo before entrypoint.sh was added, or from a broken checkout.
echo.
echo Fix (choose one):
echo   A) No git needed - from this folder run:
echo         powershell -ExecutionPolicy Bypass -File .\fix_entrypoint.ps1
echo      (or fix_entrypoint.bat if you have it), then run this script again.
echo   B) Rebuild from current code (needs Dockerfile + entrypoint.sh):
echo         docker rmi -f wibeslam:latest
echo         build.bat
echo ============================================================
set /p ok=Try quick repair now? (y/n):
if /i "%ok%"=="y" powershell -ExecutionPolicy Bypass -File "%~dp0fix_entrypoint.ps1"
pause
exit /b 1

:end
pause