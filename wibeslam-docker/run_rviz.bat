@echo off
echo === RViz2 via X11 ===

tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if "%ERRORLEVEL%"=="1" goto nox

docker exec -it wibeslam_agent env DISPLAY=host.docker.internal:0 LIBGL_ALWAYS_SOFTWARE=1 bash -c "source /opt/ros/humble/setup.bash && rviz2 -d /root/palmslam_viz.rviz"
goto end

:nox
echo X-server NOT running! Start xlaunch.xlaunch first.
pause

:end