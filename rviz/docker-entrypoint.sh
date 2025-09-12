#!/bin/bash
set -e

echo " === Запуск Xvfb (виртуальный X-сервер) ==="
# Увеличиваем разрешение для лучшего отображения
Xvfb :1 -screen 0 1820x980x24 -ac -noreset +extension GLX +render -nolisten tcp &
XVFB_PID=$!
sleep 2

export DISPLAY=:1
export XDG_RUNTIME_DIR=/tmp
export XAUTHORITY=/tmp/.Xauth

echo " === Запуск оконного менеджера fluxbox ==="
fluxbox &
sleep 1

echo " === Активация окружения ROS 2 ==="
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

echo " === Запуск RViz2 ==="
DISPLAY=:1 rviz2 -d /workspace/src/Lslidar/lslidar_driver/rviz/lslidar.rviz &
RVIZ_PID=$!
sleep 2

echo " === Запуск x11vnc ==="
# Обновляем геометрию под новое разрешение
x11vnc -display :1 -nopw -forever -shared -ncache 10 -listen localhost -geometry 1920x1080 &
VNC_PID=$!
sleep 2

echo " === Запуск noVNC (websockify на порту 6080) ==="
websockify --web /usr/share/novnc/ 6080 localhost:5900 &
NOVNC_PID=$!
sleep 2

echo " === Все процессы запущены ==="
wait $XVFB_PID $RVIZ_PID $VNC_PID $NOVNC_PID
