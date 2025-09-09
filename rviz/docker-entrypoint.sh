#!/bin/bash
set -e

# Запускаем виртуальный X-сервер на дисплее :1
Xvfb :1 -screen 0 1024x768x24 &

# Запускаем простой оконный менеджер
fluxbox &

# Загружаем окружение ROS 2 и собранный workspace
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Запускаем RViz на дисплее :1 с преднастройкой
DISPLAY=:1 rviz2 -d /workspace/src/Lslidar/lslidar_driver/rviz/lslidar.rviz &

# Запускаем x11vnc без пароля, пробрасывая виртуальный Xvfb
x11vnc -display :1 -nopw -forever -shared &

# Запускаем noVNC на порту 6080, перенаправляя на VNC-сервер
websockify --web /usr/share/novnc/ 6080 localhost:5900
