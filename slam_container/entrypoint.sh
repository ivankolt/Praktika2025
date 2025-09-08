#!/bin/bash

# Логирование
echo "Starting SLAM container..."

# Запуск SLAM
source /workspace/install/setup.bash
exec ros2 launch slam_package slam_launch.py
