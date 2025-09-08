#!/bin/bash

# Источник окружения ROS 2
source /workspace/install/setup.bash

# Запуск ноды hoverboard
ros2 run odometry_package odometry_node
