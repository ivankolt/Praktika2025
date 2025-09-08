#!/bin/bash

# Источник окружения ROS 2
source /workspace/install/setup.bash

# Запуск ноды hoverboard
ros2 run motor_controller_package motor_controller_node --ros-args \
  -p serial_port:=/dev/serial0 \
  -p baud_rate:=115200
