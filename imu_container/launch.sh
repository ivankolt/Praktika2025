#!/bin/bash

# Источник окружения ROS 2
source /workspace/install/setup.bash

# Запуск ноды IMU
ros2 run sensor_package mpu_node
