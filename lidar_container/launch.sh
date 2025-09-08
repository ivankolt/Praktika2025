#!/bin/bash

# Источник окружения ROS 2
source /workspace/install/setup.bash

# Запуск драйвера лидара
ros2 launch lslidar_driver lslidar_launch.py &

# Ожидание завершения процессов
wait
