#!/bin/bash

# Источник окружения ROS 2
source /workspace/install/setup.bash

# Проверяем доступность UART портов
echo "Проверка доступных UART портов..."
ls -la /dev/ttyUSB* /dev/serial* 2>/dev/null || echo "UART порты не найдены"

# Запуск ноды hoverboard с автоматическим определением порта
if [ -e "/dev/serial0" ]; then
    echo "Используем /dev/serial0 (Raspberry Pi UART)"
    SERIAL_PORT="/dev/serial0"
elif [ -e "/dev/ttyUSB0" ]; then
    echo "Используем /dev/ttyUSB0 (USB-UART адаптер)"
    SERIAL_PORT="/dev/ttyUSB0"
elif [ -e "/dev/ttyUSB1" ]; then
    echo "Используем /dev/ttyUSB1 (USB-UART адаптер)"
    SERIAL_PORT="/dev/ttyUSB1"
else
    echo "ОШИБКА: UART порт не найден!"
    exit 1
fi

echo "Запуск motor_controller_node с портом: $SERIAL_PORT"
ros2 run motor_controller_package motor_controller_node --ros-args \
  -p serial_port:=$SERIAL_PORT \
  -p baud_rate:=115200 \
  -p max_linear_speed:=1.0 \
  -p max_angular_speed:=1.0 \
  -p wheel_separation:=0.5
