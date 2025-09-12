#!/bin/bash

# Скрипт для проверки связи с загрузчиком STM32

echo "=== Проверка связи с загрузчиком STM32 ==="
echo ""

# Остановка Docker контейнера, если он запущен
echo "Остановка Docker контейнера hoverboard..."
sudo docker stop hoverboard 2>/dev/null || true

# Ожидание освобождения порта
sleep 2

echo "Проверка доступных портов..."
echo "USB-TTL адаптер:"
ls -l /dev/ttyUSB* 2>/dev/null || echo "  Не найден"
echo "Встроенный UART:"
ls -l /dev/ttyS* 2>/dev/null || echo "  Не найден"
echo ""

echo "Попытка подключения к загрузчику через USB-TTL (/dev/ttyUSB0)..."
if stm32flash -b 115200 -m 8n1 /dev/ttyUSB0 2>/dev/null; then
    echo "✓ Загрузчик найден на /dev/ttyUSB0!"
    echo "Информация о чипе:"
    stm32flash -b 115200 -m 8n1 /dev/ttyUSB0
else
    echo "✗ Загрузчик не найден на /dev/ttyUSB0"
fi

echo ""
echo "Попытка подключения к загрузчику через встроенный UART (/dev/ttyS0)..."
if stm32flash -b 115200 -m 8n1 /dev/ttyS0 2>/dev/null; then
    echo "✓ Загрузчик найден на /dev/ttyS0!"
    echo "Информация о чипе:"
    stm32flash -b 115200 -m 8n1 /dev/ttyS0
else
    echo "✗ Загрузчик не найден на /dev/ttyS0"
fi

echo ""
echo "Если загрузчик не найден:"
echo "1. Убедитесь, что плата STM32 подключена и включена"
echo "2. Нажмите кнопку RESET на плате дважды быстро (в течение 0.5 сек)"
echo "3. Светодиод на PC13 должен начать часто мигать"
echo "4. Запустите скрипт снова"
echo ""
echo "Запуск Docker контейнера hoverboard..."
sudo docker start hoverboard
