#!/bin/bash

# Скрипт для настройки UART на Raspberry Pi

echo "=== Настройка UART для STM32 ==="
echo ""

# Проверяем доступные UART порты
echo "Доступные UART порты:"
ls -la /dev/tty* | grep -E "(USB|S0|ACM)" | head -10
echo ""

# Проверяем права доступа
echo "Права доступа к портам:"
ls -la /dev/ttyUSB* /dev/ttyS0 /dev/ttyACM* 2>/dev/null
echo ""

# Проверяем группы пользователя
echo "Группы пользователя:"
groups
echo ""

# Проверяем, есть ли stm32flash
echo "Проверка stm32flash:"
if command -v stm32flash >/dev/null 2>&1; then
    echo "✓ stm32flash установлен: $(which stm32flash)"
    stm32flash -h | head -5
else
    echo "✗ stm32flash не найден"
    echo "Установите: sudo apt install stm32flash"
fi
echo ""

# Проверяем подключенные USB устройства
echo "Подключенные USB устройства:"
lsusb | grep -i "serial\|uart\|ftdi\|ch340\|cp210"
echo ""

# Тестируем связь с загрузчиком
echo "Тестирование связи с загрузчиком..."
PORTS=("/dev/ttyUSB0" "/dev/ttyS0" "/dev/ttyACM0")

for PORT in "${PORTS[@]}"; do
    if [ -e "$PORT" ]; then
        echo "Тестируем $PORT..."
        timeout 3 sudo stm32flash "$PORT" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "✓ Связь установлена с $PORT"
        else
            echo "✗ Нет связи с $PORT"
        fi
    fi
done

echo ""
echo "=== Инструкции ==="
echo "1. Подключите USB-TTL адаптер к STM32:"
echo "   - TX адаптера → RX STM32 (PA10)"
echo "   - RX адаптера → TX STM32 (PA9)"
echo "   - GND → GND"
echo "   - 3.3V → 3.3V (или 5V → 5V, если плата поддерживает)"
echo ""
echo "2. Для входа в режим загрузчика:"
echo "   - Быстро нажмите RESET дважды (в течение 0.5 сек)"
echo "   - Или перезагрузите плату и сразу запустите: ./flash.sh firmware.bin"
echo ""
echo "3. Прошивка:"
echo "   ./flash.sh your_firmware.bin"
