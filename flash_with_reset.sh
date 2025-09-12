#!/bin/bash

# Скрипт для прошивки STM32 с автоматическим сбросом через RTS
# Использование: ./flash_with_reset.sh firmware.bin

if [ $# -eq 0 ]; then
    echo "Использование: $0 <файл_прошивки.bin>"
    echo "Пример: $0 firmware.bin"
    exit 1
fi

FIRMWARE_FILE="$1"

if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "Ошибка: Файл $FIRMWARE_FILE не найден!"
    exit 1
fi

echo "=== Прошивка STM32 с автоматическим сбросом ==="
echo "Файл прошивки: $FIRMWARE_FILE"
echo ""

# Попробуем разные порты с автоматическим сбросом
PORTS=("/dev/ttyUSB0" "/dev/ttyS0" "/dev/ttyACM0")

for PORT in "${PORTS[@]}"; do
    if [ -e "$PORT" ]; then
        echo "Пробуем порт: $PORT"
        
        # Проверяем связь с загрузчиком (с автоматическим сбросом)
        echo "Проверка связи с загрузчиком (с автосбросом)..."
        if sudo stm32flash -R "$PORT" > /dev/null 2>&1; then
            echo "✓ Связь установлена с $PORT"
            echo ""
            echo "Прошивка с автоматическим сбросом..."
            sudo stm32flash -R -b 230400 -w "$FIRMWARE_FILE" -v -g 0x08002000 "$PORT"
            
            if [ $? -eq 0 ]; then
                echo ""
                echo "✓ Прошивка завершена успешно!"
                echo "Прошивка запущена по адресу 0x08002000"
                exit 0
            else
                echo "✗ Ошибка при прошивке через $PORT"
            fi
        else
            echo "✗ Нет связи с загрузчиком через $PORT"
        fi
        echo ""
    fi
done

echo "✗ Не удалось установить связь ни с одним портом"
echo ""
echo "Попробуйте ручной режим:"
echo "1. Быстро нажмите RESET дважды (в течение 0.5 сек)"
echo "2. Сразу запустите: ./flash.sh $FIRMWARE_FILE"
