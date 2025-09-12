#!/bin/bash

# Скрипт для прошивки STM32 через UART
# Использование: ./flash.sh firmware.bin

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

echo "=== Прошивка STM32 ==="
echo "Файл прошивки: $FIRMWARE_FILE"
echo ""

# Попробуем разные порты
PORTS=("/dev/ttyUSB0" "/dev/ttyS0" "/dev/ttyACM0")

for PORT in "${PORTS[@]}"; do
    if [ -e "$PORT" ]; then
        echo "Пробуем порт: $PORT"
        
        # Проверяем связь с загрузчиком
        echo "Проверка связи с загрузчиком..."
        if sudo stm32flash "$PORT" > /dev/null 2>&1; then
            echo "✓ Связь установлена с $PORT"
            echo ""
            echo "Прошивка..."
            sudo stm32flash -b 230400 -w "$FIRMWARE_FILE" -v -g 0x08002000 "$PORT"
            
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
echo "Проверьте:"
echo "1. Подключен ли USB-TTL адаптер"
echo "2. Правильно ли подключены TX/RX (перекрестно)"
echo "3. Подано ли питание на плату STM32"
echo "4. Находится ли плата в режиме загрузчика"
echo ""
echo "Для входа в режим загрузчика:"
echo "- Быстро нажмите RESET дважды (в течение 0.5 сек)"
echo "- Или перезагрузите плату и сразу запустите команду"