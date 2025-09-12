#!/bin/bash

# Демонстрационный скрипт для работы с STM32 через UART

echo "=========================================="
echo "    Демонстрация работы с STM32 UART     "
echo "=========================================="
echo ""

echo "1. Проверка системы..."
./setup_uart.sh
echo ""

echo "2. Доступные команды:"
echo "   ./flash.sh firmware.bin              - прошивка с ручным сбросом"
echo "   ./flash_with_reset.sh firmware.bin   - прошивка с автосбросом"
echo "   ./setup_uart.sh                      - диагностика системы"
echo ""

echo "3. Примеры команд stm32flash:"
echo "   # Проверка связи:"
echo "   sudo stm32flash /dev/ttyUSB0"
echo ""
echo "   # Прошивка с проверкой:"
echo "   sudo stm32flash -w firmware.bin -v -g 0x08002000 /dev/ttyUSB0"
echo ""
echo "   # Прошивка с автосбросом:"
echo "   sudo stm32flash -R -b 230400 -w firmware.bin -v -g 0x08002000 /dev/ttyUSB0"
echo ""

echo "4. Схема подключения:"
echo "   USB-TTL адаптер    STM32"
echo "   ├─ TX (зеленый)  → PA10 (RX)"
echo "   ├─ RX (белый)    → PA9  (TX)"  
echo "   ├─ GND (черный)  → GND"
echo "   └─ 3.3V (красный)→ 3.3V"
echo ""

echo "5. Режимы загрузчика:"
echo "   - Автосброс: ./flash_with_reset.sh firmware.bin"
echo "   - Двойной сброс: быстро нажмите RESET дважды"
echo "   - Ручной сброс: нажмите RESET и сразу запустите команду"
echo ""

echo "=========================================="
echo "Система готова к работе!"
echo "=========================================="
