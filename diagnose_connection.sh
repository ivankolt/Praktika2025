#!/bin/bash

# Скрипт для диагностики подключения к плате hoverboard

UART_DEVICE="/dev/ttyUSB0"

echo "=== Диагностика подключения к плате hoverboard ==="
echo ""

# Проверяем существование UART устройства
echo "1. Проверка UART устройства..."
if [ -e "$UART_DEVICE" ]; then
    echo "✓ Устройство $UART_DEVICE найдено"
    ls -la "$UART_DEVICE"
else
    echo "✗ Устройство $UART_DEVICE не найдено"
    echo "Проверьте подключение USB-UART адаптера"
    exit 1
fi

echo ""

# Проверяем права доступа
echo "2. Проверка прав доступа..."
if [ -r "$UART_DEVICE" ] && [ -w "$UART_DEVICE" ]; then
    echo "✓ Права на чтение и запись есть"
else
    echo "✗ Нет прав на чтение/запись"
    echo "Пользователь должен быть в группе dialout"
    groups | grep dialout || echo "Пользователь НЕ в группе dialout"
fi

echo ""

# Проверяем настройки порта
echo "3. Проверка настроек порта..."
stty -F "$UART_DEVICE" 2>/dev/null && echo "✓ Порт доступен для настройки" || echo "✗ Порт недоступен"

echo ""

# Проверяем, есть ли активность на порту
echo "4. Проверка активности на порту..."
echo "Отправляем тестовые данные на порт..."
echo -e "\x7F" > "$UART_DEVICE" 2>/dev/null && echo "✓ Данные отправлены" || echo "✗ Не удалось отправить данные"

echo ""

# Пробуем разные команды stm32flash
echo "5. Тестирование stm32flash..."

echo "Попытка 1: Базовая команда"
timeout 3 stm32flash "$UART_DEVICE" 2>&1 | head -5

echo ""
echo "Попытка 2: С параметром -c (resume connection)"
timeout 3 stm32flash -c "$UART_DEVICE" 2>&1 | head -5

echo ""
echo "Попытка 3: С низкой скоростью"
timeout 3 stm32flash -b 9600 "$UART_DEVICE" 2>&1 | head -5

echo ""
echo "Попытка 4: С высокой скоростью"
timeout 3 stm32flash -b 230400 "$UART_DEVICE" 2>&1 | head -5

echo ""
echo "=== Рекомендации ==="
echo "1. Убедитесь, что плата hoverboard подключена к USB-UART адаптеру"
echo "2. Проверьте подключение проводов:"
echo "   - TX платы -> RX адаптера"
echo "   - RX платы -> TX адаптера"
echo "   - GND платы -> GND адаптера"
echo "   - 3.3V или 5V платы -> VCC адаптера (если нужно)"
echo "3. Убедитесь, что плата получает питание"
echo "4. Для активации загрузчика:"
echo "   - Включите плату и сразу запустите прошивку"
echo "   - Или быстро нажмите RESET 2 раза"
echo "5. Светодиод на PC13 должен мигать в режиме загрузчика"
