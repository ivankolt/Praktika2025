#!/bin/bash

# Скрипт для компиляции и прошивки STM32 Motor Controller

echo "=== Компиляция и прошивка STM32 Motor Controller ==="
echo ""

# Проверяем наличие arm-none-eabi-gcc
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "❌ arm-none-eabi-gcc не найден!"
    echo "Установите ARM GCC toolchain:"
    echo "sudo apt install gcc-arm-none-eabi"
    exit 1
fi

# Создаем директорию сборки
mkdir -p build

echo "🔨 Компиляция прошивки..."

# Компилируем прошивку
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Wall -O2 -g \
    -DSTM32F103xB \
    -T STM32F103C8Tx_FLASH.ld \
    -Wl,--gc-sections \
    -specs=nano.specs \
    simple_motor_controller.c \
    -o build/motor_controller.elf

if [ $? -ne 0 ]; then
    echo "❌ Ошибка компиляции!"
    exit 1
fi

echo "✅ Компиляция завершена"

# Создаем BIN файл
arm-none-eabi-objcopy -O binary build/motor_controller.elf build/motor_controller.bin

echo "📦 Создан файл: build/motor_controller.bin"

# Показываем размер
arm-none-eabi-size build/motor_controller.elf

echo ""
echo "🚀 Готов к прошивке!"
echo ""

# Спрашиваем, прошивать ли
read -p "Прошить плату сейчас? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🔌 Прошивка платы..."
    ./flash.sh build/motor_controller.bin
else
    echo "💾 Файл прошивки сохранен: build/motor_controller.bin"
    echo "Для прошивки запустите: ./flash.sh build/motor_controller.bin"
fi
