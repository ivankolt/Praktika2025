#!/bin/bash

# Скрипт для прошивки STM32 с прошивкой motor_controller
# Использует stm32flash для прошивки через UART

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Функция для вывода сообщений
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Проверка наличия stm32flash
check_stm32flash() {
    if ! command -v stm32flash &> /dev/null; then
        print_error "stm32flash не найден. Установите его:"
        echo "sudo apt update"
        echo "sudo apt install git build-essential"
        echo "git clone https://github.com/stm32duino/stm32flash.git"
        echo "cd stm32flash && make && sudo make install"
        exit 1
    fi
    print_info "stm32flash найден"
}

# Определение UART порта
detect_uart_port() {
    if [ -e "/dev/serial0" ]; then
        UART_PORT="/dev/serial0"
        print_info "Используем /dev/serial0 (Raspberry Pi UART)"
    elif [ -e "/dev/ttyUSB0" ]; then
        UART_PORT="/dev/ttyUSB0"
        print_info "Используем /dev/ttyUSB0 (USB-UART адаптер)"
    else
        print_error "UART порт не найден!"
        echo "Доступные порты:"
        ls -la /dev/tty* /dev/serial* 2>/dev/null || echo "Порты не найдены"
        exit 1
    fi
}

# Проверка связи с загрузчиком
test_bootloader() {
    print_info "Проверка связи с загрузчиком..."
    
    # Пытаемся подключиться к загрузчику
    if timeout 3 stm32flash $UART_PORT > /dev/null 2>&1; then
        print_info "✓ Связь с загрузчиком установлена"
        return 0
    else
        print_warning "✗ Связь с загрузчиком не установлена"
        return 1
    fi
}

# Компиляция прошивки
compile_firmware() {
    print_info "Компиляция прошивки motor_controller..."
    
    # Проверяем наличие исходного файла
    if [ ! -f "stm32_motor_controller.c" ]; then
        print_error "Файл stm32_motor_controller.c не найден!"
        exit 1
    fi
    
    # Проверяем наличие Makefile
    if [ ! -f "Makefile" ]; then
        print_error "Makefile не найден!"
        exit 1
    fi
    
    # Компилируем
    if make motor_controller.bin; then
        print_info "✓ Прошивка скомпилирована успешно"
    else
        print_error "✗ Ошибка компиляции прошивки"
        exit 1
    fi
}

# Прошивка
flash_firmware() {
    local firmware_file="$1"
    
    if [ ! -f "$firmware_file" ]; then
        print_error "Файл прошивки $firmware_file не найден!"
        exit 1
    fi
    
    print_info "Прошивка $firmware_file на адрес 0x08002000..."
    
    # Прошиваем с проверкой
    if stm32flash -b 230400 -w "$firmware_file" -v -g 0x08002000 $UART_PORT; then
        print_info "✓ Прошивка завершена успешно!"
        print_info "Прошивка запущена по адресу 0x08002000"
    else
        print_error "✗ Ошибка прошивки!"
        exit 1
    fi
}

# Основная функция
main() {
    print_info "=== Прошивка STM32 Motor Controller ==="
    
    # Проверяем зависимости
    check_stm32flash
    detect_uart_port
    
    # Проверяем связь с загрузчиком
    if ! test_bootloader; then
        print_warning "Не удалось установить связь с загрузчиком"
        print_info "Попробуйте:"
        echo "1. Перезагрузить плату"
        echo "2. Нажать RESET дважды быстро (двойной сброс)"
        echo "3. Убедиться, что BOOT0 подключен правильно"
        echo ""
        read -p "Продолжить прошивку? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    # Определяем файл прошивки
    if [ $# -eq 1 ]; then
        FIRMWARE_FILE="$1"
    else
        # Ищем скомпилированную прошивку
        if [ -f "motor_controller.bin" ]; then
            FIRMWARE_FILE="motor_controller.bin"
        else
            print_info "Компилируем прошивку..."
            compile_firmware
            FIRMWARE_FILE="motor_controller.bin"
        fi
    fi
    
    # Прошиваем
    flash_firmware "$FIRMWARE_FILE"
    
    print_info "=== Прошивка завершена ==="
    print_info "Теперь можно тестировать связь:"
    echo "python3 test_hoverboard_connection.py --port $UART_PORT"
}

# Обработка аргументов командной строки
case "${1:-}" in
    -h|--help)
        echo "Использование: $0 [файл_прошивки.bin]"
        echo ""
        echo "Примеры:"
        echo "  $0                           # Автоматическая компиляция и прошивка"
        echo "  $0 motor_controller.bin      # Прошивка готового файла"
        echo "  $0 --help                    # Показать эту справку"
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac
