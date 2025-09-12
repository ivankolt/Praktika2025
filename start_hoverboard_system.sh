#!/bin/bash

# Скрипт для запуска всей системы hoverboard
# Включает проверку зависимостей, сборку и запуск

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Функции для вывода
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Проверка зависимостей
check_dependencies() {
    print_header "Проверка зависимостей"
    
    # Проверка Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker не установлен"
        exit 1
    fi
    print_info "✓ Docker установлен"
    
    # Проверка Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose не установлен"
        exit 1
    fi
    print_info "✓ Docker Compose установлен"
    
    # Проверка Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python3 не установлен"
        exit 1
    fi
    print_info "✓ Python3 установлен"
    
    # Проверка UART портов
    if [ -e "/dev/serial0" ] || [ -e "/dev/ttyUSB0" ]; then
        print_info "✓ UART порты доступны"
    else
        print_warning "UART порты не найдены"
    fi
}

# Проверка и настройка UART
setup_uart() {
    print_header "Настройка UART"
    
    # Проверяем, включен ли UART
    if ! grep -q "enable_uart=1" /boot/config.txt 2>/dev/null; then
        print_warning "UART может быть не включен в /boot/config.txt"
        print_info "Добавьте 'enable_uart=1' в /boot/config.txt и перезагрузитесь"
    fi
    
    # Проверяем права доступа
    if [ -e "/dev/serial0" ]; then
        if groups | grep -q "dialout"; then
            print_info "✓ Пользователь в группе dialout"
        else
            print_warning "Добавьте пользователя в группу dialout: sudo usermod -a -G dialout $USER"
        fi
    fi
}

# Сборка контейнеров
build_containers() {
    print_header "Сборка Docker контейнеров"
    
    print_info "Сборка контейнеров..."
    if docker-compose build; then
        print_info "✓ Контейнеры собраны успешно"
    else
        print_error "Ошибка сборки контейнеров"
        exit 1
    fi
}

# Запуск системы
start_system() {
    print_header "Запуск системы"
    
    print_info "Запуск контейнеров..."
    if docker-compose up -d; then
        print_info "✓ Контейнеры запущены"
    else
        print_error "Ошибка запуска контейнеров"
        exit 1
    fi
    
    # Ждем запуска
    print_info "Ожидание запуска сервисов..."
    sleep 5
    
    # Проверяем статус
    print_info "Статус контейнеров:"
    docker-compose ps
}

# Проверка работы системы
check_system() {
    print_header "Проверка работы системы"
    
    # Проверяем ROS2 топики
    print_info "Проверка ROS2 топиков..."
    if docker-compose exec -T ros2-core ros2 topic list | grep -q "cmd_vel"; then
        print_info "✓ ROS2 топики доступны"
    else
        print_warning "ROS2 топики не найдены"
    fi
    
    # Проверяем статус hoverboard
    print_info "Проверка статуса hoverboard..."
    if docker-compose logs hoverboard | grep -q "Motor Controller Node запущен"; then
        print_info "✓ Hoverboard нода запущена"
    else
        print_warning "Hoverboard нода может не работать"
    fi
}

# Показать инструкции
show_instructions() {
    print_header "Инструкции по использованию"
    
    echo "Система запущена! Теперь вы можете:"
    echo ""
    echo "1. Отправить команду движения:"
    echo "   docker-compose exec ros2-core ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}\""
    echo ""
    echo "2. Остановить движение:"
    echo "   docker-compose exec ros2-core ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}\""
    echo ""
    echo "3. Проверить статус моторов:"
    echo "   docker-compose exec ros2-core ros2 topic echo /motor_status"
    echo ""
    echo "4. Посмотреть логи:"
    echo "   docker-compose logs hoverboard"
    echo ""
    echo "5. Остановить систему:"
    echo "   docker-compose down"
    echo ""
    echo "6. Тестирование связи:"
    echo "   python3 test_hoverboard_connection.py --interactive"
}

# Остановка системы
stop_system() {
    print_header "Остановка системы"
    
    print_info "Остановка контейнеров..."
    docker-compose down
    print_info "✓ Система остановлена"
}

# Основная функция
main() {
    case "${1:-start}" in
        start)
            check_dependencies
            setup_uart
            build_containers
            start_system
            check_system
            show_instructions
            ;;
        stop)
            stop_system
            ;;
        restart)
            stop_system
            sleep 2
            main start
            ;;
        status)
            print_header "Статус системы"
            docker-compose ps
            echo ""
            print_info "Логи hoverboard:"
            docker-compose logs --tail=10 hoverboard
            ;;
        test)
            print_header "Тестирование связи"
            python3 test_hoverboard_connection.py --interactive
            ;;
        logs)
            docker-compose logs -f hoverboard
            ;;
        help|--help|-h)
            echo "Использование: $0 [команда]"
            echo ""
            echo "Команды:"
            echo "  start     - Запуск системы (по умолчанию)"
            echo "  stop      - Остановка системы"
            echo "  restart   - Перезапуск системы"
            echo "  status    - Показать статус"
            echo "  test      - Интерактивное тестирование"
            echo "  logs      - Показать логи hoverboard"
            echo "  help      - Показать эту справку"
            ;;
        *)
            print_error "Неизвестная команда: $1"
            echo "Используйте '$0 help' для справки"
            exit 1
            ;;
    esac
}

# Обработка Ctrl+C
trap 'print_info "Прервано пользователем"; exit 1' INT

# Запуск
main "$@"
