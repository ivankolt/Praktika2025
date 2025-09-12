# Настройка Hoverboard с ROS2 и STM32

Этот документ описывает настройку системы управления hoverboard через UART с использованием ROS2 и STM32 контроллера.

## Архитектура системы

```
Raspberry Pi (ROS2) <--UART--> STM32F103 <--PWM--> Hoverboard Motors
```

## Компоненты

1. **Raspberry Pi** - основной компьютер с ROS2
2. **STM32F103C8** - контроллер моторов
3. **UART соединение** - связь между Pi и STM32
4. **Hoverboard** - платформа с двумя моторами

## Протокол связи

### Формат пакета (8 байт)
```
[steer_low][steer_high][speed_low][speed_high][crc32_low][crc32_med1][crc32_med2][crc32_high]
```

- **steer**: -1000 до 1000 (поворот)
- **speed**: -1000 до 1000 (скорость)
- **crc32**: контрольная сумма для проверки целостности

### Преобразование Twist в команды моторов

```python
# Дифференциальный привод
left_speed = linear_speed - (angular_speed * wheel_separation / 2.0)
right_speed = linear_speed + (angular_speed * wheel_separation / 2.0)

# Преобразование в протокол
steer = right_speed - left_speed
speed = (left_speed + right_speed) // 2
```

## Установка и настройка

### 1. Настройка Raspberry Pi

#### Включение UART
```bash
sudo raspi-config
# Interface Options → Serial Port
# Отключить "Login shell over serial"
# Включить "Serial port hardware"
sudo reboot
```

#### Проверка UART
```bash
ls -l /dev/serial0
# Должно показать: /dev/serial0 -> ttyS0
```

#### Установка stm32flash
```bash
sudo apt update
sudo apt install git build-essential
git clone https://github.com/stm32duino/stm32flash.git
cd stm32flash
make
sudo make install
```

### 2. Компиляция и прошивка STM32

#### Установка зависимостей
```bash
make install-deps
```

#### Компиляция прошивки
```bash
make
```

#### Прошивка платы
```bash
# Автоматическая прошивка
./flash_motor_controller.sh

# Или через Makefile
make flash
```

### 3. Запуск ROS2 системы

#### Сборка Docker контейнеров
```bash
docker-compose build
```

#### Запуск системы
```bash
docker-compose up -d
```

#### Проверка работы
```bash
# Проверка статуса контейнеров
docker-compose ps

# Логи hoverboard контейнера
docker-compose logs hoverboard
```

## Тестирование

### 1. Тест связи с hoverboard
```bash
# Автоматический тест
python3 test_hoverboard_connection.py

# Интерактивный тест
python3 test_hoverboard_connection.py --interactive
```

### 2. Тест через ROS2
```bash
# Отправка команды движения
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Остановка
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### 3. Проверка статуса
```bash
# Статус моторов
ros2 topic echo /motor_status

# Список активных топиков
ros2 topic list
```

## Конфигурация

### Параметры ROS2 ноды
- `serial_port`: UART порт (по умолчанию: /dev/serial0)
- `baud_rate`: скорость UART (по умолчанию: 115200)
- `max_linear_speed`: максимальная линейная скорость (м/с)
- `max_angular_speed`: максимальная угловая скорость (рад/с)
- `wheel_separation`: расстояние между колесами (м)

### Изменение параметров
```bash
# Через параметры при запуске
ros2 run motor_controller_package motor_controller_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p max_linear_speed:=2.0
```

## Устранение неполадок

### Проблема: Нет связи с загрузчиком
**Решение:**
1. Проверьте подключение TX/RX
2. Убедитесь, что питание подано на плату
3. Попробуйте двойной сброс (быстро нажать RESET дважды)
4. Проверьте настройки UART в raspi-config

### Проблема: Моторы не крутятся
**Решение:**
1. Проверьте логи ROS2 ноды
2. Убедитесь, что прошивка загружена правильно
3. Проверьте подключение моторов к STM32
4. Проверьте питание моторов

### Проблема: Неправильное направление движения
**Решение:**
1. Поменяйте местами провода моторов
2. Или измените логику в прошивке STM32

### Проблема: Контейнер не запускается
**Решение:**
1. Проверьте права доступа к UART порту
2. Убедитесь, что порт не занят другим процессом
3. Проверьте логи контейнера: `docker-compose logs hoverboard`

## Безопасность

⚠️ **ВАЖНО**: 
- Всегда тестируйте на безопасной площадке
- Убедитесь, что hoverboard не может упасть или повредить что-либо
- Имейте возможность быстро отключить питание
- Проверьте все соединения перед первым запуском

## Дополнительные возможности

### Интеграция с навигацией
Система готова для интеграции с:
- SLAM (картографирование)
- Nav2 (навигация)
- Odometry (одометрия)

### Мониторинг
- Статус соединения через топик `/motor_status`
- Логи через Docker
- Возможность добавления телеметрии

## Поддержка

При возникновении проблем:
1. Проверьте логи: `docker-compose logs`
2. Проверьте статус топиков: `ros2 topic list`
3. Проверьте параметры ноды: `ros2 param list`
4. Обратитесь к документации ROS2 и STM32
