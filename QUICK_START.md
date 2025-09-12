# Быстрый старт - Hoverboard с ROS2

## 🚀 Запуск системы за 5 минут

### 1. Подготовка Raspberry Pi
```bash
# Включение UART
sudo raspi-config
# Interface Options → Serial Port → Отключить login shell, включить hardware

# Установка stm32flash
sudo apt update
sudo apt install git build-essential
git clone https://github.com/stm32duino/stm32flash.git
cd stm32flash && make && sudo make install

# Перезагрузка
sudo reboot
```

### 2. Прошивка STM32
```bash
# Компиляция и прошивка
make install-deps  # только при первом запуске
make
./flash_motor_controller.sh
```

### 3. Запуск ROS2 системы
```bash
# Автоматический запуск всей системы
./start_hoverboard_system.sh
```

### 4. Тестирование
```bash
# Отправка команды движения
docker-compose exec ros2-core ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Остановка
docker-compose exec ros2-core ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## 🔧 Полезные команды

```bash
# Статус системы
./start_hoverboard_system.sh status

# Логи hoverboard
./start_hoverboard_system.sh logs

# Интерактивное тестирование
./start_hoverboard_system.sh test

# Остановка системы
./start_hoverboard_system.sh stop
```

## 📋 Проверочный список

- [ ] UART включен в raspi-config
- [ ] stm32flash установлен
- [ ] STM32 прошит
- [ ] UART подключен (TX→RX, RX→TX)
- [ ] Питание подано на STM32 и моторы
- [ ] Система запущена без ошибок

## ⚠️ Безопасность

- Тестируйте на безопасной площадке
- Имейте возможность быстро отключить питание
- Проверьте все соединения перед запуском

## 🆘 Устранение неполадок

**Нет связи с загрузчиком:**
- Проверьте подключение TX/RX
- Попробуйте двойной сброс (быстро нажать RESET дважды)
- Проверьте питание

**Моторы не крутятся:**
- Проверьте логи: `docker-compose logs hoverboard`
- Убедитесь, что прошивка загружена
- Проверьте подключение моторов

**Контейнер не запускается:**
- Проверьте права доступа к UART
- Убедитесь, что порт не занят
