# Базовый образ ROS 2 Humble
FROM ros:humble

# Установка зависимостей Nav2
RUN apt update && \
    apt install -y ros-humble-navigation2 ros-humble-nav2-* && \
    rm -rf /var/lib/apt/lists/*

# Копирование конфигурационных файлов
COPY nav2_config/ /workspace/nav2_config/

# Настройка рабочей директории
WORKDIR /workspace

# Запуск Nav2 при старте контейнера
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup navigation_launch.py"]
