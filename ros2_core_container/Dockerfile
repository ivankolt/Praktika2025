# Базовый образ с ROS 2 Humble
FROM osrf/ros:humble-desktop

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /workspace

# Запуск ros2 daemon
CMD ["ros2", "daemon"]
