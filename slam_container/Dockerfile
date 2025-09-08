# Базовый образ ROS 2 Humble
FROM ros:humble

# Установка зависимостей
RUN apt update && \
    apt install -y ros-humble-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*

# Копирование твоего SLAM-пакета
COPY src/ /workspace/src

# Настройка рабочей директории
WORKDIR /workspace

# Инициализация рабочего пространства ROS
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Установка переменных окружения
ENV ROS_DOMAIN_ID=42

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Запуск SLAM при старте контейнера
CMD ["/entrypoint.sh"]
