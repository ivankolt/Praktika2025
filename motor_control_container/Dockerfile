# Базовый multi-arch образ ROS 2 Humble (автоматически подтянет arm64 на Raspberry Pi)
FROM ros:humble-desktop

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
 && rm -rf /var/lib/apt/lists/*

# Установка pyserial
RUN pip3 install --no-cache-dir pyserial

# Копирование исходников
WORKDIR /workspace
COPY src/ /workspace/src/

# Сборка пакетов ROS 2
RUN . /opt/ros/humble/setup.sh && colcon build

# Настройка окружения
ENV COLCON_PREFIX_PATH=/workspace/install
ENV PATH=$COLCON_PREFIX_PATH/bin:$PATH
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Скрипт запуска
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

# Команда запуска контейнера
CMD ["/launch.sh"]
