# Базовый образ с ROS 2 Humble
FROM osrf/ros:humble-desktop

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Установка pip-зависимостей
COPY requirements.txt /workspace/
RUN pip3 install -r /workspace/requirements.txt

# Копирование исходного кода
COPY src/sensor_package/ /workspace/src/sensor_package/

# Сборка пакетов ROS 2
WORKDIR /workspace
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
