# Базовый образ с ROS 2 Humble
FROM osrf/ros:humble-desktop

# Настройка репозиториев ROS 2
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc  | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list

# Обновление списка пакетов
RUN apt-get update

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libpcap-dev \
    ros-humble-sensor-msgs \
    ros-humble-diagnostic-updater \
    #ros-humble-colcon-common-extensions \
    #ros-humble-laser-proc \
    #ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Копирование исходного кода

RUN pip3 install -U colcon-common-extensions

# Сборка пакетов ROS 2
WORKDIR /workspace
COPY src/ /workspace/src/
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# Настройка окружения
ENV COLCON_PREFIX_PATH=/workspace/install
ENV PATH=$COLCON_PREFIX_PATH/bin:$PATH
RUN . /opt/ros/humble/setup.sh && colcon build



# Скрипт запуска
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

# Команда запуска контейнера
CMD ["/launch.sh"]
