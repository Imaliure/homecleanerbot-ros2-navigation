# Ubuntu 22.04 üzerine ROS2 Humble Desktop kurulumu
FROM osrf/ros:humble-desktop-full

# Gerekli sistem paketlerinin kurulumu
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-teleop-twist-keyboard \
    gedit \
    git \
    && rm -rf /var/lib/apt/lists/*

# Çalışma dizini oluşturma
WORKDIR /homecleanerbot_ws
COPY . .

# Projeyi build et
RUN . /opt/ros/humble/setup.sh && colcon build

# Kaynak dosyasını (source) bashrc'ye ekleme (her terminal açıldığında ROS hazır gelsin)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /homecleanerbot_ws/install/setup.bash" >> ~/.bashrc

# Başlangıç komutu
CMD ["bash"]
