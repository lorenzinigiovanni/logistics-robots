FROM tiryoh/ubuntu-desktop-lxde-vnc:jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
    wget \
    curl \
    git \
    build-essential \
    vim \
    sudo \
    lsb-release \
    locales \
    bash-completion \
    tzdata \
    gosu \
    gnupg2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -yq \
    ros-humble-desktop-full \
    ros-humble-gazebo-* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/ubuntu/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

ENV USER ubuntu
