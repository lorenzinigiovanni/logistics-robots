FROM tiryoh/ubuntu-desktop-lxde-vnc:jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG ROS_DISTRO=humble
ARG INSTALL_PACKAGE=desktop

# Install ROS2
RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

# Install Gazebo
RUN apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-* && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

# Install turtlebot3 packages
RUN apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-gazebo && \
    rm -rf /var/lib/apt/lists/*

RUN gosu ubuntu rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /home/ubuntu/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /home/ubuntu/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

RUN echo 'sed -i -e "s/palette_color_4=rgb([0-9,]*)/palette_color_4=rgb(51,61,255)/g" $HOME/.config/lxterminal/lxterminal.conf' > /usr/local/bin/update-lxterminal.sh && \
    chmod +x /usr/local/bin/update-lxterminal.sh

ENV USER ubuntu
