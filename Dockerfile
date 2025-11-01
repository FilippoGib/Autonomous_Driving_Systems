FROM ubuntu:24.04

# set a non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
      build-essential cmake git curl locales software-properties-common \
      libpcl-dev libboost-all-dev libeigen3-dev \
      mesa-utils libgl1-mesa-dri libglx-mesa0 libgl1 libglu1-mesa \
      libx11-6 libxext6 libxrender1 libsm6 libice6 \
      libxrandr2 libxinerama1 libxcursor1 libxi6 libceres-dev && \
    # ROS2 rolling installation
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -oP '"tag_name": "\K[^"]*') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y ros-rolling-ros-base ros-dev-tools ros-rolling-pcl-conversions  && \
    # clean up apt lists and temporary files to reduce final image size
    rm -rf /var/lib/apt/lists/* /tmp/*.deb

# by default source ros in every shell
RUN echo "source /opt/ros/rolling/setup.bash" >> /root/.bashrc

WORKDIR /app

CMD ["/bin/bash"]
