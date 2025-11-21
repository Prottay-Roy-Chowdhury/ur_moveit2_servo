ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color


RUN apt-get update && apt-get install -y --no-install-recommends\
    wget \
    curl \
    terminator \
    iputils-ping \
    openssh-server \
    iproute2 \
    python3-pip \
    nano \
    build-essential \
    cmake \
    git \
    unzip \
    xterm \
    zsh \
    zenity \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-vcstool \
    python3-argcomplete \
    python3-rosdep \
    freeglut3-dev \
    lsb-release gnupg \
    libglew-dev \
    libqt5opengl5-dev \
    libarchive-tools \
    libpcl-dev \
    libcap-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir --ignore-installed \
    ruff \
    black \
    numpy \
    opencv-python \
    pandas \
    matplotlib \
    setuptools \
    rerun-sdk \
    rosbags
    
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rosbag2-py \
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-ur-msgs \
    ros-$ROS_DISTRO-ur-description \
    ros-$ROS_DISTRO-ur-client-library \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-kinematics-interface \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-pilz-industrial-motion-planner \
    ros-$ROS_DISTRO-ament-clang-format \
    ros-$ROS_DISTRO-moveit-visual-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# niceness permissions (for ur driver use without Prempt-RT)
RUN echo "* - nice -15" >> /etc/security/limits.conf

FROM base AS camera

RUN curl -ssL -o Mech-Eye_API_2.5.1_amd64.zip "https://cdn-obj.azureedge.net/community/community_downloads/community_downloads/Mech-Eye_API_2.5.1_amd64.zip?se=4763-06-18T07%3A27%3A08Z&sp=r&sv=2023-11-03&sr=b&sig=T%2BnKM9Q6gze19THtJQmOo/aaJmi2CzRoYB6L1XYmoHg%3D" \
    && unzip Mech-Eye_API_2.5.1_amd64.zip \
    && rm Mech-Eye_API_2.5.1_amd64.zip \
    && dpkg -i 'Mech-Eye_API_2.5.1_amd64.deb' \
    && dpkg -l | grep mecheyeapi \
    && rm -rf 'Mech-Eye_API_2.5.1_amd64.deb'


RUN pip3 install --no-cache-dir open3d --ignore-installed PyYAML


FROM camera AS build

RUN mkdir -p /dev_ws/src
WORKDIR /dev_ws/src 

COPY . /dev_ws/src
RUN pip3 install setuptools==79.0.1
WORKDIR /dev_ws
FROM build AS bash

RUN sed -i \
    's|<origin xyz="0 0 0" rpy="0 0 0"[ ]*/>|<origin xyz="0 0 0" rpy="0 0 -${pi}" />|' \
    /opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro

# RUN sed -i 's/\(\s*\)_Bool,/\1BoolType,/' /opt/mech-mind/mech-eye-sdk/include/Parameter.h



RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"] 


RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]

ENTRYPOINT ["bash", "/entrypoint.bash"]

CMD ["bash"]

FROM build AS zsh

SHELL ["zsh", "chsh -s ~/.zshrc" ]

RUN ["/bin/zsh", "-c", "source /opt/ros/$ROS_DISTRO/setup.sh && \
     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"]
RUN ["/bin/zsh", "-c", "cp /dev_ws/src/.docker/setup.zsh /dev_ws/setup.zsh && chmod 777 /dev_ws/setup.zsh"]
RUN ["/bin/zsh", "-c", "cp /dev_ws/src/.docker/entrypoint.zsh / && chmod 777 /entrypoint.zsh"]  

ENTRYPOINT ["zsh", "/entrypoint.zsh"]

CMD ["zsh"]