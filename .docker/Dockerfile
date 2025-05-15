ARG ROS_DISTRO=jazzy
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN curl -ssL -O https://security.ubuntu.com/ubuntu/pool/main/libc/libcap2/libcap-dev_2.66-5ubuntu2.2_amd64.deb \
    && dpkg -i libcap-dev_2.66-5ubuntu2.2_amd64.deb \
    && rm libcap-dev_2.66-5ubuntu2.2_amd64.deb

RUN pip3 install --no-cache-dir --break-system-packages\
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    mypy \
    prospector \
    pylint \
    numpy \
    opencv-python \
    panda \
    matplotlib \
    rosbags     

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
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
    ros-$ROS_DISTRO-trac-ik-kinematics-plugin \
    ros-$ROS_DISTRO-ament-clang-format \
    ros-$ROS_DISTRO-moveit-visual-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# niceness permissions (for ur driver use without Prempt-RT)
RUN echo "* - nice -15" >> /etc/security/limits.conf

FROM base AS camera

RUN curl -ssL -o Mech-Eye_API_2.5.0_amd64.zip "https://cdn-obj.azureedge.net/community/community_downloads/Mech-Eye%20SDK%202.5.0/Mech-Eye_API_2.5.0_amd64.zip?se=4763-03-21T07%3A44%3A16Z&sp=r&sv=2023-11-03&sr=b&sig=KFqefXy%2BD%2B35t4BDQjm8zgbdGVA6/nwAUTqwL4djB5g%3D" \
    && unzip Mech-Eye_API_2.5.0_amd64.zip \
    && rm Mech-Eye_API_2.5.0_amd64.zip \
    && dpkg -i 'Mech-Eye_API_2.5.0_amd64.deb' \
    && dpkg -l | grep mecheyeapi \
    && rm -rf 'Mech-Eye_API_2.5.0_amd64.deb'


RUN pip3 install --no-cache-dir --break-system-packages open3d --ignore-installed PyYAML


FROM camera AS build

RUN mkdir -p /dev_ws/src
WORKDIR /dev_ws/src 

WORKDIR /dev_ws

COPY . /dev_ws/src

FROM build AS bash

RUN sed -i 's/\(\s*\)_Bool,/\1BoolType,/' /opt/mech-mind/mech-eye-sdk/include/Parameter.h


RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"] 


RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]

ENTRYPOINT ["bash", "/entrypoint.bash"]

CMD ["bash"]