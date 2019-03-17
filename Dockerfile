# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

#
# ROS
# from https://github.com/osrf/docker_images/blob/eaca344ae304c30254451da89bae328eb65ee385/ros/melodic/ubuntu/bionic/ros-core/Dockerfile

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
	ros-melodic-desktop \
    && rm -rf /var/lib/apt/lists/*

#
# Other things for ROS

RUN apt-get update && apt-get install -q -y \
	ninja-build python-pip python-dev python-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U \
    catkin-tools \
    jinja2

# Nice backtraces for backward-cpp

RUN apt-get update && apt-get install -q -y \
    binutils-dev \
    && rm -rf /var/lib/apt/lists/*

#
# Project

RUN apt-get update && apt-get install -q -y \
    autoconf \
    ccache \
    clang-format-6.0 \
    libopenvdb-dev \
    htop \
    ros-melodic-octomap \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-pcl-ros \
    ros-melodic-tf2-eigen \
    rsync \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U \
    matplotlib \
    numpy \
    pandas \
    seaborn \
    tabulate \
    toml \
    zpretty \
    && rm -rf $HOME/.cache/pip

#
#

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
