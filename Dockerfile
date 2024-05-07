FROM osrf/ros:humble-desktop-jammy

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

##### Setup Gazebo #####
# essential install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    binutils \
    mesa-utils \
    kmod \
    x-window-system \
    curl

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN curl -sSL http://get.gazebosim.org | sh
########################

WORKDIR /home/dev_ws
RUN mkdir src

# necessary ros packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# personal packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    vim \
    gdb \
    gdbserver

# setup bashrc settings
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc && \
    echo "alias b='source /home/dev_ws/install/local_setup.bash'" >> ~/.bashrc