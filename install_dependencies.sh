#!/bin/bash

set -e  # Dừng nếu có lỗi

sudo apt update
sudo apt install -y \
    libomp-dev \
    python3-colcon-common-extensions \
    rviz \
    ros-noetic-tf2 \
    ros-noetic-tf2-msgs \
    ros-noetic-tf2-tools \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-eigen-conversions \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-geometry-msgs \
    libgoogle-glog-dev \
    ros-noetic-libnabo \
    x11-xserver-utils \
    wget \
    ubuntu-drivers-common \
    cmake \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags2.2 libgflags-dev \
    liblapack-dev libopenblas-dev \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-geographic-msgs \
    libgeographic-dev \
    ros-noetic-plotjuggler-ros

echo "Cài đặt hoàn tất!"
