FROM docker.io/library/ros:humble-ros-base-jammy
SHELL ["/bin/bash","-o","pipefail","-c"]
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/osrf-gazebo.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/osrf-gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" >/etc/apt/sources.list.d/osrf.list && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      gz-harmonic libgz-sim8-dev libgz-plugin2-dev build-essential cmake python3-colcon-common-extensions \
      ros-humble-moveit ros-humble-robot-state-publisher ros-humble-rviz2 python3-numpy python3-scipy \
      libgl1-mesa-dri mesa-utils && rm -rf /var/lib/apt/lists/*
