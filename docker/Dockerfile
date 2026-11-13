
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger

WORKDIR /workspace/robust_nonlinear_controller

CMD ["bash"]
