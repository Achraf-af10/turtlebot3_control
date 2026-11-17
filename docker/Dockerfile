
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-teleop \
    ros-humble-diagnostic-msgs \
    ros-humble-std-srvs \
    python3-pytest \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger

WORKDIR /workspace/turtlebot3_control

COPY src ./src
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

CMD ["bash"]
