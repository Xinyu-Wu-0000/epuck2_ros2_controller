FROM ros:iron-ros-base

RUN mkdir -p /root/epuck2_ros2_controller/src

COPY ./src /root/epuck2_ros2_controller/src

RUN apt update && \
    apt install ros-iron-cv-bridge \
    -y

RUN cd /root/epuck2_ros2_controller && \
    . /opt/ros/iron/setup.sh && \
    colcon build --symlink-install

RUN echo 'alias ros-env="source /opt/ros/iron/setup.bash"' >> /root/.bashrc

COPY ./start.sh /root/epuck2_ros2_controller
RUN chmod +x /root/epuck2_ros2_controller/start.sh

WORKDIR /root/epuck2_ros2_controller
