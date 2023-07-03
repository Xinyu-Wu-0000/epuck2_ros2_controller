# epuck2 mobile robot controller based on ros2-iron

## Prerequisites

1. install [docker engine](https://docs.docker.com/engine/install/ubuntu/)
2. install [ros2](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) or use [ros1_bridge](https://github.com/ros2/ros1_bridge)
3. install [firmware](https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#WiFi) in epuck2 robot


## Usage

1. setup epuck2 [wifi connection](https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#Connecting_to_the_WiFi)
2. run the docker container with command `docker run --name=epuck2 -it  xinyu0000/epuck2_ros2_controller:latest`
3. add ip, id and configs to  `/root/epuck2_ros2_controller/src/epuck2_driver_cpp/launch/epuck2_controller.py`
4. run `/root/epuck2_ros2_controller/src/epuck2_driver_cpp/start.sh`
5. then you can use [rqt](https://docs.ros.org/en/iron/Concepts/About-RQt.html) or use your own program to get sensor data and send command.
