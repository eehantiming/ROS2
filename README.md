# ROS2 docker container

## Introduction
- Docker container for Ubuntu 20 + ROS2 Foxy desktop. Make sure you have [Docker engine](https://docs.docker.com/engine/install/ubuntu/) installed.
- Builds basic python and interface package
- Add build instructions in package.xml and CMakeLists.txt/setup.py before running `colcon build --packages-select {packagename}`. you need to source bashrc for building of new packages

## Instructions
1. git clone and `cd ROS2`
2. Build docker image with `docker build -t foxy_tutorial .`
3. Start a container with `xhost +local:docker && docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $PWD/py_nodes:/root/ros2_ws/src/my_py_pkg/my_py_pkg/py_nodes -v $PWD/interfaces:/root/ros2_ws/src/my_robot_interfaces/interfaces  --name foxy foxy_tutorial`
4. To add your own files, use an additional -v tag when starting the container OR   
`docker cp {filename} foxy:/root/ros2_ws/src/my_py_pkg/my_py_pkg/`
5. You can test your installation with `ros2 run demo_nodes_cpp talker` and test display with `rqt` or `ros2 run turtlesim turtlesim_node`
7. To start another shell in the running container, open a new terminal and `docker exec -it foxy bash`