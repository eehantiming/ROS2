# ROS2

## Introduction
- Dockerfile for Ubuntu 20 + ROS2 foxy desktop
- Builds basic python and interface package
- Add build instructions in package.xml CMakeLists/setup.py before running `colcon build --packages-select {packagename}`. you need to source bashrc for new nodes

## Instructions
1. git clone and `cd ROS2`
2. build docker image with 'docker build -t foxy_tutorial .'
3. start a container with 'xhost +local:docker && docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $PWD/py_nodes:/root/ros2_ws/src/my_py_pkg/my_py_pkg/py_nodes -v $PWD/interfaces:/root/ros2_ws/src/my_robot_interfaces/interfaces  --name foxy foxy_tutorial'
4. To add files with volume add additional -v tag OR `docker cp {filename} {container name}:/root/ros2_ws/src/my_py_pkg/my_py_pkg/`
5. test installation with `ros2 run demo_nodes_cpp talker`
6. test display with `rqt` or `ros2 run turtlesim turtlesim_node`
7. to start another shell in the container, open a new terminal and `docker exec -it foxy bash`