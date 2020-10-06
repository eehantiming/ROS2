# docker build -t foxy_tutorial .
# xhost +local:docker && docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $PWD/py_nodes:/root/ros2_ws/src/my_py_pkg/my_py_pkg/py_nodes -v $PWD/interfaces:/root/ros2_ws/src/my_robot_interfaces/interfaces  --name foxy foxy_tutorial


FROM osrf/ros:foxy-desktop
SHELL ["/bin/bash", "-c"]

# RUN sudo apt install python3-colcon-common-extensions # already in foxy desktop
RUN apt-get update && apt-get install -y vim

## Set up bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc # exists after the first colcon build
RUN ln -s /opt/ros/foxy/lib/libconsole_bridge.so /opt/ros/foxy/lib/libconsole_bridge.so.1.0 #required for cpp example node to run ros2 run demo_nodes_cpp talker

## Set up ros2 workspace and build packages
RUN mkdir -p ~/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN source /opt/ros/foxy/setup.bash && ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
# COPY my_first_node.py /root/ros2_ws/src/my_py_pkg/my_py_pkg/my_first_node.py

RUN source /opt/ros/foxy/setup.bash && ros2 pkg create my_robot_interfaces
RUN cd my_robot_interfaces && rm -rf include/ && rm -rf src/

# docker cp number_publisher.py {container name}:/root/ros2_ws/src/my_py_pkg/my_py_pkg/
# need to edit setup.py in python package "py_test = my_py_pkg.my_first_node:main"
# cd ~/ros2_ws && colcon build --package-select my_py_pkg --symlink-install

# cd ~/ros2_ws && colcon build --package-select my_robot_interfaces
# in my_robot_interfaces.package.xml, add <build_depend>rosidl_default_generators</build_depend>; <exec_depend>rosidl_default_runtime</exec_depend>; <member_of_group>rosidl_interface_packages</member_of_group>
# in CMakeLists.txt, delete C99 and if build testing block; add find_package(rosidl_default_generators REQUIRED); rosidl_generate_interfaces(${PROJECT_NAME} "interfaces/msg/LedState.msg" "")
# cd ~/ros2_ws && colcon build --package-select my_robot_interfaces

# source ~/.bashrc