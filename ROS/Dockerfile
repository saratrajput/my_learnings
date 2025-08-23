FROM moveit/moveit:noetic-source

# Prepare workspace
WORKDIR /root/ws_moveit/src
RUN DEBIAN_FRONTEND=noninteractive git clone https://github.com/moveit/moveit_tutorials.git -b master \
    && git clone https://github.com/moveit/panda_moveit_config.git -b noetic-devel \
    && apt-get update \
    && rosdep update \
    && rosdep install -y --from-paths . --ignore-src --rosdistro noetic

# Build catkin packages
WORKDIR /root/ws_moveit
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build \
    && echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
