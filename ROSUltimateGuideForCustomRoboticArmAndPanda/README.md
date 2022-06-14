# ROS Ultimate guide for Custom Robotic Arms and Pand 7 DOF

```
# Install ROS-Noetic on Ubuntu 20
# http://wiki.ros.org/noetic/Installation/Ubuntu

sudo apt install ros-noetic-joint-state-publisher*
sudo apt install ros-noetic-robot-state-publisher
```

### Create custom package "Bazu"

```
mkdir -p robotic_arms_ws/src
cd robotic_arms_ws/src/

# Initialize workspace
catkin_init_workspace

# Create package
catkin_create_pkg bazu std_msgs rospy
cd ..
catkin_make

cd src/bazu/
mkdir urdf launch world

# Create urdf
cd urdf
touch bazu.urdf

# Install ROS Snippets extension for VS Code. Search for xml ros.
```


