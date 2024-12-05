#!/bin/bash

echo "RoboTemplate Setup Script"
echo "========================"

# Check OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Install ROS
    echo "Installing ROS..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-noetic-desktop-full

    # Install additional ROS packages
    sudo apt install -y \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins

elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS setup
    echo "Installing dependencies for macOS..."
    brew install python@3.8
    pip3 install rosdep rospkg catkin_pkg
    
    echo "Note: For full ROS functionality, please consider using Docker or a Linux VM"
    echo "Docker setup instructions can be found in docs/docker_setup.md"
fi

# Create catkin workspace
mkdir -p ../catkin_ws/src
cd ../catkin_ws
catkin_make

# Install Python dependencies
pip3 install -r ../requirements.txt

echo "Setup complete! Please source your workspace:"
echo "source devel/setup.bash" 