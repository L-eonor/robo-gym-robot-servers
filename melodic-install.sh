#!/bin/sh

# Run this from the src folder
echo "Installing robot-servers..."
cd ~/robogym_ws/src
git clone -b melodic https://github.com/jr-robotics/mir_robot.git
git clone -b melodic https://github.com/jr-robotics/universal_robot.git
git clone -b melodic-devel https://github.com/L-eonor/robotiq.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git #for mimc joint plugin
#for the gazebo_grasp_plugin
sudo apt-get install ros-melodic-gazebo-ros
sudo apt-get install ros-melodic-eigen-conversions
sudo apt-get install ros-melodic-object-recognition-msgs
sudo apt-get install ros-melodic-roslint
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git

cd ..
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -qq -y python-rosdep python-catkin-tools
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y --rosdistro melodic
source /opt/ros/melodic/setup.bash
catkin init
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
pip install numpy scipy
sudo apt-get install ros-melodic-gripper-action-controller #gripper action controller
echo "Installed robot-servers"
