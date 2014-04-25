#!/bin/bash
# installe ROS et met en place les repertoires pour le TP 

echo "Init ROS repositories"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu lucid main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update

echo "Installing ROS packages"
sudo apt-get install -yq ros-fuerte-ros-comm ros-fuerte-rviz python-rosinstall python-rosdep gitk qtcreator ros-fuerte-vision-visp ros-fuerte-pr2-controllers ros-fuerte-simulator-gazebo

echo "Setup ROS environment"
echo "# ROS stuff" >> ~/.bashrc
echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=~/ros_workspace:/opt/ros/fuerte/share:/opt/ros/fuerte/stacks" >> ~/.bashrc
echo "export ROS_WORKSPACE=~/ros_workspace" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update

echo "Downloading final packages"
mkdir ~/ros_workspace
cp $(cd `dirname "$0"` && pwd)/`basename "$0"` ~/ros_workspace/
cd ~/ros_workspace
git clone https://github.com/oKermorgant/tpRobots.git
rospack profile

echo ""
echo "Packages are ready to be used"
