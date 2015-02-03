#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq


echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $1 main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing ROS]"
sudo apt-get install -y ros-$2-desktop-full ros-$2-rqt-* ros-$2-vision-visp qtcreator python-rosdep gitk python-sympy

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/$2/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/ros/src
cd ~/ros/src
catkin_init_workspace
cd ~/ros/
catkin_make
