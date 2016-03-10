#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

echo "[Updating & upgrading the packages]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Adding the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $1 main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Downloading the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Updating & upgrading the packages]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing ROS]"
sudo apt-get install -y ros-$2-desktop ros-$2-vision-visp qtcreator python-rosdep gitk python-sympy python-rosinstall

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
