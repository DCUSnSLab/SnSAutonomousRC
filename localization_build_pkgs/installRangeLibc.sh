#!/bin/bash

cd ~/Development/SnSAutonomousRC
sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro melodic -y

sudo pip install cython
git clone https://github.com/DCUSnSLab/range_libc_for_ROS_noetic.git
cd range_libc-for-Noetic/pywrapper

./compile.sh
