#!/bin/bash
cd ~/Development/SnSAutonomousRC

pip install recordclass
# this one will take a while
pip install scikit-image

sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro melodic -y
