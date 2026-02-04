#!/bin/bash

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")") # get dir of script
cd $BASE_DIR/../crazyflie_mapping_demo/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON