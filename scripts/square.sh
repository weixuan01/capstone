#!/bin/bash

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")") # get dir of script
source $BASE_DIR/../simulation/crazyflie_mapping_demo/ros2_ws/install/setup.bash # assumes scripts and simulation folder are always in the same dir (capstone)
export GZ_SIM_RESOURCE_PATH="$BASE_DIR/../simulation/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

ros2 launch crazyflie_ros2_multiranger_bringup square_mapper_simulation.launch.py

