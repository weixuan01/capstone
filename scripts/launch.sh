#!/bin/bash

MODE=$1
TYPE=$2

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")") # get dir of script
source $BASE_DIR/../simulation/crazyflie_mapping_demo/ros2_ws/install/setup.bash # assumes scripts and simulation folder are always in the same dir (capstone)
export GZ_SIM_RESOURCE_PATH="$BASE_DIR/../simulation/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"


display_help() {
    echo "Usage format: ./launch.sh <A> <B>"
    echo "    A: sim | real"
    echo "    B: wallfollowing | manual | square | frontier-exploration"
    exit 1
}

if [[ $MODE == "sim" ]] 
then   
    if [[ $TYPE == "wallfollowing" ]] 
    then
	    ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
    elif [[ $TYPE == "manual" ]] 
    then
	    ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py &
        gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; bash"
    elif [[ $TYPE == "square" ]]
    then
        ros2 launch crazyflie_ros2_multiranger_bringup square_mapper_simulation.launch.py
    elif [[ $TYPE == 'frontier-exploration' ]]
    then
        ros2 launch crazyflie_ros2_multiranger_bringup frontier_exploration_mapper_simulation.launch.py
    else
        display_help
    fi

elif [[ $MODE == "real" ]] 
then
    if [[ $TYPE == "wallfollowing" ]] 
    then
        ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
    elif [[ $TYPE == "manual" ]] 
    then
        ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py &
        gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; bash"
    elif [[ $TYPE == "square" ]]
    then
        ros2 launch crazyflie_ros2_multiranger_bringup square_mapper_real.launch.py
    elif [[ $TYPE == 'frontier-exploration' ]]
    then
        ros2 launch crazyflie_ros2_multiranger_bringup frontier_exploration_mapper_real.launch.py
    else
        display_help
    fi
    
else
    display_help
fi
        
