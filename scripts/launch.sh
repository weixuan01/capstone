#!/bin/bash

MODE=$1
TYPE=$2
MAP=$3

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")") # Get dir of script
SIM_LAUNCH_PATH="$BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_bringup/launch/crazyflie_simulation.launch.py"

display_help() {
    echo "Usage format: ./launch.sh <MODE> <TYPE> [MAP]"
    echo "    MODE: sim | real"
    echo "    TYPE: wallfollowing | manual | square | frontier-exploration"
    echo "    MAP: (Optional, for sim only) crazyflie_world | custom | maze | circle-maze"
    exit 1 # Terminates the script
}

source $BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/install/setup.bash # Assumes scripts and core folder are always in the same dir (capstone)
export GZ_SIM_RESOURCE_PATH="$BASE_DIR/../core/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

if [[ -z "$MODE" ]] || [[ -z "$TYPE" ]] # If no. of input parameters < 2
then
    display_help
fi

if [[ $MODE == "sim" ]] 
then
    # If type is not valid
    if [[ "$TYPE" != "wallfollowing" && "$TYPE" != "manual" && "$TYPE" != "square" && "$TYPE" != "frontier-exploration" ]]
    then
        display_help
    fi

    # Check if map parameter is not empty
    if [[ -n "$MAP" ]]
    then
        # Check if the input map name is not the current map, otherwise skip overwrite
        if ! grep -q "'$MAP.sdf -r'" $SIM_LAUNCH_PATH
        then
            # sed -i 's/pattern/replacement/'
            sed -i "s/'.*\.sdf -r'/'$MAP.sdf -r'/" $SIM_LAUNCH_PATH

            # Rebuild project
            $BASE_DIR/build-project.sh
            source $BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/install/setup.bash # Re-source after rebuilding
        fi
    fi

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
        
