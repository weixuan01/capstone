#!/bin/bash

MODE=$1
TYPE=$2
MAP=$3

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")
SIM_LAUNCH_PATH="$BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_bringup/launch/crazyflie_simulation.launch.py"

display_help() {
    echo "Usage format: ./launch.sh <MODE> <TYPE> [MAP]"
    echo "    MODE: sim | real"
    echo "    TYPE: wallfollowing | manual | square | frontier-exploration | map-user"
    echo "    MAP: (sim wallfollowing/manual/square/frontier-exploration)"
    echo "              crazyflie_world | custom | maze | circle-maze"
    echo "         (sim/real map-user, optional)"
    echo "              Absolute path to a saved map.yaml — pre-loads that map"
    echo "              into the shared mapper.  Omit to start with a blank map."
    exit 1
}

source $BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$BASE_DIR/../core/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

if [[ -z "$MODE" ]] || [[ -z "$TYPE" ]]
then
    display_help
fi

if [[ $MODE == "sim" ]]
then
    if [[ "$TYPE" != "wallfollowing" && "$TYPE" != "manual" && "$TYPE" != "square" \
       && "$TYPE" != "frontier-exploration" && "$TYPE" != "map-user" ]]
    then
        display_help
    fi

    if [[ $TYPE == "wallfollowing" || $TYPE == "manual" || $TYPE == "square" \
       || $TYPE == "frontier-exploration" ]]
    then
        # For these types MAP selects the simulation world, not a map file.
        if [[ -n "$MAP" ]]
        then
            if ! grep -q "'$MAP.sdf -r'" $SIM_LAUNCH_PATH
            then
                sed -i "s/'.*\.sdf -r'/'$MAP.sdf -r'/" $SIM_LAUNCH_PATH
                $BASE_DIR/build-project.sh
                source $BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws/install/setup.bash
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
        elif [[ $TYPE == "frontier-exploration" ]]
        then
            ros2 launch crazyflie_ros2_multiranger_bringup frontier_exploration_mapper_simulation.launch.py
        fi

    elif [[ $TYPE == "map-user" ]]
    then
        # MAP is an optional path to a saved map.yaml.
        # When provided, the shared mapper loads it on startup.
        if [[ -n "$MAP" ]]
        then
            ros2 launch crazyflie_ros2_multiranger_bringup map_user_simulation.launch.py \
                map_file:="$MAP"
        else
            ros2 launch crazyflie_ros2_multiranger_bringup map_user_simulation.launch.py
        fi
    fi

elif [[ $MODE == "real" ]]
then
    if [[ "$TYPE" != "wallfollowing" && "$TYPE" != "manual" && "$TYPE" != "square" \
       && "$TYPE" != "frontier-exploration" && "$TYPE" != "map-user" ]]
    then
        display_help
    fi

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
    elif [[ $TYPE == "frontier-exploration" ]]
    then
        ros2 launch crazyflie_ros2_multiranger_bringup frontier_exploration_mapper_real.launch.py
    elif [[ $TYPE == "map-user" ]]
    then
        if [[ -n "$MAP" ]]
        then
            ros2 launch crazyflie_ros2_multiranger_bringup map_user_real.launch.py \
                map_file:="$MAP"
        else
            ros2 launch crazyflie_ros2_multiranger_bringup map_user_real.launch.py
        fi
    fi

else
    display_help
fi
