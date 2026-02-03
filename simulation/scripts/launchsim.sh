#!/bin/bash

SIM_TYPE=$1

source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

if [ $SIM_TYPE = "wallfollowing" ]
then
	ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
else
	ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py &
	gnome-terminal -- bash -c "
	ros2 run teleop_twist_keyboard teleop_twist_keyboard; bash"
fi

