#!/bin/bash

# Command to stop and land drone, preventing it from crashing
ros2 service call /crazyflie_real/stop_wall_following std_srvs/srv/Trigger
