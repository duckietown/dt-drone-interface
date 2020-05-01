#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch --wait drone_interface all.launch veh:=$VEHICLE_NAME \
                              robot_type:=$ROBOT_TYPE

