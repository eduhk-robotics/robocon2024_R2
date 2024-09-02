#!/bin/bash

# Navigate to the workspace
cd ~/r2_ros
source install/setup.bash

# Launch all the nodes in new terminals
gnome-terminal -- bash -c "ros2 run sensor stp32l; exec bash"
gnome-terminal -- bash -c "ros2 run sensor gyrosensor; exec bash"
gnome-terminal -- bash -c "ros2 run sensor sensor_publisher; exec bash"

echo "All node have been started"
