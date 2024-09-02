#!/bin/bash

# Navigate to the workspace
cd ~/r2_ros
source install/setup.bash

# Launch all the nodes in new terminals
gnome-terminal -- bash -c "ros2 run motor_control_package motor_control_subscriber; exec bash"
gnome-terminal -- bash -c "ros2 run r2_auto omni_pid; exec bash"
gnome-terminal -- bash -c "ros2 run r2_auto r2_main_node; exec bash"

echo "All node have been started"
