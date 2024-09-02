#!/bin/bash

# Source the ROS2 workspace
cd ~/r2_ros
source install/setup.bash

# Start motor_data_reader node
# gnome-terminal -- bash -c "ros2 run motor_control_package motor_data_reader; exec bash"

# Start motor_control_subscriber node
gnome-terminal -- bash -c "ros2 run motor_control_package motor_control_subscriber; exec bash"

# Start ps4_publisher node
gnome-terminal -- bash -c "ros2 run ps4 ps4_publisher; exec bash"

# Start navigation_node node
gnome-terminal -- bash -c "ros2 run navigation navigation_node; exec bash"

# Start omni_pid node
gnome-terminal -- bash -c "ros2 run navigation omni_pid; exec bash"

# Start button control node
gnome-terminal -- bash -c "ros2 run r2_auto r2_ps4_node; exec bash"

echo "All nodes have been started."
