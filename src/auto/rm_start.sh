#!/bin/bash

# Function to clean up background processes
cleanup() {
    echo "Terminating all nodes..."
    # Kill all background jobs
    kill $(jobs -p)
    echo "All nodes have been terminated."
}

# Trap termination signals (e.g., CTRL+C) to clean up background jobs
trap cleanup EXIT

# Source the ROS2 workspace
cd ~/r2_ros
source install/setup.bash

# run this to read motor data
# ros2 run motor_control_package motor_data_reader &

# Start motor_control_subscriber node
ros2 run motor_control_package motor_control_subscriber &

# Start ps4_publisher node
ros2 run ps4 ps4_publisher &

# # Start navigation_node node
ros2 run navigation navigation_node &

# # Start omni_pid node
ros2 run navigation omni_pid &

# Wait for all background jobs to complete
wait
