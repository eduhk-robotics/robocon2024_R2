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
colcon build --packages-select n motor_control_package
# Run this to read motor data
ros2 run motor_control_package motor_data_reader &

# Start motor_control_subscriber node
ros2 run motor_control_package motor_control_subscriber &

# Start ps4_publisher node
ros2 run ps4 ps4_publisher &

# Run this to control motor in angular motion
ros2 run motor_control_package angle_rotation &

# Start navigation_node node
ros2 run navigation navigation_node &

# Start omni_pid node
#ros2 run navigation omni_pid &

# ros2 run r2_auto r2_ps4_node &

# Start gyrosensor node
#ros2 run sensor gyrosensor &

# Start sensor_publisher node
#ros2 run sensor sensor_publisher &

# Start stp32l node
#ros2 run sensor stp32l &

# Start auto_move node
#ros2 run navigation auto_move &

# Wait for all background jobs to complete
wait
