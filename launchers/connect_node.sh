#!/bin/bash

# Source environment and setup ROS environment
source /environment.sh

# Initialize launch file environment
dt-launchfile-init

# Start the camera node and check if it runs successfully
rosrun my_package camera_node.py &
CAMERA_PID=$!
sleep 3  # Wait for the node to initialize

# Check if the camera node is still running
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo "Camera node failed to start. Exiting."
    exit 1
fi

echo "Camera node started successfully."

# Start the wheel node
rosrun my_package wheel_node.py &
WHEEL_PID=$!
sleep 2  # Wait for the node to initialize

# Ensure all nodes are running
if ! kill -0 $WHEEL_PID 2>/dev/null; then
    echo "Wheel node failed to start. Exiting."
    exit 1
fi

echo "Wheel node started successfully."

# Wait for all apps to end
dt-launchfile-join

