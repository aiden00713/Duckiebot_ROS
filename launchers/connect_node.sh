#!/bin/bash

# Source environment and setup ROS environment
source /environment.sh

# Initialize launch file environment
dt-launchfile-init

# Start the straight camera node and check if it runs successfully
rosrun my_package camera-node_straight.py &
CAMERA_STRAIGHT_PID=$!
sleep 3  # Wait for the node to initialize

# Check if the straight camera node is still running
if ! kill -0 $CAMERA_STRAIGHT_PID 2>/dev/null; then
    echo "Straight camera node failed to start. Exiting."
    exit 1
fi

echo "Straight camera node started successfully."

# Start the turn camera node and check if it runs successfully
rosrun my_package camera-node_turn.py &
CAMERA_TURN_PID=$!
sleep 3  # Wait for the node to initialize

# Check if the turn camera node is still running
if ! kill -0 $CAMERA_TURN_PID 2>/dev/null; then
    echo "Turn camera node failed to start. Exiting."
    exit 1
fi

echo "Turn camera node started successfully."

# Start the wheel node
rosrun my_package wheel_node4.py &
WHEEL_PID=$!
sleep 2  # Wait for the node to initialize

# Ensure all nodes are running
if ! kill -0 $WHEEL_PID 2>/dev/null; then
    echo "Wheel node failed to start. Exiting."
    exit 1
fi

echo "Wheel node started successfully."

rosrun my_package control.py &
CONTROL_PID=$!
sleep 2  # Wait for the node to initialize

# Ensure all nodes are running
if ! kill -0 $CONTROL_PID 2>/dev/null; then
    echo "Control node failed to start. Exiting."
    exit 1
fi

echo "Control node started successfully."


# Wait for all apps to end
dt-launchfile-join

