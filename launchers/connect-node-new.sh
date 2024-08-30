#!/bin/bash

# Source environment and setup ROS environment
source /environment.sh

# Initialize launch file environment
dt-launchfile-init

# Function to start a ROS node and check if it runs successfully
start_ros_node() {
    local node_name=$1
    local node_script=$2

    echo "Starting $node_name..."
    rosrun my_package $node_script &
    local NODE_PID=$!
    sleep 3  # Wait for the node to initialize

    # Check if the node is still running
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "$node_name failed to start. Exiting."
        exit 1
    fi

    echo "$node_name started successfully."
    return $NODE_PID
}

# Start the straight camera node
start_ros_node "Straight camera node" "camera-node_straight.py"
CAMERA_STRAIGHT_PID=$?

# Start the turn camera node
start_ros_node "Turn camera node" "camera-node_turn.py"
CAMERA_TURN_PID=$?

# Start the wheel node
start_ros_node "Wheel node" "wheel_node4.py"
WHEEL_PID=$?

# Start the control node
start_ros_node "Control node" "control.py"
CONTROL_PID=$?

# Wait for all nodes to end
dt-launchfile-join