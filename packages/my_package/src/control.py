#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String, Float32
import traceback
from collections import deque
import time  # Import time module for precise time tracking

inter_distance = 0
distance_threshold1 = 330
distance_threshold2 = 320

tof_distance = 0
tof_threshold = 500  # Adjust based on experiment

lane_positions = []  # Store collected lane position data

def tof_callback(msg):
    global tof_distance
    tof_distance = int(1000 * msg.range)  # Convert meters to mm

# Collect lane position data
def inter_distance_callback(msg):
    global inter_distance
    inter_distance = msg.data
    lane_positions.append(inter_distance)  # Store lane position data

def collect_lane_data(duration=20):
    """ Collect lane position data for a given duration (default: 20 seconds). """
    rospy.loginfo(f"Collecting lane position data for {duration} seconds...")
    start_time = time.time()
    
    while time.time() - start_time < duration and not rospy.is_shutdown():
        rospy.sleep(0.2)  # Sleep in small increments to keep data collection smooth
    
    rospy.loginfo("Lane data collection complete. Starting movement...")

def publish_commands(commands, publisher):
    global inter_distance, distance_threshold1, distance_threshold2

    threshold_window = deque(maxlen=5)
    i = 0

    while i < len(commands):
        command = commands[i]

        # 🚗 Default: Always move forward
        if command == '1':
            rospy.loginfo(f"Publishing command: {command} in loop")
            publisher.publish(command)
            rospy.sleep(1.0)  

            # Check for lane changes using ToF
            for _ in range(40):
                rospy.sleep(0.2)

                if tof_distance < tof_threshold:  # 🚧 Obstacle detected
                    rospy.loginfo("Obstacle detected by ToF, switching lane to the LEFT")
                    publisher.publish('4')  # Force Left Lane Change
                    rospy.sleep(2.0)  
                    publisher.publish('1')  # 🔄 Return to moving forward
                    break  

                if inter_distance is not None:
                    threshold_window.append(inter_distance)
                    count_warning = sum(1 for d in threshold_window if d >= distance_threshold1)
                    count_final = sum(1 for d in threshold_window if d >= distance_threshold2)

                    if i + 1 < len(commands) and commands[i + 1] in ['2', '3']:  # Turn Left or Right
                        if count_warning >= 3 and count_final >= 3:
                            rospy.loginfo(f"Breaking loop as command {commands[i+1]} follows '1' and moving average exceeds threshold")
                            break  

            i += 1
            continue  

        # 🏁 Turn Left (`2`), Right (`3`), Big Left (`6`), Big Right (`7`)
        elif command in ['2', '3', '6', '7']:
            rospy.loginfo(f"Executing turn command: {command}")
            publisher.publish(command)
            rospy.sleep(5.0)  # Reduce stop delay for efficiency
            rospy.loginfo(f"Returning to straight after turn")
            publisher.publish('1')  # 🔄 Always return to straight after turn

        # Stop, Lane Change Left (`4`), Right (`5`)
        elif command in ['0', '4', '5']:
            rospy.loginfo(f"Publishing command: {command} once")
            publisher.publish(command)
            rospy.sleep(5.0)  
            
        else:
            rospy.logwarn(f"Unknown command: {command}")

        i += 1

def main():
    try:
        rospy.init_node('control_node')
        rospy.loginfo("ROS node initialized")
        vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot06')
        global distance_threshold

        command_topic = f"/{vehicle_name}/wheel_control_node/command"
        rospy.loginfo(f"Command topic: {command_topic}")
        command_publisher = rospy.Publisher(command_topic, String, queue_size=10)
        
        rospy.Subscriber(f"/{vehicle_name}/camera_node_turn/inter_dist", Float32, inter_distance_callback)  # Replace with actual topic name
        rospy.Subscriber(f"/{vehicle_name}/front_center_tof_driver_node/range", Float32, tof_callback)

        # 🛠️ **Collect lane position data before starting movement**
        collect_lane_data(20)

        rospy.loginfo("Getting command sequence")
        command_sequence = rospy.get_param('~command_sequence', '013353350')  # Get command sequence from parameter
        publish_commands(command_sequence, command_publisher)

    except Exception as e:
        rospy.logerr(f"Control node failed with exception: {e}")
        rospy.logerr(traceback.format_exc())
        exit(1)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Unexpected error in the main function: {e}")
        rospy.logerr(traceback.format_exc())
        exit(1)

'''
2025.02.16 
'''