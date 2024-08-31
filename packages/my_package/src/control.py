#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String, Float32
import traceback
from collections import deque

inter_distance = 0
distance_threshold1 = 330
distance_threshold2 = 320

#距離路口虛線距離來判斷何時轉彎
def inter_distance_callback(msg):
    global inter_distance
    inter_distance = msg.data
    print(f"inter distance: {inter_distance}")


def publish_commands(commands, publisher):
    global inter_distance, distance_threshold1, distance_threshold2
    
    threshold_window = deque(maxlen=5)
    
    i = 0
    while i < len(commands):
        command = commands[i]

        if command == '1':
            rospy.loginfo(f"Publishing command: {command} in loop")
            publisher.publish(command)
            rospy.sleep(1.0)  # Short sleep between each loop iteration

            # Start a loop with rospy.sleep(10) and allow it to break out if conditions are met
            for _ in range(40):
                rospy.sleep(0.2)  # Sleep in smaller increments to check conditions more frequently

                if inter_distance is not None:
                    # Add the current distance to the moving average window
                    threshold_window.append(inter_distance)

                    # Count how many readings in the window exceed the threshold
                    count_warning = sum(1 for distance in threshold_window if distance >= distance_threshold1)

                    # Count how many readings exceed the final threshold
                    count_final = sum(1 for distance in threshold_window if distance >= distance_threshold2)

                    # Check if the moving average exceeds the threshold
                    if i + 1 < len(commands) and commands[i + 1] in ['2', '3']:
                        if count_warning >= 3 and count_final >= 3:
                            rospy.loginfo(f"Breaking loop as command {commands[i+1]} follows '1' and moving average exceeds threshold")
                            break

            i += 1
            continue  # Continue to the next command if the inner loop was not broken

        elif command in ['0', '2', '3']:
            rospy.loginfo(f"Publishing command: {command} once")
            publisher.publish(command)
            rospy.sleep(3.0)

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

        rospy.loginfo("Getting command sequence")
        command_sequence = rospy.get_param('~command_sequence', '01310')  # Get command sequence from parameter
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
