#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import traceback

def publish_commands(commands, publisher):
    for command in commands:
        rospy.loginfo(f"Publishing command: {command}")
        publisher.publish(command)
        rospy.sleep(1.5)  # Add delay between commands to simulate execution time

def main():
    try:
        rospy.init_node('control_node')
        vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot06')
        command_topic = f"/{vehicle_name}/wheel_control_node/command"
        
        command_publisher = rospy.Publisher(command_topic, String, queue_size=10)
        
        mode = rospy.get_param('~mode', '1')  # Get the mode from ROS parameter, default to '1'

        if mode == "1":
            command_sequence = rospy.get_param('~command_sequence', '0001111110')  # Get command sequence from parameter
            publish_commands(command_sequence, command_publisher)
        elif mode == "2":
            command_sequence = rospy.get_param('~manual_commands', '0001111110')  # Manual commands as a single string
            if command_sequence:
                publish_commands(command_sequence, command_publisher)
            else:
                rospy.logwarn("No manual commands provided.")
        else:
            rospy.logerr("Invalid mode. Please set mode to 1 for Automatic or 2 for Manual.")
    
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
