#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String, Float32
import traceback
from collections import deque
import time  # Import time module for precise time tracking

inter_distance = 0
lane_positions = []
tof_distance = 0

#distance_threshold1 = 330
#distance_threshold2 = 320
#tof_threshold = 500  # Adjust based on experiment

def tof_callback(msg):
    global tof_distance
    tof_distance = int(1000 * msg.range)  # Convert meters to mm

def inter_distance_callback(msg):
    global inter_distance
    inter_distance = msg.data
    lane_positions.append(inter_distance)  # Store lane position data

def collect_lane_data(duration=5):
    rospy.loginfo(f"Collecting lane position data for {duration} seconds...")
    start_time = time.time()
    while time.time() - start_time < duration and not rospy.is_shutdown():
        rospy.sleep(0.2)
    rospy.loginfo("Lane data collection complete. Starting movement...")

def wait_for_intersection(threshold=80, timeout=10):
    global inter_distance
    start_time = time.time()
    while time.time() - start_time < timeout and not rospy.is_shutdown():
        if inter_distance and inter_distance < threshold:
            return True
        rospy.sleep(0.1)
    return False

def publish_commands(commands, publisher):
    global inter_distance, tof_distance

    i = 0
    while not rospy.is_shutdown():
        if i >= len(commands):
            rospy.loginfo("✅ All commands executed.")
            break

        command = commands[i]

        if command == '1':
            publisher.publish('1')
            rospy.loginfo("🚗 Forward")
            rospy.sleep(2.0) 
            i += 1

        elif command == '0':
            publisher.publish('0')
            rospy.loginfo("🛑 Stop")
            rospy.sleep(3.0)
            i += 1

        elif command in ['2', '3']:  # 左轉或右轉
            rospy.loginfo(f"⚠️ Command {command}: waiting for intersection...")
            if wait_for_intersection():
                publisher.publish(command)
                rospy.loginfo("🔄 Executing turn")
                rospy.sleep(3.0)
                publisher.publish('1')
                rospy.sleep(2.0)
                i += 1
            else:
                rospy.loginfo("❌ No intersection detected yet")
                rospy.sleep(0.5)

        elif command in ['4', '5']:  # 變道
            publisher.publish(command)
            rospy.loginfo(f"↔️ Lane change: {command}")
            rospy.sleep(3.0)
            publisher.publish('1')
            rospy.sleep(2.0)
            i += 1

        else:
            rospy.logwarn(f"❓ Unknown command: {command}")
            i += 1

def main():
    try:
        rospy.init_node('control_node')
        rospy.loginfo("ROS node initialized")
        vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot06')

        command_topic = f"/{vehicle_name}/wheel_control_node/command"
        rospy.loginfo(f"Command topic: {command_topic}")
        command_publisher = rospy.Publisher(command_topic, String, queue_size=10)

        rospy.Subscriber(f"/{vehicle_name}/camera_node_turn/inter_dist", Float32, inter_distance_callback)
        rospy.Subscriber(f"/{vehicle_name}/front_center_tof_driver_node/range", Float32, tof_callback)

        collect_lane_data(5)

        rospy.loginfo("Getting command sequence")
        command_sequence = rospy.get_param('~command_sequence', '11110')
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
revised 2025.04.04
整合視覺交叉口判斷 + 路徑規劃命令執行
'''
