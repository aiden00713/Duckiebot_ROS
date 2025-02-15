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
    global inter_distance, distance_threshold1, distance_threshold2, tof_distance

    i = 0
    
    # 🚗 大 while 迴圈讓車輛持續直行
    while not rospy.is_shutdown():
        rospy.loginfo("🚗 Keeping vehicle moving forward")
        publisher.publish('1')  # 讓車輛持續直行
        rospy.sleep(0.5)  # 讓直行指令頻繁發送，避免丟失

        # 🏁 檢查是否需要執行其他命令
        while i < len(commands):  # 內部 while 負責處理命令輸入
            command = commands[i]

            if command == '0':  # 🛑 完全停止
                rospy.loginfo("🛑 Stop command received, stopping the vehicle")
                publisher.publish('0')  # 發送停止指令
                rospy.sleep(3.0)  # 確保停止後才繼續
                rospy.loginfo("✅ Vehicle stopped, waiting for next command")
                i += 1  # 移動到下一個指令
                break  # 結束內部 while，等待新的命令

            elif command in ['2', '3', '4', '5']:  # 🚥 轉向或變道
                if command in ['2', '3']:  # 轉彎
                    rospy.loginfo(f"Executing turn command: {command}")
                    publisher.publish(command)
                    rospy.sleep(5.0)  # 等待轉彎完成
                    rospy.loginfo(f"Returning to straight after turn")

                elif command in ['4', '5']:  # 變道
                    rospy.loginfo(f"Executing lane change command: {command}")
                    publisher.publish(command)
                    rospy.sleep(3.0)  # 等待變道完成

                # 轉向或變道後，恢復直行
                rospy.loginfo("✅ Resuming forward movement")
                publisher.publish('1')  
                rospy.sleep(2.0)  # 讓直行至少保持 2 秒再檢查下一個指令

                i += 1  # 移動到下一個指令
                break  # 結束內部 while，回到外部 while 讓車輛繼續直行


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