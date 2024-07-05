#!/usr/bin/env python3
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 0.5 * (2 * math.pi)  # 表示每秒左輪轉 0.5 圈
W_RIGHT = 0.5 * (2 * math.pi)  # 表示每秒右輪轉 0.5 圍
TURN_SPEED = 0.3  # 調整此值以控制轉彎的靈敏度
MAX_SPEED = 0.4  # 最大速度限制
MIN_SPEED = 0.1  # 最小速度限制

TURN_DISTANCE_THRESHOLD = 240  # 轉彎距離閾值

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        wheel_radius_param = f"/{self._vehicle_name}/kinematics_node/radius"

        # 獲取Duckiebot的輪徑
        wheel_radius = rospy.get_param(wheel_radius_param)

        # 計算線速度
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius

        # ToF傳感器和相機節點角度和距離的主題
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._straight_status_topic = f"/{self._vehicle_name}/camera_node/straight_status"
        self._angle_topic = f"/{self._vehicle_name}/camera_node/angles"
        self._offset_topic = f"/{self._vehicle_name}/camera_node/offset"
        self._right_roi_distance_topic = f"/{self._vehicle_name}/camera_node/right_dist"
        self._left_roi_distance_topic = f"/{self._vehicle_name}/camera_node/left_dist"

        # 構造發布者和訂閱者
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance_subscriber = rospy.Subscriber(self._tof, Range, self.dis_callback)
        self.straight_status_subscriber = rospy.Subscriber(self._straight_status_topic, String, self.straight_status_callback)
        self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.offset_subscriber = rospy.Subscriber(self._offset_topic, Float32, self.offset_callback)
        self.right_roi_distance_subscriber = rospy.Subscriber(self._right_roi_distance_topic, Float32, self.right_roi_distance_callback)

        # 狀態變量
        self.state = "STRAIGHT"
        self.turn_direction = "NONE"
        self._dis = 0
        self._right_roi_distance = 9999  # 初始值設置為較大值
        self._offset = 0
        self._angle = 0

        # 初始化時間變量
        self.last_time = rospy.get_time()

    def dis_callback(self, data):
        self._dis = int(1000 * data.range)

    def right_roi_distance_callback(self, msg):
        self._right_roi_distance = msg.data

    def offset_callback(self, msg):
        self._offset = msg.data

    def angle_callback(self, msg):
        self._angle = msg.data
        self.adjust_wheels_based_on_offset_and_angle()

    def limit_speed(self, speed):
        """將速度限制在設置的範圍內。"""
        return max(MIN_SPEED, min(MAX_SPEED, speed))

    def turn_left(self):
        rospy.loginfo("Turning left")
        self.publish_wheel_cmd(0, TURN_SPEED)

    def turn_right(self):
        rospy.loginfo("Turning right")
        self.publish_wheel_cmd(TURN_SPEED, 0)

    def forward(self):
        rospy.loginfo("Moving forward")
        left = self.limit_speed(self._vel_left)
        right = self.limit_speed(self._vel_right)
        message = WheelsCmdStamped()
        message.vel_left = left
        message.vel_right = right
        self._publisher.publish(message)

    def publish_wheel_cmd(self, left, right):
        message = WheelsCmdStamped()
        message.vel_left = self.limit_speed(left)
        message.vel_right = self.limit_speed(right)
        self._publisher.publish(message)

    def straight_status_callback(self, msg):
        status, direction = msg.data.split(",")
        self.state = status
        self.turn_direction = direction
        rospy.loginfo(f"Straight Status: {self.state}, Turn Direction: {self.turn_direction}")

        if self.state == "TURN":
            if self._right_roi_distance < TURN_DISTANCE_THRESHOLD:
                if self.turn_direction == "LEFT":
                    self.turn_left()
                elif self.turn_direction == "RIGHT":
                    self.turn_right()
            else:
                self.forward()
        elif self.state == "STRAIGHT":
            self.forward()

    def adjust_wheels_based_on_offset_and_angle(self):
        # 根據偏移量和角度計算調整值
        adjustment = self.calculate_combined_adjustment(self._offset, self._angle)

        # 根據調整值調整輪速
        if adjustment > 0:  # Turn right
            left = self._vel_left * (1 + abs(adjustment))
            right = self._vel_right * (1 - abs(adjustment))
        else:  # Turn left
            left = self._vel_left * (1 - abs(adjustment))
            right = self._vel_right * (1 + abs(adjustment))
        
        left = self.limit_speed(left)
        right = self.limit_speed(right)
        
        rospy.loginfo(f"Adjusting wheels based on offset and angle adjustment: left={left}, right={right}")
        self.publish_wheel_cmd(left, right)

    def calculate_combined_adjustment(self, offset, angle):
        """根據偏移量和角度計算調整值"""
        offset_adjustment = offset * 0.01  # 偏移量調整係數
        angle_adjustment = angle * 0.01  # 角度調整係數
        return offset_adjustment + angle_adjustment

    def run(self):
        rospy.spin()

    def on_shutdown(self):
        if hasattr(self, '_publisher'):
            stop = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(stop)
        rospy.loginfo("Shutting down, stopping the robot")

        # Unregister all subscribers and publishers
        self.distance_subscriber.unregister()
        self.straight_status_subscriber.unregister()
        self.angle_subscriber.unregister()
        self.right_roi_distance_subscriber.unregister()
        self.offset_subscriber.unregister()
        self._publisher.unregister()

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()

'''
20240706 不使用PID控制 僅使用角度和偏移量控制馬達
'''