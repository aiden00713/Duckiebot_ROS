#!/usr/bin/env python3
import os
import math
import rospy
import cv2
import numpy as np
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 1.2 * (2 * math.pi)  # 表示每秒左轮转 1.2 圈
W_RIGHT = 1.2 * (2 * math.pi)  # 表示每秒右轮转 1.2 圈
TURN_SPEED = 0.5  # Adjust this value to control turn sharpness
TURN_DURATION = 50  # Number of iterations to keep turning left

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        wheel_radius_param = f"/{self._vehicle_name}/kinematics_node/radius"

        # Get Duckiebot's wheel radius
        wheel_radius = rospy.get_param(wheel_radius_param)

        # Compute linear speeds
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius
        self.counter = 0  # Initialize counter

        # Topics for ToF sensor and camera node angles & dist
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._angle_topic = f"/{self._vehicle_name}/camera_node/angles"
        self._right_dist_topic = f"/{self.vehicle_name}/camera_node/right_dist"
        self._left_dist_topic = f"/{self.vehicle_name}/camera_node/left_dist"

        # Construct publisher and subscriber
        self.mul = 0
        self._dis = 0
        self._d_pre = 0
        self.receiver = rospy.Subscriber("Duckie", String, self.call)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance = rospy.Subscriber(self._tof, Range, self.dis)
        #self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.right_dist_subscriber = rospy.Subscriber(self._right_dist_topic, Float32, self.right_dist_callback)
        self.left_dist_subscriber = rospy.Subscriber(self._left_dist_topic, Float32, self.left_dist_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.on_shutdown)
        # 是否正在轉彎
        self.is_turning = False
    
    def on_shutdown(self):
        # This function is called when you press Ctrl+C
        rospy.loginfo("Shutting down: Stopping the motors")
        stop = WheelsCmdStamped()
        stop.vel_left = 0.0
        stop.vel_right = 0.0
        self._publisher.publish(stop)
        print("Shut down completed")

    def call(self, data):
        self.mul = int(data.data)

    def dis(self, data):
        self._d_pre = self._dis
        self._dis = int(1000 * data.range)

    def adjust(self, par):
        l = 0  # 左轮初始值
        r = 0  # 右轮初始值

        if par >= 100:
            par = par * 0.32 - 23.6

        if par <= -100:
            par = (par / 10) - 2
        if par >= 9:
            l = par / 40 * self._vel_left
        if par <= -9:
            r = -par / 40 * self._vel_right
        return l, r

    def turn_left(self):
        if not self.is_turning:
            left = self._vel_left * TURN_SPEED
            right = self._vel_right
            #message = WheelsCmdStamped(vel_left=left, vel_right=right)
            message = WheelsCmdStamped(vel_left=0, vel_right=0.5)
            self._publisher.publish(message)
            self.is_turning = True

    def turn_right(self):
        if not self.is_turning:
            rospy.loginfo("Turning left")
            left = self._vel_left 
            right = self._vel_right * TURN_SPEED
            #essage = WheelsCmdStamped(vel_left=left, vel_right=right)
            message = WheelsCmdStamped(vel_left=0.5, vel_right=0)
            self._publisher.publish(message)
            self.is_turning = True

    def forward(self):
        left = self._vel_left 
        right = self._vel_right
        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)
        self.is_turning = False

    def angle_callback(self, msg):
        angle = msg.data
        rospy.loginfo(f"Received angle: {angle}")
        if abs(angle) > 70:  # Assuming 70 degrees indicates a sharp turn
            for _ in range(TURN_DURATION):
                self.turn_left()
    
    # under 340 turn right
    def right_dist_callback(self, msg):
        dist = msg.data
        rospy.loginfo(f"Received RIGHT inter. dist: {dist}")
        if dist < 340:  
            for _ in range(TURN_DURATION):
                self.turn_right()
        else:
            for _ in range(TURN_DURATION):
                self.forward()
    
    
    def left_dist_callback(self, msg):
        dist = msg.data
        rospy.loginfo(f"Received LEFT inter. dist: {dist}")
        if dist < 340:  
            for _ in range(TURN_DURATION):
                self.turn_left()
        else:
            for _ in range(TURN_DURATION):
                self.forward()

    
    def run(self):
        rate = rospy.Rate(10)
        pre = 0
        while not rospy.is_shutdown():
            est_mul = pre * 0.1 + self.mul * 0.9
            left, right = self.adjust(est_mul)
            left = self._vel_left + left
            right = self._vel_right + right

            message = WheelsCmdStamped(vel_left=left, vel_right=right)
            self._publisher.publish(message)
            pre = self.mul
            rate.sleep()

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()
