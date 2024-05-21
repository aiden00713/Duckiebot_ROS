#!/usr/bin/env python3
import os
import sys
import math
import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String, Float32

# angular velocities for each wheel (quarter rotation a second)
W_LEFT = 1.2 * (2 * math.pi) #表示每秒左輪轉1.2圈
W_RIGHT = 1.2* (2 * math.pi) #表示每秒右輪轉1.2圈
Par = (2*math.pi) #完整弧度 = 360度
TURN_SPEED = 0.5  # Adjust this value to control turn sharpness
TURN_DURATION = 50  # Number of iterations to keep turning left

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        global Par
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        wheel_radius_param = f"/{vehicle_name}/kinematics_node/radius"

        # get duckiebot's wheel radius
        wheel_radius = rospy.get_param(wheel_radius_param)

        # compute linear speeds 線性速度
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius
        Par *= wheel_radius
        self.counter = 0  # Initialize counter


        #topic for tof 測量距離
        self._tof = f"/{vehicle_name}/front_center_tof_driver_node/range"

        # Topic for camera node angles
        self._angle_topic = f"/{vehicle_name}/camera_node/angles"

        # Topic for camera node dist
        self._dist_topic = f"/{vehicle_name}/camera_node/dist"

        # construct publisher and subscriber
        self.mul = 0
        self._dis = 0
        self._d_pre = 0
        self.receiver = rospy.Subscriber("Duckie", String, self.call)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance = rospy.Subscriber(self._tof, Range, self.dis)
        self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)


    def angle_callback(self, msg):
        # This method will be called every time a new Float32 message is published on the _angle_topic
        angle = msg.data  # Access the data in the message
        # Here you can add code to process the angle and perhaps adjust wheel commands accordingly
        rospy.loginfo(f"Received angle: {angle}")

    def call(self, data):
        self.mul = int(data.data)


    def dis(self, data):
        self._d_pre = self._dis 
        self._dis = int(1000 * data.range)


    def adjust(self, par):
        l = 0 #左輪初始值
        r = 0 #右輪初始值

        if par >= 100: 
            par = par * 0.32 - 23.6

        if par <= -100:
            par = (par / 10) - 2
        if par >= 9:
            l = par / 40 * Par
        if par <= -9:
            r = -par / 40 * Par
        return l, r


    def turn_left(self):
        left = self._vel_left * TURN_SPEED
        right = self._vel_right

        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)


    def on_shutdown(self):
        if hasattr(self, '_publisher'):
            stop = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(stop)
        print("Shut down completed")


        '''stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        for _ in range(2):
            self._publisher.publish(stop)
        print("SHUT_DOWN")
        sys.exit()'''


    def run(self):
        # publish 1 messages every second.
        rate = rospy.Rate(10)
        pre = 0
        while not rospy.is_shutdown():
            est_mul = pre * 0.1 + self.mul * 0.9
            #if self.angle_subscribe == 70:
            left, right = self.adjust(est_mul)  # compute left,right motor speed
            left = self._vel_left + left
            right = self._vel_right + right

            message = WheelsCmdStamped(vel_left=left, vel_right=right)
            self._publisher.publish(message)  # change motor speed
            pre = self.mul
            rate.sleep()

    

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()