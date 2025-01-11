#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
import time
import math

# throttle and direction for each wheel
W_LEFT = 0.8 * (2 * math.pi)  # 表示每秒左輪轉 0.8 圈
W_RIGHT = 0.5 * (2 * math.pi)  # 表示每秒右輪轉 0.5 圈

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        wheel_radius_param = f"/{vehicle_name}/kinematics_node/radius"
        wheel_radius = rospy.get_param(wheel_radius_param)
        # form the message
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius
        print(f"INIT VEL LEFT {self._vel_left} INIT VEL RIGHT {self._vel_right}")
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, Twist2DStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)  # 設定頻率為 10 Hz
        message = Twist2DStamped(v=0.1, omega=0) #forward
        start_time = time.time()  # 紀錄開始時間

        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()  # 維持 10 Hz 的頻率
            
            # 檢查是否超過 5 秒
            if time.time() - start_time >= 5:
                print("Execution finished after 5 seconds.")
                break

    def on_shutdown(self):
        stop = Twist2DStamped(v = 0, omega = 0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()