#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import Range

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self.sub = rospy.Subscriber(self._tof, Range, self.callback)
        self._pub = rospy.Publisher("DIS",String,queue_size = 10)

    def callback(self, data):
        rospy.loginfo("Duckiebot heard message: '" + str(int(1000*data.range)) + "'")
    def run(self):


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    node.run()
    # keep spinning
    rospy.spin()
