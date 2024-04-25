#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        
        self.pub = rospy.Publisher('Duckie', String, queue_size=10)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        timer = 0 #numbers of 'a' message
        while not rospy.is_shutdown():
            message = "a"
            timer+=1
            if timer ==12:
                timer = 0 # recount
            if timer >= 10: #after sending 'a' for 5 times
                message = "c"#send 'c'
            rospy.loginfo("Duckiebot published message: '" + message+ "'")

            self.pub.publish(message)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
