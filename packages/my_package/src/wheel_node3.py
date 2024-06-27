#!/usr/bin/env python3
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 1.2 * (2 * math.pi)  # 表示每秒左輪轉 1.2 圈
W_RIGHT = 1.2 * (2 * math.pi)  # 表示每秒右輪轉 1.2 圈
TURN_SPEED = 0.5  # Adjust this value to control turn sharpness

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative



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

        # Topics for ToF sensor and camera node angles & dist
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._straight_status_topic = f"/{self._vehicle_name}/camera_node/straight_status"

        # Construct publisher and subscriber
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance = rospy.Subscriber(self._tof, Range, self.dis)
        self.straight_status_subscriber = rospy.Subscriber(self._straight_status_topic, String, self.straight_status_callback)
        self.angle_subscriber = rospy.Subscriber(f"/{self._vehicle_name}/camera_node/angle_pub", Float32, self.angle_callback)

        # State variable
        self.state = "STRAIGHT"
        self.turn_direction = "NONE"
        self._dis = 0

        # Initialize PID controller
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.last_time = rospy.get_time()

    def dis(self, data):
        self._d_pre = self._dis
        self._dis = int(1000 * data.range)

    def turn_left(self):
        left = self._vel_left * TURN_SPEED
        right = self._vel_right
        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)

    def turn_right(self):
        left = self._vel_left 
        right = self._vel_right * TURN_SPEED
        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)

    def forward(self):
        left = self._vel_left 
        right = self._vel_right
        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)

    def straight_status_callback(self, msg):
        status, direction = msg.data.split(",")
        self.state = status
        self.turn_direction = direction
        rospy.loginfo(f"Straight Status: {self.state}, Turn Direction: {self.turn_direction}")

        if self.state == "TURN":
            if self.turn_direction == "LEFT":
                self.turn_left()
            elif self.turn_direction == "RIGHT":
                self.turn_right()
        elif self.state == "STRAIGHT":
            self.forward()

    def angle_callback(self, msg):
        angle = msg.data
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Use PID controller to compute adjustment
        adjustment = self.pid.compute(0, angle, dt)
        self.adjust_wheels_based_on_angle(adjustment)

    def adjust_wheels_based_on_angle(self, adjustment):
        # Adjust the wheel speeds based on the PID adjustment
        if adjustment > 0:  # Turn right
            left = self._vel_left
            right = self._vel_right * (1 - abs(adjustment) / 90)
        else:  # Turn left
            left = self._vel_left * (1 - abs(adjustment) / 90)
            right = self._vel_right
        message = WheelsCmdStamped(vel_left=left, vel_right=right)
        self._publisher.publish(message)


    def run(self):
        rospy.spin()

    def on_shutdown(self):
        if hasattr(self, '_publisher'):
            stop = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(stop)
        print("Shut down completed")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()

'''
20240628
1.新增pid控制
2.得到camera_node的角度做修正

'''