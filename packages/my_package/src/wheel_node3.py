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

        # 獲取Duckiebot的輪徑
        wheel_radius = rospy.get_param(wheel_radius_param)

        # 計算線速度
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius

        # ToF傳感器和相機節點角度和距離的主題
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._straight_status_topic = f"/{self._vehicle_name}/camera_node/straight_status"
        self._angle_topic = f"/{self._vehicle_name}/camera_node/angle_pub"
        self._right_roi_distance_topic = f"/{self._vehicle_name}/camera_node/right_dist"

        # 構造發布者和訂閱者
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance_subscriber = rospy.Subscriber(self._tof, Range, self.dis)
        self.straight_status_subscriber = rospy.Subscriber(self._straight_status_topic, String, self.straight_status_callback)
        self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.right_roi_distance_subscriber = rospy.Subscriber(self._right_roi_distance_topic, Float32, self.right_roi_distance_callback)

        # 狀態變量
        self.state = "STRAIGHT"
        self.turn_direction = "NONE"
        self._dis = 0
        self._right_roi_distance = 9999  # 初始值設置為較大值

        # 初始化PID控制器
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.last_time = rospy.get_time()

    def dis(self, data):
        self._dis = int(1000 * data.range)

    def right_roi_distance_callback(self, msg):
        self._right_roi_distance = msg.data

    def limit_speed(self, speed):
        """將速度限制在設置的範圍內。"""
        return max(MIN_SPEED, min(MAX_SPEED, speed))

    def turn_left(self):
        rospy.loginfo("Turning left")
        left = self.limit_speed(0)
        right = self.limit_speed(0.3)
        message = WheelsCmdStamped()
        message.vel_left = left
        message.vel_right = right
        self._publisher.publish(message)

    def turn_right(self):
        rospy.loginfo("Turning right")
        left = self.limit_speed(0.3)
        right = self.limit_speed(0)
        message = WheelsCmdStamped()
        message.vel_left = left
        message.vel_right = right
        self._publisher.publish(message)

    def forward(self):
        rospy.loginfo("Moving forward")
        left = self.limit_speed(self._vel_left)
        right = self.limit_speed(self._vel_right)
        message = WheelsCmdStamped()
        message.vel_left = left
        message.vel_right = right
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

    def angle_callback(self, msg):
        angle = msg.data
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 使用PID控制器計算調整
        adjustment = self.pid.compute(0, angle, dt)
        self.adjust_wheels_based_on_angle(adjustment)

    def adjust_wheels_based_on_angle(self, adjustment):
        # 根據PID調整值調整輪速
        if adjustment > 0:  # Turn right
            left = self.limit_speed(self._vel_left * (1 + abs(adjustment) / 90))
            right = self.limit_speed(self._vel_right * (1 - abs(adjustment) / 90))
            rospy.loginfo(f"Adjusting wheels to turn right: left={left}, right={right}")
        else:  # Turn left
            left = self.limit_speed(self._vel_left * (1 - abs(adjustment) / 90))
            right = self.limit_speed(self._vel_right * (1 + abs(adjustment) / 90))
            rospy.loginfo(f"Adjusting wheels to turn left: left={left}, right={right}")
        message = WheelsCmdStamped()
        message.vel_left = left
        message.vel_right = right
        self._publisher.publish(message)

    def run(self):
        rospy.spin()

    def on_shutdown(self):
        if hasattr(self, '_publisher'):
            stop = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(stop)
        rospy.loginfo("Shut down completed")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
