#!/usr/bin/env python3
import os
import math
import collections
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"

        self._v = 0.1
        self._omega = 0

        # ToF傳感器和相機節點角度和距離的主題
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self.left_angle_topic = f"/{self._vehicle_name}/camera_node_straight/leftangle" #直線角度
        self.right_angle_topic = f"/{self._vehicle_name}/camera_node_straight/rightangle" #直線角度
        self._offset_topic = f"/{self._vehicle_name}/camera_node_straight/offset"

        self.command_topic = f"/{self._vehicle_name}/wheel_control_node/command"

        # 構造發布者和訂閱者
        #self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.left_angle_subscriber = rospy.Subscriber(self.left_angle_topic, Float32, self.left_angle_callback)
        self.right_angle_subscriber = rospy.Subscriber(self.right_angle_topic, Float32, self.right_angle_callback)
        self.offset_subscriber = rospy.Subscriber(self._offset_topic, Float32, self.offset_callback)

        self.command_subscriber = rospy.Subscriber(self.command_topic, String, self.command_callback)

        # 初始化
        self._offset = 0
        self.left_angle = 0
        self.right_angle = 0

        rospy.on_shutdown(self.on_shutdown)
        # 初始化 PID 控制器
        #self.offset_pid = PIDController(kp=0.06, ki=0, kd=0.01)
        #self.angle_pid = PIDController(kp=0.06, ki=0, kd=0.01)

        # 移动平均窗口
        #self.offset_window = collections.deque(maxlen=10)
        #self.angle_window = collections.deque(maxlen=10)

    
    def command_callback(self, msg):
        command = msg.data
        rospy.loginfo(f"Received command: {command}")
        self.execute_command_sequence(command)

    def execute_command_sequence(self, command_sequence):
        for command in command_sequence:
            rospy.loginfo(f"Executing command: {command}")
            self.execute_command(command)
            #rospy.sleep(1)

    def execute_command(self, command):
        if command == "0":
            self.stop()
        elif command == "1":
            self.forward()
        else:
            rospy.logwarn(f"Unknown command received: {command}")



    def offset_callback(self, msg):
        self._offset = msg.data
        #rospy.loginfo(f"Received offset: {self._offset}")

    def left_angle_callback(self, msg):
        self.left_angle = msg.data
        #self.left_angle_window.append(self.left_angle)
        #rospy.loginfo(f"Received angle: {self._angle}")

    def right_angle_callback(self, msg):
        self.right_angle = msg.data
        #self.right_angle_window.append(self.right_angle)
        #rospy.loginfo(f"Received angle: {self._angle}")


    def forward(self):
        adjustment = self.calculate_combined_adjustment(self._offset, self.left_angle,  self.right_angle)
        message = Twist2DStamped(v = 0.1, omega = adjustment)

        print(f"Twist2D omega: {adjustment}")

        self.publisher.publish(message)
        rospy.loginfo("Moving forward...")


    def stop(self, event=None):
        message = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(message)
        rospy.loginfo("Stop...")

    def calculate_combined_adjustment(self, offset, left_angle, right_angle):
        """根據偏移量和角度計算調整值"""
        A1, B1, C1, D1 = 0.0297, -0.1831, -0.0497, 12.6796
        A2, B2, C2, D2 = 0.1589,  0.7349, -0.7358, 90.5100

        # 控制增益
        Kp_d = 0.0571  # rad/s per cm
        Kp_a = 0.3820  # rad/s per rad

        # 1) 狀態估計
        d_est     = A1*offset + B1*left_angle + C1*right_angle + D1        # cm
        angle_est = A2*offset + B2*left_angle + C2*right_angle + D2        # 絕對角度 (°)

        # 2) 算出「偏航誤差」（度）
        yaw_err_deg = angle_est - 90.0

        # 3) 轉成弧度
        yaw_err = math.radians(yaw_err_deg)  # rad

        # 4) 計算角速度
        omega = - Kp_d * d_est - Kp_a * yaw_err

        print(f"dest     = {d_est:.2f} cm")
        print(f"yaw_err   = {yaw_err_deg:.2f}° → {yaw_err:.4f} rad")
        print(f"omega     = {omega:.4f} rad/s")

        return omega
    
    def calculate_smoothed_value(self, data, alpha=0.25):
        if not data:
            return 0
        ewma = data[0]
        for value in data[1:]:
            ewma = (1 - alpha) * ewma + alpha * value
        return ewma

    def on_shutdown(self):
        self.stop()  # Use the stop method to stop the robot
        rospy.loginfo("Shutting down, stopping the robot")

        # Unregister all subscribers and publishers
        #self.distance_subscriber.unregister()
        #self.straight_status_subscriber.unregister()
         # Unregister all subscribers
        if hasattr(self, 'angle_subscriber'):
            self.angle_subscriber.unregister()
        if hasattr(self, 'offset_subscriber'):
            self.offset_subscriber.unregister()


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        # Calculate error
        error = setpoint - measurement

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error)

        # Compute the output
        output = proportional + integral + derivative

        # Save error for next derivative calculation
        self.previous_error = error

        return output


if __name__ == '__main__':
    try:
        node = WheelControlNode(node_name='wheel_control_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

'''
2025.06.25 
'''