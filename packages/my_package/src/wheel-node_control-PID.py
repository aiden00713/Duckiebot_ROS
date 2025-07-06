#!/usr/bin/env python3
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32, String

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"

        # topic 訂閱與發布
        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.left_angle_subscriber = rospy.Subscriber(f"/{self._vehicle_name}/camera_node_straight/leftangle", Float32, self.left_angle_callback)
        self.right_angle_subscriber = rospy.Subscriber(f"/{self._vehicle_name}/camera_node_straight/rightangle", Float32, self.right_angle_callback)
        self.offset_subscriber = rospy.Subscriber(f"/{self._vehicle_name}/camera_node_straight/offset", Float32, self.offset_callback)
        self.command_subscriber = rospy.Subscriber(f"/{self._vehicle_name}/wheel_control_node/command", String, self.command_callback)

        # 初始化控制參數
        self._offset = 0
        self.left_angle = 0
        self.right_angle = 0

        # PID 控制器（單一控制器處理合併誤差）
        self.combined_pid = PIDController(kp=0.4, ki=0.01, kd=0.1) 
        self.omega_max = 0.4  # 最大角速度限制

        # 關閉時停止
        rospy.on_shutdown(self.on_shutdown)

    def offset_callback(self, msg):
        self._offset = msg.data

    def left_angle_callback(self, msg):
        self.left_angle = msg.data

    def right_angle_callback(self, msg):
        self.right_angle = msg.data

    def command_callback(self, msg):
        self.execute_command_sequence(msg.data)

    def execute_command_sequence(self, command_sequence):
        for command in command_sequence:
            self.execute_command(command)

    def execute_command(self, command):
        if command == "0":
            self.stop()
        elif command == "1":
            self.forward()
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def forward(self):
        omega = self.calculate_combined_adjustment(self._offset, self.left_angle, self.right_angle)
        message = Twist2DStamped(v=0.1, omega=omega)
        self.publisher.publish(message)
        rospy.loginfo("Moving forward...")

    def stop(self, event=None):
        message = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(message)
        rospy.loginfo("Stopped.")

    def calculate_combined_adjustment(self, offset, left_angle, right_angle):
        # 線性回歸模型估計 d_est 和 yaw
        A1, B1, C1, D1 = 0.0297, -0.1831, -0.0497, 12.6796
        A2, B2, C2, D2 = 0.1589,  0.7349, -0.7358, 90.5100

        d_est = A1 * offset + B1 * left_angle + C1 * right_angle + D1        # cm
        angle_est = A2 * offset + B2 * left_angle + C2 * right_angle + D2    # deg
        yaw_err_deg = angle_est - 90.0
        yaw_err_rad = math.radians(yaw_err_deg)  # rad

        # 歸一化 + 加權合併誤差
        offset_max = 7           # 最大偏移 (cm)
        yaw_max = math.radians(20.0)  # 最大偏航角 (rad)
        offset_norm = d_est / offset_max
        yaw_norm = yaw_err_rad / yaw_max
        alpha = 1.0
        beta = 1.0
        combined_error = alpha * offset_norm + beta * yaw_norm

        # PID 計算
        omega_raw = self.combined_pid.compute(setpoint=0, measurement=combined_error)
        omega = max(min(omega_raw, self.omega_max), -self.omega_max)
        #omega = omega_raw
        # 除錯用 log
        print(f"[PID控制資訊] offset = {d_est:.2f} cm | yaw_err = {yaw_err_deg:.2f}° | omega = {omega:.4f} rad/s")

        return omega

    def on_shutdown(self):
        self.stop()
        rospy.loginfo("Shutting down, stopping the robot.")

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

if __name__ == '__main__':
    try:
        node = WheelControlNode(node_name='wheel_control_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
