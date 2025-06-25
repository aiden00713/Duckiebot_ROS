#!/usr/bin/env python3
import os
import math
import collections
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 0.8 * (2 * math.pi)  # 表示每秒左輪轉 0.5 圈
W_RIGHT = 0.5 * (2 * math.pi)  # 表示每秒右輪轉 0.5 圈
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 4.0

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheel_radius_param = f"/{self._vehicle_name}/kinematics_node/radius"
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"

        # 獲取Duckiebot的輪徑
        wheel_radius = rospy.get_param(wheel_radius_param)

        # 計算線速度
        self._vel_left = W_LEFT * wheel_radius
        self._vel_right = W_RIGHT * wheel_radius
        print(f"INIT VEL LEFT {self._vel_left} INIT VEL RIGHT {self._vel_right}")

        self._v = VELOCITY
        self._omega = OMEGA

        # ToF傳感器和相機節點角度和距離的主題
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._angle_topic = f"/{self._vehicle_name}/camera_node_straight/angles" #直線角度

        self._offset_topic = f"/{self._vehicle_name}/camera_node_straight/offset"

        self._straight_status_topic = f"/{self._vehicle_name}/camera_node_turn/straight_status"
        self._inter_distance_topic = f"/{self._vehicle_name}/camera_node_turn/inter_dist" #距離路口虛線的距離
        self._right_roi_distance_topic = f"/{self._vehicle_name}/camera_node_turn/right_dist"
        self._left_roi_distance_topic = f"/{self._vehicle_name}/camera_node_turn/left_dist"

        self.command_topic = f"/{self._vehicle_name}/wheel_control_node/command"

        # 構造發布者和訂閱者
        #self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.offset_subscriber = rospy.Subscriber(self._offset_topic, Float32, self.offset_callback)


        self.command_subscriber = rospy.Subscriber(self.command_topic, String, self.command_callback)

        # 狀態變量
        self.state = "IDLE"
        self.turn_direction = "NONE"
        self._offset = 0
        self._angle = 0
        self.turning = False  # 初始化轉彎標誌變量

        # 初始化 PID 控制器
        self.offset_pid = PIDController(kp=0.06, ki=0, kd=0.01)
        self.angle_pid = PIDController(kp=0.06, ki=0, kd=0.01)

        # 初始化時間變量
        self.last_time = rospy.get_time()

        # 移动平均窗口
        self.offset_window = collections.deque(maxlen=10)
        self.angle_window = collections.deque(maxlen=10)

    
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

    def angle_callback(self, msg):
        self._angle = msg.data
        #rospy.loginfo(f"Received angle: {self._angle}")



    def forward(self):
        #smoothed_offset = self.calculate_smoothed_value(self.offset_window)
        #smoothed_angle = self.calculate_smoothed_value(self.angle_window)
        #print(f"smoothed_offset: {smoothed_offset} , smoothed_angle: {smoothed_angle}")
        
        #adjustment = self.calculate_combined_adjustment(smoothed_offset, smoothed_angle)

        # 使用 PID 控制器計算偏移量和角度的調整值
        #offset_adjustment = self.offset_pid.compute(setpoint=0, measurement=smoothed_offset)
        #angle_adjustment = self.angle_pid.compute(setpoint=0, measurement=smoothed_angle)

        # 結合兩者的調整值來計算最終的 omega
        #adjustment = offset_adjustment + angle_adjustment
        

        #message = WheelsCmdStamped(vel_left = left, vel_right = right)
        message = Twist2DStamped(v = 0.09, omega = adjustment)

        print(f"Twist2D omega: {adjustment}")

        self.publisher.publish(message)
        rospy.loginfo("Moving forward")
        self.turning = False

    def stop(self, event=None):
        message = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(message)
        rospy.loginfo("Stopped the robot")
        self.turning = False



    def calculate_combined_adjustment(self, offset, angle):
        """根據偏移量和角度計算調整值"""
        offset_adjustment = offset * 0.0005  # 偏移量調整係數
        angle_adjustment = angle * 0.008  # 角度調整係數
        combined_adjustment = offset_adjustment + angle_adjustment
        combined_adjustment = max(-0.5, min(0.5, combined_adjustment))  # 限制調整值的範圍
        #rospy.loginfo(f"Calculated combined adjustment: {combined_adjustment} (offset: {offset_adjustment}, angle: {angle_adjustment})")
        return combined_adjustment
    
    def calculate_smoothed_value(self, data, alpha=0.25):
        if not data:
            return 0
        ewma = data[0]
        for value in data[1:]:
            ewma = (1 - alpha) * ewma + alpha * value
        return ewma

    def run(self):
        rospy.spin()

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