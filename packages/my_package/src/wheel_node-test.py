#!/usr/bin/env python3
import os
import math
import collections
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 0.5 * (2 * math.pi)  # 表示每秒左輪轉 0.5 圈
W_RIGHT = 0.5 * (2 * math.pi)  # 表示每秒右輪轉 0.5 圈
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 0
TURN_DISTANCE_THRESHOLD = 350  # 轉彎距離閾值


class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
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
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range" #ToF
        self._imu = f"/{self._vehicle_name}/imu_node/data" #IMU
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
        self.distance_subscriber = rospy.Subscriber(self._tof, Range, self.dis_callback)
        #self.straight_status_subscriber = rospy.Subscriber(self._straight_status_topic, String, self.straight_status_callback)
        self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.offset_subscriber = rospy.Subscriber(self._offset_topic, Float32, self.offset_callback)
        self.inter_distance_subscriber = rospy.Subscriber(self._inter_distance_topic, Float32, self.inter_distance_callback)
        self.right_roi_distance_subscriber = rospy.Subscriber(self._right_roi_distance_topic, Float32, self.right_roi_distance_callback)

        self.command_subscriber = rospy.Subscriber(self.command_topic, String, self.command_callback)

        # 狀態變量
        self.state = "IDLE"
        self.turn_direction = "NONE"
        self._dis = 0
        self._right_roi_distance = 9999  # 初始值設置為較大值
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
        self.imu_window = collections.deque(maxlen=10)

        self.control_mode = 0
        self.running = False
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.forward)


    def dis_callback(self, data):
        self._dis = int(1000 * data.range)
    
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
            self.running = False
            self.stop()
        elif command == "1":
            self.running = True
            rospy.Timer(rospy.Duration(3.0), self.stop, oneshot=True)
            rospy.loginfo("🚗 Starting forward for 3 seconds")
            self.forward()
        elif command == "2":
            self.turn_left()
        elif command == "3":
            self.turn_right()
        elif command == "4":
            self.change_lanes_left()
        elif command == "5":
            self.change_lanes_right()
        elif command == "6":
            self.big_turn_left()
        elif command == "7":
            self.big_turn_right()
        else:
            rospy.logwarn(f"Unknown command received: {command}")


    def inter_distance_callback(self, msg):
        self._inter_distance = msg.data

    def right_roi_distance_callback(self, msg):
        self._right_roi_distance = msg.data

    def offset_callback(self, msg):
        self._offset = msg.data
        #rospy.loginfo(f"Received offset: {self._offset}")

    def angle_callback(self, msg):
        self._angle = msg.data
        #rospy.loginfo(f"Received angle: {self._angle}")

    def limit_speed(self, speed):
        """將速度限制在設置的範圍內。"""
        limited_speed = max(0.4, min(-0.4, speed))
        return round(limited_speed, 1)
        #return math.floor(limited_speed*10) / 10

    def turn_left(self):
        #left = 0.1 * par + (t * par) / (div + 3)
        #right = 2.5 * par + (t * par) / div
        #self.publish_wheel_cmd(0, TURN_SPEED)
        #self.publish_wheel_cmd(vel_left=left, vel_right=right)
        message = Twist2DStamped(v = 0.19, omega = 4.0)
        self.publisher.publish(message)
        self.turning = True  # 開始轉彎，設置轉彎標誌
        rospy.Timer(rospy.Duration(4), self.stop, oneshot=True)
        rospy.loginfo("Turning left")

    def turn_right(self):
        
        #left = 2.5 * par + (t * par) / div  # 计算左轮速度
        #right = 0.1 * par + (t * par) / (div + 3)  # 计算右轮速度
        #self.publish_wheel_cmd(vel_left=left, vel_right=right)
        message = Twist2DStamped(v = 0.19, omega = -4.0)
        self.publisher.publish(message)
        self.turning = True  # 開始轉彎，設置轉彎標誌
        rospy.Timer(rospy.Duration(4), self.stop, oneshot=True)
        rospy.loginfo("Turning right")

    def change_lanes_left(self):
        # 先向左
        message = Twist2DStamped(v = 0.19, omega = 1.5)
        self.publisher.publish(message)
        rospy.sleep(1.4)
        # 再向右回正
        message = Twist2DStamped(v = 0.19, omega = -1.5)
        self.publisher.publish(message)

    def change_lanes_right(self):
        # 先向右
        message = Twist2DStamped(v = 0.19, omega = -1.5)
        self.publisher.publish(message)
        rospy.sleep(1.5)
        # 再向左回正
        message = Twist2DStamped(v = 0.2, omega = 1.5)
        self.publisher.publish(message)

    def big_turn_left(self):
        #left = 0.1 * par + (t * par) / (div + 3)
        #right = 2.5 * par + (t * par) / div
        #self.publish_wheel_cmd(0, TURN_SPEED)
        #self.publish_wheel_cmd(vel_left=left, vel_right=right)
        message = Twist2DStamped(v = 0.19, omega = 2.0)
        self.publisher.publish(message)
        self.turning = True  # 開始轉彎，設置轉彎標誌
        rospy.Timer(rospy.Duration(4), self.stop, oneshot=True)
        rospy.loginfo("Turning big left")

    def big_turn_right(self):
        
        #left = 2.5 * par + (t * par) / div  # 计算左轮速度
        #right = 0.1 * par + (t * par) / (div + 3)  # 计算右轮速度
        #self.publish_wheel_cmd(vel_left=left, vel_right=right)
        message = Twist2DStamped(v = 0.19, omega = -2.0)
        self.publisher.publish(message)
        self.turning = True  # 開始轉彎，設置轉彎標誌
        rospy.Timer(rospy.Duration(4), self.stop, oneshot=True)
        rospy.loginfo("Turning big right")

    '''
    def forward(self):
        smoothed_offset = self.calculate_smoothed_value(self.offset_window)
        smoothed_angle = self.calculate_smoothed_value(self.angle_window)
        print(f"smoothed_offset: {smoothed_offset} , smoothed_angle: {smoothed_angle}")
        
        #adjustment = self.calculate_combined_adjustment(smoothed_offset, smoothed_angle)

        # 使用 PID 控制器計算偏移量和角度的調整值
        offset_adjustment = self.offset_pid.compute(setpoint=0, measurement=smoothed_offset)
        angle_adjustment = self.angle_pid.compute(setpoint=0, measurement=smoothed_angle)

        # 結合兩者的調整值來計算最終的 omega
        adjustment = offset_adjustment + angle_adjustment


        #message = WheelsCmdStamped(vel_left = left, vel_right = right)
        message = Twist2DStamped(v = 0.09, omega = adjustment)

        print(f"Twist2D omega: {adjustment}")

        self.publisher.publish(message)
        rospy.loginfo("Moving forward")
        self.turning = False
    '''

    def forward(self, enent=None):
        if not self.running:
            return
        
        smoothed_offset = self.calculate_smoothed_value(self.offset_window)
        smoothed_angle = self.calculate_smoothed_value(self.angle_window)

        if self.control_mode == 0:
            # 無 EWMA、無 PID（用 raw data 計算）
            adjustment = self.calculate_combined_adjustment(self._offset, self._angle)
            print(f"{self.control_mode},{rospy.Time.now().to_sec():.2f},{self._offset:.3f},{self._angle:.3f},{adjustment:.3f}")

        elif self.control_mode == 1:
            # 只有 PID（直接用 raw data 丟 PID）
            offset_adjustment = self.offset_pid.compute(setpoint=0, measurement=self._offset)
            angle_adjustment = self.angle_pid.compute(setpoint=0, measurement=self._angle)
            adjustment = offset_adjustment + angle_adjustment

            print(f"{self.control_mode},{rospy.Time.now().to_sec():.2f},{offset_adjustment:.3f},{angle_adjustment:.3f},{adjustment:.3f}")


        elif self.control_mode == 2:
            # 只有 EWMA（不使用 PID）
            adjustment = self.calculate_combined_adjustment(smoothed_offset, smoothed_angle)

            print(f"{self.control_mode},{rospy.Time.now().to_sec():.2f},{smoothed_offset:.3f},{smoothed_angle:.3f},{adjustment:.3f}")


        elif self.control_mode == 3:
            # PID + EWMA（預設）
            offset_adjustment = self.offset_pid.compute(setpoint=0, measurement=smoothed_offset)
            angle_adjustment = self.angle_pid.compute(setpoint=0, measurement=smoothed_angle)
            adjustment = offset_adjustment + angle_adjustment

            print(f"{self.control_mode},{rospy.Time.now().to_sec():.2f},{offset_adjustment:.3f},{angle_adjustment:.3f},{adjustment:.3f}")

        else:
            rospy.logwarn("Unknown control mode, defaulting to PID+EWMA.")
            offset_adjustment = self.offset_pid.compute(setpoint=0, measurement=smoothed_offset)
            angle_adjustment = self.angle_pid.compute(setpoint=0, measurement=smoothed_angle)
            adjustment = offset_adjustment + angle_adjustment

        message = Twist2DStamped(v=0.1, omega=adjustment)
        self.publisher.publish(message)
        #rospy.loginfo(f"[MODE {self.control_mode}] Moving forward with omega={adjustment:.4f}")


    def stop(self, event=None):
        message = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(message)
        rospy.loginfo("Stopped the robot")
        self.turning = False
        self.running = False


    def publish_wheel_cmd(self, left, right):
        message = WheelsCmdStamped()
        message.vel_left = self.limit_speed(left)
        message.vel_right = self.limit_speed(right)
        self.publisher.publish(message)

    def straight_status_callback(self, msg):
        status, direction = msg.data.split(",")
        self.state = status
        self.turn_direction = direction
        #print(f"Straight Status: {self.state}, Turn Direction: {self.turn_direction}")

        '''
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
        '''

        if self.state == "TURN" and not self.turning:
            if self.turn_direction == "LEFT" and self._inter_distance < TURN_DISTANCE_THRESHOLD :
                self.turn_left()
                print("---NOW turn_left()---")
            elif self.turn_direction == "RIGHT" and self._inter_distance < TURN_DISTANCE_THRESHOLD:
                self.turn_right()
                print("---NOW turn_right()---")
        elif self.state == "STRAIGHT":
            self.forward()

    def adjust_wheels_based_on_offset_and_angle(self):
        # 計算平滑後的偏移量
        # 使用平均的方式
        #smoothed_offset = sum(self.offset_window) / len(self.offset_window) if self.offset_window else self._offset
        #smoothed_angle = sum(self.angle_window) / len(self.angle_window) if self.angle_window else self._angle

        # 使用不同權重的方式
        smoothed_offset = self.calculate_smoothed_value(self.offset_window)
        smoothed_angle = self.calculate_smoothed_value(self.angle_window)

        # 根據偏移量和角度計算調整值
        adjustment = self.calculate_combined_adjustment(smoothed_offset, smoothed_angle)

        # 根據調整值調整輪速
        if adjustment > 0:  # Turn right
            left = self._vel_left * (1 + abs(adjustment))
            right = self._vel_right * (1 - abs(adjustment))
        else:  # Turn left
            left = self._vel_left * (1 - abs(adjustment))
            right = self._vel_right * (1 + abs(adjustment))
        
        #left = self.limit_speed(left)
        #right = self.limit_speed(right)
        
        print(f"Adjusting wheels based on offset and angle adjustment: left={left}, right={right}")
        self.publish_wheel_cmd(left, right)

    def calculate_combined_adjustment(self, offset, angle, alpha=0.5):
        offset_max = 100.0 #可能要修改
        angle_max = 30.0 #可能要修改

        offset_norm = offset / offset_max
        angle_norm = angle / angle_max

        omega = alpha * offset_norm + (1 - alpha) * angle_norm

        # 若角度與偏移方向矛盾 → 補償 heading 修正
        if offset * angle < 0:
            omega += np.sign(angle) * 0.1

        # 限制 omega 變化速率（防止打方向爆衝）
        if hasattr(self, 'prev_omega'):
            omega = np.clip(omega, self.prev_omega - 0.3, self.prev_omega + 0.3)
        self.prev_omega = omega

        return np.clip(omega, -1.5, 1.5)
    
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
        if hasattr(self, 'inter_distance_subscriber'):
            self.inter_distance_subscriber.unregister()
        if hasattr(self, 'right_roi_distance_subscriber'):
            self.right_roi_distance_subscriber.unregister()


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
2025.04.04
'''