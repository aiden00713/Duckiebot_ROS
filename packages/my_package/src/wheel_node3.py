#!/usr/bin/env python3
import os
import math
import rospy
import cv2
import numpy as np
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32, String

# Angular velocities for each wheel (quarter rotation a second)
W_LEFT = 1.2 * (2 * math.pi)  # 表示每秒左轮转 1.2 圈
W_RIGHT = 1.2 * (2 * math.pi)  # 表示每秒右轮转 1.2 圈
TURN_SPEED = 0.5  # Adjust this value to control turn sharpness
TURN_DURATION = 50  # Number of iterations to keep turning left

# Function to calculate the angle with the horizontal axis
def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    if angle == 0:
        return None
    else:
        if angle > 30 and angle < 165:
            return angle
        else:
            return None

def draw_grid(image, grid_size=(5, 4)):
    height, width = image.shape[:2]
    dy, dx = height / grid_size[0], width / grid_size[1]

    for i in range(1, grid_size[1]):  # 垂直
        x = int(dx * i)
        cv2.line(image, (x, 0), (x, height), (255, 0, 0), 2)
    for i in range(1, grid_size[0]):  # 水平
        y = int(dy * i)
        cv2.line(image, (0, y), (width, y), (255, 0, 0), 2)

    return image

def detect_lane(frame, roi_points):
    rect = cv2.boundingRect(roi_points)
    x, y, w, h = rect
    cropped = frame[y:y+h, x:x+w].copy()
    pts2 = roi_points - roi_points.min(axis=0)
    dst_pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    src_pts = pts2.astype(np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(cropped, M, (w, h))

    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # 使用 EDLines 进行线段检测
    lines = LineSegmentDetectionED(edges, min_line_len=20, line_fit_err_thres=1.4)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[:4]
            cv2.line(warped, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return warped, lines

def calculate_steering_angle(lines):
    if lines is None:
        return 0
    angles = []
    for line in lines:
        x1, y1, x2, y2 = line[:4]
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        angles.append(angle)
    avg_angle = np.mean(angles)
    return avg_angle

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
        self.counter = 0  # Initialize counter

        # Topics for ToF sensor and camera node angles & dist
        self._tof = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._angle_topic = f"/{self._vehicle_name}/camera_node/angles"
        self._right_dist_topic = f"/{self.vehicle_name}/camera_node/right_dist"
        self._left_dist_topic = f"/{self.vehicle_name}/camera_node/left_dist"

        # Construct publisher and subscriber
        self.mul = 0
        self._dis = 0
        self._d_pre = 0
        self.receiver = rospy.Subscriber("Duckie", String, self.call)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.distance = rospy.Subscriber(self._tof, Range, self.dis)
        #self.angle_subscriber = rospy.Subscriber(self._angle_topic, Float32, self.angle_callback)
        self.right_dist_subscriber = rospy.Subscriber(self._right_dist_topic, Float32, self.right_dist_callback)
        self.left_dist_subscriber = rospy.Subscriber(self._left_dist_topic, Float32, self.left_dist_callback)

        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()
        self._camera_sub = rospy.Subscriber(f"/{self._vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)

    def call(self, data):
        self.mul = int(data.data)

    def dis(self, data):
        self._d_pre = self._dis
        self._dis = int(1000 * data.range)

    def adjust(self, par):
        l = 0  # 左轮初始值
        r = 0  # 右轮初始值

        if par >= 100:
            par = par * 0.32 - 23.6

        if par <= -100:
            par = (par / 10) - 2
        if par >= 9:
            l = par / 40 * self._vel_left
        if par <= -9:
            r = -par / 40 * self._vel_right
        return l, r

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

    def angle_callback(self, msg):
        angle = msg.data
        rospy.loginfo(f"Received angle: {angle}")
        if abs(angle) > 70:  # Assuming 70 degrees indicates a sharp turn
            for _ in range(TURN_DURATION):
                self.turn_left()
    
    # under 340 turn right
    def right_dist_callback(self, msg):
        dist = msg.data
        rospy.loginfo(f"Received RIGHT inter. dist: {dist}")
        if dist < 340:  
            for _ in range(TURN_DURATION):
                self.turn_right()
        else:
            for _ in range(TURN_DURATION):
                self.forward()
    
    
    def left_dist_callback(self, msg):
        dist = msg.data
        rospy.loginfo(f"Received LEFT inter. dist: {dist}")
        if dist < 340:  
            for _ in range(TURN_DURATION):
                self.turn_left()
        else:
            for _ in range(TURN_DURATION):
                self.forward()

    
    def camera_callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        left_processed_image, left_lines = detect_lane(image, self.left_roi_points)
        right_processed_image, right_lines = detect_lane(image, self.right_roi_points)

        left_steering_angle = calculate_steering_angle(left_lines)
        right_steering_angle = calculate_steering_angle(right_lines)

        self.angle_callback(Float32((left_steering_angle + right_steering_angle) / 2))

        left_processed_image_grid = draw_grid(left_processed_image)
        right_processed_image_grid = draw_grid(right_processed_image)
        combined_image = np.hstack((left_processed_image_grid, right_processed_image_grid))
        cv2.imshow("Camera Image", combined_image)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        pre = 0
        while not rospy.is_shutdown():
            est_mul = pre * 0.1 + self.mul * 0.9
            left, right = self.adjust(est_mul)
            left = self._vel_left + left
            right = self._vel_right + right

            message = WheelsCmdStamped(vel_left=left, vel_right=right)
            self._publisher.publish(message)
            pre = self.mul
            rate.sleep()

    def on_shutdown(self):
        if hasattr(self, '_publisher'):
            stop = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(stop)
        print("Shut down completed")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()
