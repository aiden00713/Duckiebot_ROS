# 新增左右邊線延伸至畫面中央來判斷是否為正


#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32


# Function to calculate the angle with the horizontal axis
def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    # 忽略與水平線角度為0度的情況
    if angle == 0:
        return None
    else:
        if angle > 30 and angle < 165:
            return angle
        else:
            return None

def extend_line(x1, y1, x2, y2, height):
    if x2 != x1:
        slope = (y2 - y1) / (x2 - x1)
        y_top = 0
        x_top = int(x1 + (y_top - y1) / slope)
        y_bottom = height
        x_bottom = int(x1 + (y_bottom - y1) / slope)
    else:  # 垂直線
        x_top = x1
        x_bottom = x2
        y_top = 0
        y_bottom = height
    return (x_top, y_top, x_bottom, y_bottom)

def find_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom != 0:
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        return (px, py)
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

def distance(x1, y1, width, height):
    center_x = width / 2
    center_y = height / 2
    distance = np.sqrt((x1 - center_x) ** 2 + (y1 - center_y) ** 2)
    return distance

def x_distance(x1, width):
    center_x = width / 2
    x_distance = abs(x1 - center_x)
    return x_distance

def lookup_xtable(distance):
    if distance > 0 and distance <= 160:
        return 70
    elif distance > 160 and distance <= 320:
        return 50
    else:
        return 0

def detect_lane(frame, roi_points):
    rect = cv2.boundingRect(roi_points)
    x, y, w, h = rect
    cropped = frame[y:y+h, x:x+w].copy()
    pts2 = roi_points - roi_points.min(axis=0)
    dst_pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    src_pts = pts2.astype(np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    # 影像透視 不規則圖形轉平面
    warped = cv2.warpPerspective(cropped, M, (w, h))

    red = warped[:, :, 2]
    # Apply Gaussian blur to remove noise and shadows
    gaussian = cv2.GaussianBlur(red, (7, 7), 0)

    # 使用 EDLines 进行线段检测
    lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)
    
    # 判斷路口直角
    detected_right_angle = False
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[:4]
            angle = angle_with_horizontal(x1, y1, x2, y2)
            if angle is not None and (angle > 75):
                detected_right_angle = True
                cv2.line(warped, (x1, y1), (x2, y2), (0, 0, 255), 2) #red:right_angle
            else:
                cv2.line(warped, (x1, y1), (x2, y2), (0, 255, 0), 2) #blue:only line
    else:
        rospy.loginfo("No lines detected")

    return warped, lines, detected_right_angle

def calculate_steering_angle(lines):
    if lines is None:
        return 0
    angles = []
    for line in lines:
        x1, y1, x2, y2 = line[:4]
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        angles.append(angle)
    if len(angles) == 0:
        return 0
    avg_angle = np.mean(angles)
    return avg_angle

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        print(self._vehicle_name)
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # create window
        self._window = "camera-reader"

        self._window_left = "Left ROI"
        self._window_right = "Right ROI"


        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        #publisher angle
        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/angles", Float32, queue_size=10)

        #publisher inter_dis
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/left_dist", Float32, queue_size=10)

        # 定義左右平行四邊形區域 ROI
        #self.left_roi_points = np.array([[100, 0], [200, 0], [100, 480], [200, 480]], np.int32).reshape((-1, 1, 2))
        #self.right_roi_points = np.array([[400, 0], [600, 0], [400, 480], [600, 480]], np.int32).reshape((-1, 1, 2))

        self.left_roi_points = np.array([[100, 200], [200, 200], [200, 400], [100, 400]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[350, 200], [450, 200], [450, 400], [350, 400]], np.int32).reshape((-1, 1, 2))

    def callback(self, msg):
        # convert JPEG bytes to CV image
        left_steering_angle = 0
        right_steering_angle = 0
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        left_processed_image, left_lines, left_detected_right_angle = detect_lane(image, self.left_roi_points)
        right_processed_image, right_lines, right_detected_right_angle = detect_lane(image, self.right_roi_points)

        if left_steering_angle != 0 or right_steering_angle != 0:
            self.angle_pub.publish(Float32((left_steering_angle + right_steering_angle) / 2))

        left_steering_angle = calculate_steering_angle(left_lines)
        right_steering_angle = calculate_steering_angle(right_lines)
        print(f"Left Steering Angle: {left_steering_angle:.2f} degrees")
        print(f"Right Steering Angle: {right_steering_angle:.2f} degrees")

        # Extend left and right lines to find intersection
        height, width = image.shape[:2]
        if left_lines is not None and right_lines is not None:
            left_line = np.mean(left_lines, axis=0).astype(int)
            right_line = np.mean(right_lines, axis=0).astype(int)
            
            left_extended = extend_line(*left_line[:4], height)
            right_extended = extend_line(*right_line[:4], height)
            
            intersection = find_intersection(left_extended, right_extended)
            
            if intersection:
                cv2.circle(image, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 255), -1)
                center_x = width / 2
                center_threshold = 20  # 可以調整這個閾值
                if abs(intersection[0] - center_x) < center_threshold:
                    straight = True
                else:
                    straight = False
                print(f"Intersection: {intersection}, Straight: {straight}")
            else:
                print("No intersection found.")
        
        if right_detected_right_angle:
            print("Detected RIGHT_ROI right angle.")
            dist = distance(right_lines[0][0], right_lines[0][1], width, height)
            print(f"RIGHT_ROI Intersection Distance: {dist:.2f}")
            self.right_inter_dist_pub.publish(Float32(dist))

        if left_detected_right_angle:
            print("Detected LEFT_ROI right angle.")
            dist = distance(left_lines[0][0], left_lines[0][1], width, height)
            print(f"LEFT_ROI Intersection Distance: {dist:.2f}")
            self.left_inter_dist_pub.publish(Float32(dist))


        self.angle_pub.publish(Float32((left_steering_angle + right_steering_angle) / 2))
        
        processed_image = self.process_image(image)

        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_left, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_right, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window, processed_image)
        cv2.imshow(self._window_left, left_processed_image)
        cv2.imshow(self._window_right, right_processed_image)
        cv2.waitKey(1)
    

    def process_image(self, src):
        # Get dimensions of the image
        height, width = src.shape[:2]
        # Keep only the lower half of the image
        cropped_src = src[height//2:height, :]
        # Isolate the red channel
        red = cropped_src[:, :, 2]
        # Apply Gaussian blur to remove noise and shadows
        gaussian = cv2.GaussianBlur(red, (7, 7), 0)
        # Assume LineSegmentDetectionED is a function defined elsewhere
        lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[:4]
                angle = angle_with_horizontal(x1, y1, x2, y2)
                # 僅處理角度大於0的線段
                if angle is not None:
                    # 在圖像上繪製線段
                    cv2.line(gaussian, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 添加角度文字
                    text_position = (min(x1, x2), min(y1, y2) + 20)  # 顯示角度的位置稍微在線段上方
                    distance_value = lookup_xtable(x_distance(x1, width))
                    cv2.putText(gaussian, f"{distance_value:.2f}", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    # ros publisher
                    self.angle_pub.publish(Float32(angle))
        
        return gaussian

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()
