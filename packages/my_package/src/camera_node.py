#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32


# Function to calculate the angle with the horizontal axis
def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    # 忽略與水平線角度為0度的情況
    if angle == 0:
        return None
    else:
        if angle > 30 and angle < 165:
            #print(f"Angle: {angle:.2f} degrees")
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

def calculate_slope(x1, y1, x2, y2):
    if x2 - x1 == 0:  # Avoid division by zero
        return None  # Vertical line
    return (y2 - y1) / (x2 - x1)

def calculate_angle(slope1, slope2):
    if slope1 is None or slope2 is None:  # One of the lines is vertical
        return 90
    tan_theta = abs((slope2 - slope1) / (1 + slope1 * slope2))
    return np.degrees(np.arctan(tan_theta))

def line_intersection(line1, line2):
    # Unpack points
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    # Calculate determinant
    det = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)
    if det == 0:
        return None  # lines are parallel

    # Calculate the intersection point
    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    x = (det1 * (x3 - x4) - (x1 - x2) * det2) / det
    y = (det1 * (y3 - y4) - (y1 - y2) * det2) / det
    return x, y

def detect_lane(frame, roi_points):
    rect = cv2.boundingRect(roi_points)
    x, y, w, h = rect
    cropped = frame[y:y+h, x:x+w].copy()
    pts2 = roi_points - roi_points.min(axis=0)
    dst_pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    src_pts = pts2.astype(np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(cropped, M, (w, h))

    red = warped[:, :, 2]
    gaussian = cv2.GaussianBlur(red, (7, 7), 0)

    # Placeholder function for line detection, replace with your actual function call
    lines = LineSegmentDetectionED(gaussian, min_line_len=30, line_fit_err_thres=1.6)

    detected_right_angle = False
    angle = 0

    if lines is not None and len(lines) > 1:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1 = lines[i]
                line2 = lines[j]
                #x1, y1, x2, y2 = line1[:4]
                #x3, y3, x4, y4 = line2[:4]
                intersection = line_intersection(line1, line2)

                '''
                slope1 = calculate_slope(x1, y1, x2, y2)
                slope2 = calculate_slope(x3, y3, x4, y4)
                angle = calculate_angle(slope1, slope2)
                mid_x1, mid_y1 = (x1 + x2) // 2, (y1 + y2) // 2
                '''
                if intersection is not None:
                    x1, y1, x2, y2 = line1
                    x3, y3, x4, y4 = line2
                    slope1 = calculate_slope(x1, y1, x2, y2)
                    slope2 = calculate_slope(x3, y3, x4, y4)
                    angle = calculate_angle(slope1, slope2)
                    mid_x1, mid_y1 = (x1 + x2) // 2, (y1 + y2) // 2
                else:
                    rospy.loginfo("ROI NO Intersection")

                if 60 <= angle <= 75:  # Checking for near-right angles
                    detected_right_angle = True
                    cv2.line(gaussian, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red: right angle
                    cv2.line(gaussian, (x3, y3), (x4, y4), (0, 0, 255), 2)  # Red: right angle
                    cv2.putText(gaussian, f"{angle:.2f} ", (mid_x1, mid_y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    #rospy.loginfo("No degree between 60-75")
                    cv2.line(gaussian, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue: only line
                    cv2.line(gaussian, (x3, y3), (x4, y4), (255, 0, 0), 2)  # Blue: only line
                    #cv2.putText(gaussian, f"{angle:.2f} ", (mid_x1, mid_y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    else:
        rospy.loginfo("[ROI detect_lane] No lines detected or insufficient lines for angle calculation")

    return gaussian, lines, detected_right_angle


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

        self.left_roi_points = np.array([[150, 200], [200, 200], [200, 400], [150, 400]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[350, 200], [450, 200], [450, 400], [350, 400]], np.int32).reshape((-1, 1, 2))

        # Dictionary to store intersection data
        self.intersection_data = {}

    def callback(self, msg):
        # convert JPEG bytes to CV image
        left_steering_angle = 0
        right_steering_angle = 0
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        #height, width = image.shape[:2]
        #rospy.loginfo(f"Image size: width={width}, height={height}")

        left_processed_image, left_lines, left_detected_right_angle = detect_lane(image, self.left_roi_points)
        right_processed_image, right_lines, right_detected_right_angle = detect_lane(image, self.right_roi_points)

        if left_steering_angle != 0 or right_steering_angle != 0:
            self.angle_callback(Float32((left_steering_angle + right_steering_angle) / 2))

        left_steering_angle = calculate_steering_angle(left_lines)
        right_steering_angle = calculate_steering_angle(right_lines)
        print(f"Left Steering Angle: {left_steering_angle:.2f} degrees")
        print(f"Right Steering Angle: {right_steering_angle:.2f} degrees")

        if right_detected_right_angle:
            rospy.loginfo("Detected RIGHT_ROI right angle.")
            height, width = image.shape[:2]
            dist = distance(right_lines[0][0], right_lines[0][1], width, height)
            rospy.loginfo(f"RIGHT_ROI Intersection Distance: {dist:.2f}")
            self.right_inter_dist_pub.publish(Float32(dist))


        if left_detected_right_angle:
            rospy.loginfo("Detected LEFT_ROI right angle.")
            height, width = image.shape[:2]
            dist = distance(left_lines[0][0], left_lines[0][1], width, height)
            rospy.loginfo(f"LEFT_ROI Intersection Distance: {dist:.2f}")
            self.left_inter_dist_pub.publish(Float32(dist))


        self.angle_pub.publish(Float32((left_steering_angle + right_steering_angle) / 2))
        
        processed_image = self.process_image(image)

        #left_processed_image_grid = draw_grid(left_processed_image)
        #right_processed_image_grid = draw_grid(right_processed_image)

        '''
        # Process the image
        
        processed_image_grid = draw_grid(processed_image)
        '''
        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_left, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_right, cv2.WINDOW_AUTOSIZE)
        #cv2.imshow(self._window, processed_image)
        #cv2.imshow(self._window, processed_image_grid)
        #combined_image = np.hstack((left_processed_image_grid, right_processed_image_grid))
        cv2.imshow(self._window, processed_image)
        cv2.imshow(self._window_left, left_processed_image)
        cv2.imshow(self._window_right, right_processed_image)
        #cv2.imshow(self._window, combined_image)
        cv2.waitKey(1)
    
    def store_intersection_data(self, intersection_point, angle):
        """Stores the intersection data in the dictionary."""
        if intersection_point:
            self.intersection_data['point'] = intersection_point
            self.intersection_data['angle'] = angle
            # You can publish this data or log it
            #rospy.loginfo(f"Stored intersection point: {intersection_point}, angle: {angle}")

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
                else:
                    rospy.loginfo("[process_image] No lines detected or insufficient lines for angle calculation")
        else:
            rospy.loginfo("[process_image] No lines detected")

        return gaussian

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()