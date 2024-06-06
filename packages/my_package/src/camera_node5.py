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
import gc

def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
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
        if slope != 0:
            y_top = 0
            x_top = int(x1 + (y_top - y1) / slope)
            y_bottom = height
            x_bottom = int(x1 + (y_bottom - y1) / slope)
        else:
            x_top = x1
            x_bottom = x2
            y_top = y1
            y_bottom = y2
    else:
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

def detect_lane(frame, roi_points):
    rect = cv2.boundingRect(roi_points)
    x, y, w, h = rect
    cropped = frame[y:y+h, x:x+w]
    pts2 = roi_points - roi_points.min(axis=0)
    dst_pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    src_pts = pts2.astype(np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(cropped, M, (w, h))

    red = warped[:, :, 2]
    gaussian = cv2.GaussianBlur(red, (5, 5), 0)
    lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)

    detected_right_angle = False
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[:4]
            angle = angle_with_horizontal(x1, y1, x2, y2)
            if angle is not None and (angle > 75):
                detected_right_angle = True
                cv2.line(warped, (x1, y1), (x2, y2), (0, 0, 255), 2)
            else:
                cv2.line(warped, (x1, y1), (x2, y2), (0, 255, 0), 2)

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

def calculate_offset(image):
    height, width = image.shape[:2]
    center_x = width / 2

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    lines = LineSegmentDetectionED(edges, min_line_len=20, line_fit_err_thres=1.4)

    if lines is None:
        return 0

    left_lines = []
    right_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[:4]
        slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
        if slope < 0 and x1 < center_x and x2 < center_x:
            left_lines.append(line)
        elif slope > 0 and x1 > center_x and x2 > center_x:
            right_lines.append(line)

    left_line = np.mean(left_lines, axis=0).astype(int) if left_lines else [0, 0, 0, 0]
    right_line = np.mean(right_lines, axis=0).astype(int) if right_lines else [width, 0, width, 0]

    left_x1, left_y1, left_x2, left_y2 = left_line
    right_x1, right_y1, right_x2, right_y2 = right_line

    left_x_intercept = left_x1 + (left_x2 - left_x1) * (height - left_y1) / (left_y2 - left_y1) if left_y2 != left_y1 else left_x1
    right_x_intercept = right_x1 + (right_x2 - right_x1) * (height - right_y1) / (right_y2 - right_y1) if right_y2 != right_y1 else right_x1

    lane_center = (left_x_intercept + right_x_intercept) / 2
    offset = center_x - lane_center
    return offset

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        self._bridge = CvBridge()

        self._window = "MAIN_camera-reader"

        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/angles", Float32, queue_size=10)
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/left_dist", Float32, queue_size=10)
        self.straight_status_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/straight_status", String, queue_size=10)
        self.offset_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/offset", Float32, queue_size=10)

        self.left_roi_points = np.array([[100, 200], [200, 200], [200, 400], [100, 400]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[350, 200], [450, 200], [450, 400], [350, 400]], np.int32).reshape((-1, 1, 2))

        self.state = "STRAIGHT"

    def callback(self, msg):
        left_steering_angle = 0
        right_steering_angle = 0
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        left_processed_image, left_lines, left_detected_right_angle = detect_lane(image, self.left_roi_points)
        right_processed_image, right_lines, right_detected_right_angle = detect_lane(image, self.right_roi_points)

        left_steering_angle = calculate_steering_angle(left_lines)
        right_steering_angle = calculate_steering_angle(right_lines)
        
        height, width = image.shape[:2]
        straight = False
        if left_lines is not None and right_lines is not None:
            if len(left_lines) > 0 and len(right_lines) > 0:
                left_line = np.mean(left_lines, axis=0).astype(int)
                right_line = np.mean(right_lines, axis=0).astype(int)
            
                left_extended = extend_line(*left_line[:4], height)
                right_extended = extend_line(*right_line[:4], height)
            
                intersection = find_intersection(left_extended, right_extended)
            
                if intersection:
                    cv2.circle(image, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 255), -1)
                    center_x = width / 2
                    center_threshold = 20
                    if abs(intersection[0] - center_x) < center_threshold:
                        straight = True
                    print(f"Intersection: {intersection}, Straight: {straight}")
                else:
                    print("No intersection found.")
            else:
                print("Not enough lines detected.")
        else:
            print("No lines detected.")

        if self.state == "STRAIGHT":
            if right_detected_right_angle:
                self.state = "PREPARE_TURN"
                self.turn_direction = "RIGHT"
            elif left_detected_right_angle:
                self.state = "PREPARE_TURN"
                self.turn_direction = "LEFT"
        elif self.state == "PREPARE_TURN":
            if self.turn_direction == "RIGHT" and not right_detected_right_angle:
                self.state = "TURN"
            elif self.turn_direction == "LEFT" and not left_detected_right_angle:
                self.state = "TURN"
        elif self.state == "TURN":
            if self.check_straight(image):
                self.state = "STRAIGHT"
                self.turn_direction = "NONE"

        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")

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

        offset = calculate_offset(processed_image)
        self.offset_pub.publish(offset)
        print(f"Current offset: {offset}")

        cv2.imshow(self._window, processed_image)
        cv2.imshow(self._window_left, left_processed_image)
        cv2.imshow(self._window_right, right_processed_image)
        cv2.waitKey(1)

        gc.collect()
    
    def process_image(self, src):
        height, width = src.shape[:2]
        cropped_src = src[height//2:height, :]
        red = cropped_src[:, :, 2]
        gaussian = cv2.GaussianBlur(red, (5, 5), 0)
        lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[:4]
                angle = angle_with_horizontal(x1, y1, x2, y2)
                if angle is not None:
                    cv2.line(gaussian, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text_position = (min(x1, x2), min(y1, y2) + 20)
                    distance_value = lookup_xtable(x_distance(x1, width))
                    cv2.putText(gaussian, f"{distance_value:.2f}", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    self.angle_pub.publish(Float32(angle))
        
        return gaussian

    def check_straight(self, image):
        height, width = image.shape[:2]
        center_x = width / 2
        center_threshold = 20

        left_processed_image, left_lines, _ = detect_lane(image, self.left_roi_points)
        right_processed_image, right_lines, _ = detect_lane(image, self.right_roi_points)

        if left_lines is not None and right_lines is not None:
            left_line = np.mean(left_lines, axis=0).astype(int)
            right_line = np.mean(right_lines, axis=0).astype(int)
            
            left_extended = extend_line(*left_line[:4], height)
            right_extended = extend_line(*right_line[:4], height)
            
            intersection = find_intersection(left_extended, right_extended)
            
            if intersection:
                return abs(intersection[0] - center_x) < center_threshold
        return False

if __name__ == '__main__':
    try:
        node = CameraReaderNode(node_name='camera_reader_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
