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

# 利用角忽略與水平線角度為0的情況
def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    if angle == 0:
        return None
    else:
        if angle > 30 and angle < 165:
            return angle
        else:
            return None

# 
def extend_line(x1, y1, x2, y2, height, image):
    if x2 != x1:
        slope = (y2 - y1) / (x2 - x1)
        if slope != 0:
            y_top = 0
            x_top = int(x1 + (y_top - y1) / slope)
            y_bottom = height
            x_bottom = int(x1 + (y_bottom - y1) / slope)
        else:  # 水平线
            x_top = x1
            x_bottom = x2
            y_top = y1
            y_bottom = y2
    else:  # 垂直线
        x_top = x1
        x_bottom = x2
        y_top = 0
        y_bottom = height

    cv2.line(image, (x_top, y_top), (x_bottom, y_bottom), (255, 0, 0), 5)
    return (x_top, y_top, x_bottom, y_bottom)

# 計算線段所夾角度
def angle_between_lines(line1, line2):
    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    vector1 = np.array([x2 - x1, y2 - y1])
    vector2 = np.array([x4 - x3, y4 - y3])

    unit_vector1 = unit_vector(vector1)
    unit_vector2 = unit_vector(vector2)

    dot_product = np.dot(unit_vector1, unit_vector2)
    angle = np.arccos(dot_product)
    angle = np.degrees(angle)
    
    return angle

# 利用線段焦點找到路口
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

# 繪製網格圖（用在main-camera）
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

# 計算距離
def distance(x1, y1, width, height):
    center_x = width / 2
    center_y = height / 2
    distance = np.sqrt((x1 - center_x) ** 2 + (y1 - center_y) ** 2)
    return distance

# 計算x距離
def x_distance(x1, width):
    center_x = width / 2
    x_distance = abs(x1 - center_x)
    return x_distance

#查表換算角度
def lookup_xtable(distance):
    if distance > 0 and distance <= 160:
        return 70
    elif distance > 160 and distance <= 320:
        return 50
    else:
        return 0

# 計算直線角度
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

# 計算偏差值
def calculate_offset(image):
        height, width = image.shape[:2]
        center_x = width / 2

        # 使用 EDLines 進行線段檢測
        lines = LineSegmentDetectionED(image, min_line_len=20, line_fit_err_thres=1.4)

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

def is_dashed_line(points, length_threshold=100, gap_threshold=30):
    """
    判断是否为虚线。
    如果线段之间的间隔大于gap_threshold且线段长度小于length_threshold，则认为是虚线。
    """
    # 计算每个线段的长度
    lengths = np.sqrt(np.sum((points[1::2] - points[::2])**2, axis=1))
    # 计算相邻线段的间隔
    distances = np.sqrt(np.sum(np.diff(points[::2], axis=0)**2, axis=1))
    
    #print(f"Line lengths: {lengths}")
    #print(f"Line gaps: {distances}")
    
    # 判断是否为虚线
    dashed_lines = []
    for i in range(len(lengths)):
        if lengths[i] < length_threshold and (i == 0 or distances[i-1] > gap_threshold):
            dashed_lines.append((points[2*i], points[2*i+1]))
    
    #print(f"Dashed line segments: {dashed_lines}")
    return len(dashed_lines) >= 3, dashed_lines

def detect_curved_lane(image):
    # 检查图像是否加载成功
    if image is None:
        print("Error: Image not found or unable to load.")
        return None

    # 灰度化
    height, width = image.shape[:2]
    cropped_src = image[height // 2:height, :]
    red = cropped_src[:, :, 2]

    # 高斯模糊
    blur = cv2.GaussianBlur(red, (7, 7), 0)
    # Canny边缘检测
    edges = cv2.Canny(blur, 50, 150)

    # EDLines
    lines = LineSegmentDetectionED(edges, min_line_len=10, line_fit_err_thres=1.4)

    all_points = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = map(int, line[:4])
            y1 += height // 2  # 调整 y 坐标，因为我们只使用了图像的下半部分
            y2 += height // 2  # 调整 y 坐标，因为我们只使用了图像的下半部分
            all_points.append((x1, y1))
            all_points.append((x2, y2))
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        #print(f"Detected points: {all_points}")

        all_points = np.array(all_points, dtype=np.int32)

        # 检查是否为虚线
        is_dashed, dashed_lines = is_dashed_line(all_points)
        if is_dashed:  # 如果检测到虚线
            dashed_points = np.array([pt for segment in dashed_lines for pt in segment], dtype=np.int32)
            x = dashed_points[:, 0]
            y = dashed_points[:, 1]

            # 使用多项式拟合
            z = np.polyfit(x, y, 2)
            p = np.poly1d(z)

            # 控制弧线长度
            x_min = np.min(x)
            x_max = np.min(x) + (np.max(x) - np.min(x)) / 2  # 调整弧线长度为原来的一半

            # 生成拟合曲线的点
            x_new = np.linspace(x_min, x_max, 100)
            y_new = p(x_new)

            # 绘制弧线
            for i in range(len(x_new) - 1):
                cv2.line(image, (int(x_new[i]), int(y_new[i])), (int(x_new[i + 1]), int(y_new[i + 1])), (0, 0, 255), 3)
            cv2.putText(image, 'Curved Lane', (int(x_new[0]), int(y_new[0]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        else:
            print("No dashed line detected or not enough points for fitting.")
    else:
        print("No lines detected.")

    return image

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
        self._window = "MAIN_camera-reader"
        self._window_left = "Left ROI"
        self._window_right = "Right ROI"

        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        #publisher angle
        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/angles", Float32, queue_size=10)

        #publisher inter_dis
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/left_dist", Float32, queue_size=10)

        #publisher straight status
        self.straight_status_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/straight_status", String, queue_size=10)

        # 定義左右平行四邊形區域 ROI
        self.left_roi_points = np.array([[100, 240], [200, 240], [200, 480], [100, 480]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[350, 240], [450, 240], [450, 480], [350, 480]], np.int32).reshape((-1, 1, 2))

        #publisher offset
        self.offset_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/offset", Float32, queue_size=10)

        # 初始狀態是直線
        self.state = "STRAIGHT"
        self.turn_direction = "NONE" 

    # roi線段檢測
    def detect_lane(self, frame, roi_points, min_line_len_vertical, min_line_len_horizontal):
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
        gaussian = cv2.GaussianBlur(red, (5, 5), 0)
        edges = cv2.Canny(gaussian, 50, 150)

        # 使用 EDLines 進行線段檢測
        vertical_lines = LineSegmentDetectionED(gaussian, min_line_len=min_line_len_vertical, line_fit_err_thres=1.4)
        horizontal_lines = LineSegmentDetectionED(gaussian, min_line_len=min_line_len_horizontal, line_fit_err_thres=1.4)

        # 判斷路口直角
        detected_right_angle = False

        if vertical_lines is not None and horizontal_lines is not None:
            for v_line in vertical_lines:
                for h_line in horizontal_lines:
                    left_extended = extend_line(*v_line[:4], h, warped)
                    right_extended = extend_line(*h_line[:4], h, warped)

                    intersection = find_intersection(left_extended, right_extended)
                    if intersection:
                        angle = angle_between_lines(left_extended, right_extended)
                        if angle is not None and (angle > 45 and angle < 70):
                            detected_right_angle = True
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 0, 255), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 0, 255), 2)
                        else:
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 255, 0), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 255, 0), 2)
                
        return warped, vertical_lines, horizontal_lines, detected_right_angle

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        dynamic_min_line_len_vertical = 10
        dynamic_min_line_len_horizontal = 20
        
        # Detect lanes and right angles
        left_processed_image, left_vertical_lines, left_horizontal_lines, left_detected_right_angle = self.detect_lane(image.copy(), self.left_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)
        right_processed_image, right_vertical_lines, right_horizontal_lines, right_detected_right_angle = self.detect_lane(image.copy(), self.right_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)

        # Detect curved lane
        curved_lane_image = detect_curved_lane(image.copy())

        # 狀態轉換
        if self.state == "STRAIGHT":
            if right_detected_right_angle:
                self.state = "PREPARE_TURN"
                self.turn_direction = "RIGHT"
                dynamic_min_line_len_vertical = 5
                dynamic_min_line_len_horizontal = 10
            elif left_detected_right_angle:
                self.state = "PREPARE_TURN"
                self.turn_direction = "LEFT"
                dynamic_min_line_len_vertical = 5
                dynamic_min_line_len_horizontal = 10
        elif self.state == "PREPARE_TURN":
            if self.turn_direction == "RIGHT" and not right_detected_right_angle:
                self.state = "TURN"
            elif self.turn_direction == "LEFT" and not left_detected_right_angle:
                self.state = "TURN"
            if self.turn_direction == "RIGHT" or self.turn_direction == "LEFT":
                self.handle_curved_lane(curved_lane_image)
        elif self.state == "TURN":
            if self.check_straight(image):
                self.state = "STRAIGHT"
                self.turn_direction = "NONE"
        
        left_steering_angle = calculate_steering_angle(left_vertical_lines)
        right_steering_angle = calculate_steering_angle(right_vertical_lines)
        
        processed_image = self.process_image(image.copy())
        height, width = processed_image.shape[:2]

        offset = calculate_offset(processed_image)
        self.offset_pub.publish(offset)
        print(f"Current offset: {offset}")

        #print(f"Left Steering Angle: {left_steering_angle:.2f} degrees")
        #print(f"Right Steering Angle: {right_steering_angle:.2f} degrees")

        # 發布目前狀態
        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")

        if right_detected_right_angle:
            print("Detected RIGHT_ROI right angle.")
            dist = distance(right_vertical_lines[0][0], right_vertical_lines[0][1], width, height)
            print(f"RIGHT_ROI Intersection Distance: {dist:.2f}")
            self.right_inter_dist_pub.publish(Float32(dist))

        if left_detected_right_angle:
            print("Detected LEFT_ROI right angle.")
            dist = distance(left_vertical_lines[0][0], left_vertical_lines[0][1], width, height)
            print(f"LEFT_ROI Intersection Distance: {dist:.2f}")
            self.left_inter_dist_pub.publish(Float32(dist))

        self.angle_pub.publish(Float32((left_steering_angle + right_steering_angle) / 2))
        
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
        #edges = cv2.Canny(gaussian, 50, 150)
        # Assume LineSegmentDetectionED is a function defined elsewhere
        lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)

        left_lines = []
        right_lines = []
        center_x = width / 2

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[:4]
                if x1 < center_x and x2 < center_x:
                    left_lines.append(line)
                elif x1 > center_x and x2 > center_x:
                    right_lines.append(line)

            if left_lines:
                left_line = np.mean(left_lines, axis=0).astype(int)
                left_extended = extend_line(*left_line[:4], height, gaussian)
                #cv2.line(gaussian, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 255, 0), 2)

            '''
            if right_lines:
                right_line = np.mean(right_lines, axis=0).astype(int)
                right_extended = extend_line(*right_line[:4], height, gaussian)
                #cv2.line(gaussian, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 255, 0), 2)

            if left_lines and right_lines:
                intersection = find_intersection(left_extended, right_extended)
                if intersection:
                    cv2.circle(gaussian, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 255), -1)
            '''
        return gaussian
    
    def check_straight(self, image):
        height, width = image.shape[:2]
        center_x = width / 2
        center_threshold = 20  # 閥值

        left_processed_image, left_lines, _, _ = self.detect_lane(image.copy(), self.left_roi_points, 10, 20)
        right_processed_image, right_lines, _, _ = self.detect_lane(image.copy(), self.right_roi_points, 10, 20)

        if left_lines is not None and right_lines is not None:
            left_line = np.mean(left_lines, axis=0).astype(int)
            right_line = np.mean(right_lines, axis=0).astype(int)
            
            left_extended = extend_line(*left_line[:4], height, image)
            right_extended = extend_line(*right_line[:4], height, image)
            
            intersection = find_intersection(left_extended, right_extended)
            
            if intersection:
                cv2.circle(image, (int(intersection[0]), int(intersection[1])), 10, (0, 0, 255), -1)  # Highlight intersection
                return abs(intersection[0] - center_x) < center_threshold
        return False
    
    def handle_curved_lane(self, image):
        curved_image = detect_curved_lane(image.copy())
        if curved_image is not None:
            # Calculate the steering angle for the curved path
            height, width = image.shape[:2]
            x_coords = []
            y_coords = []
            for i in range(width):
                for j in range(height // 2, height):
                    if (curved_image[j, i] == [0, 0, 255]).all():  # Check for the red color of the curve
                        x_coords.append(i)
                        y_coords.append(j)

            if len(x_coords) > 0 and len(y_coords) > 0:
                z = np.polyfit(x_coords, y_coords, 2)
                p = np.poly1d(z)
                curvature = np.polyder(p, 2)(np.mean(x_coords))  # Second derivative gives curvature
                steering_angle = np.arctan(curvature)
                self.angle_pub.publish(Float32(steering_angle * 180 / np.pi))

if __name__ == '__main__':
    try:
        # create the node
        node = CameraReaderNode(node_name='camera_reader_node')
        # keep spinning
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
