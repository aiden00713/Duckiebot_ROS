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
from collections import deque

# 利用角忽略與水平線角度為0的情況
def angle_with_horizontal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    if angle == 0 or angle == 180:
        return False
    else:
        if 30 < angle < 150:  # Adjust these thresholds based on your needs
            return True
        else:
            return False
        
def filter_lines_by_angle(lines):
    filtered_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[:4]
        if angle_with_horizontal(x1, y1, x2, y2):
            filtered_lines.append(line)
    #print(f"Number of lines after angle filtering: {len(filtered_lines)}")
    return filtered_lines

# [直線]取得線段後繪製延伸線至頂部和底部並回傳兩點位置
def extend_line(x1, y1, x2, y2, height, image):
    if x2 == x1:  # 垂直线
        x_top = x1
        x_bottom = x2
        y_top = 0
        y_bottom = height
    else:
        slope = (y2 - y1) / (x2 - x1)
        if slope == 0:  # 水平线
            x_top = x1
            x_bottom = x2
            y_top = y1
            y_bottom = y2
        else:
            y_top = 0
            x_top = int(x1 + (y_top - y1) / slope)
            y_bottom = height
            x_bottom = int(x1 + (y_bottom - y1) / slope)

    cv2.line(image, (x_top, y_top), (x_bottom, y_bottom), (255, 0, 0), 5) #藍線粗度5
    return (x_top, y_top, x_bottom, y_bottom)

# [直線]取得兩條線段的最短距離
def calculate_line_distance(line1, line2):
    def point_line_distance(px, py, x1, y1, x2, y2):
        norm = np.linalg.norm([x2 - x1, y2 - y1])
        if norm == 0:
            return np.inf
        return abs((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)) / norm

    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    distances = [
        point_line_distance(x1, y1, x3, y3, x4, y4),
        point_line_distance(x2, y2, x3, y3, x4, y4),
        point_line_distance(x3, y3, x1, y1, x2, y2),
        point_line_distance(x4, y4, x1, y1, x2, y2)
    ]
    return min(distances)

# [直線]根據距離過濾線段
def filter_lines_by_distance(lines, distance_threshold_min, distance_threshold_max):
    filtered_lines = []
    for i, line1 in enumerate(lines):
        keep_line = False
        for j, line2 in enumerate(lines):
            if i != j:
                distance = calculate_line_distance(line1, line2)
                #print(f"Distance between line {i} and line {j}: {distance}")
                if distance_threshold_min < distance < distance_threshold_max:
                    keep_line = True
                    break

        if keep_line:
            filtered_lines.append(line1)
            filtered_lines.append(line2)
    #print(f"Number of lines after distance filtering: {len(filtered_lines)}")
    return filtered_lines

# 基于灰度值过滤线段
def filter_lines_by_intensity(image, lines, intensity_threshold):
    height, width = image.shape[:2]
    filtered_lines = []
    for line in lines:
        x1, y1, x2, y2 = map(int, line[:4])
        # Ensure coordinates are within image bounds
        if 0 <= x1 < width and 0 <= y1 < height and 0 <= x2 < width and 0 <= y2 < height:
            # 提取线段两端点的灰度值
            intensity1 = image[y1, x1]
            intensity2 = image[y2, x2]
            #print(f"Intensity of line endpoints: ({intensity1}, {intensity2})")
            # 如果两端点的灰度值都低於阈值，则保留该线段 顏色越深灰值越低
        if intensity1 < intensity_threshold and intensity2 < intensity_threshold:
            filtered_lines.append(line)
    #print(f"Number of lines after intensity filtering: {len(filtered_lines)}")
    return filtered_lines


# [轉彎]計算線段所夾角度 0-180度之間
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

# [轉彎]利用線段交點找到路口
def find_intersection(line1, line2, tolerance=1.0):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom != 0:
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        # Check if the intersection point (px, py) is within tolerance of both line segments
        if is_point_on_line_segment(px, py, line1, tolerance) and is_point_on_line_segment(px, py, line2, tolerance):
            return (px, py)
        else:
            return None
    else:
        return None

# [轉彎]線段交點誤差值子函式
def is_point_on_line_segment(px, py, line, tolerance):
    x1, y1, x2, y2 = line
    min_x, max_x = min(x1, x2), max(x1, x2)
    min_y, max_y = min(y1, y2), max(y1, y2)
    
    # Check if the point (px, py) is within the bounding box of the line segment with the given tolerance
    return (min_x - tolerance <= px <= max_x + tolerance) and (min_y - tolerance <= py <= max_y + tolerance)


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

# [轉彎]計算與影像底部正中間的距離
def dist_from_bottom_center(x1, y1, width, height):
    button_center_x = width / 2
    button_center_y = height
    distance = np.sqrt((x1 - button_center_x) ** 2 + (y1 - button_center_y) ** 2)
    return distance

# [直線]計算直線角度
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


# [直線]計算橫向偏移量
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

# [轉彎]判斷轉彎輔助線-虛線
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
    # 檢查影像是否成功加入
    if image is None:
        print("Error: Image not found or unable to load.")
        return None

    # 灰度
    height, width = image.shape[:2]
    cropped_src = image[height // 2:height, :]
    red = cropped_src[:, :, 2]

    # 高斯模糊
    blur = cv2.GaussianBlur(red, (7, 7), 0)
    # Canny邊緣檢測
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

        # 檢查是否為虛線
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


# 定义一个滑动窗口大小
WINDOW_SIZE = 20

# 初始化用于存储之前几帧线段数据的队列
left_line_history = deque(maxlen=WINDOW_SIZE)
right_line_history = deque(maxlen=WINDOW_SIZE)

def smooth_lines(line_history, new_line):
    # 将新线段加入历史记录
    line_history.append(new_line)
    # 计算历史记录中的平均值
    return np.mean(line_history, axis=0).astype(int)

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

        #publisher 距離左/右側路口的距離
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/left_dist", Float32, queue_size=10)

        #publisher straight status
        self.straight_status_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/straight_status", String, queue_size=10)
        
        #publisher offset
        self.offset_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/offset", Float32, queue_size=10)
        
        # 定義左右平行四邊形區域 ROI
        self.left_roi_points = np.array([[100, 240], [200, 240], [200, 480], [100, 480]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[350, 240], [450, 240], [450, 480], [350, 480]], np.int32).reshape((-1, 1, 2))

        # 初始狀態是直線
        self.state = "STRAIGHT"
        self.turn_direction = "NONE"

    def process_image(self, src):
        # Get dimensions of the image
        height, width = src.shape[:2]
        # Keep only the lower half of the image
        cropped_src = src[height//2:height, :]
        # Isolate the red channel
        red = cropped_src[:, :, 2]
        # Apply Gaussian blur to remove noise and shadows
        gaussian = cv2.GaussianBlur(red, (5, 5), 0)
        edges = cv2.Canny(gaussian, 70, 210)

        # Debugging: Show intermediate images
        #cv2.imshow("Red Channel", red)
        #cv2.imshow("Gaussian Blur", gaussian)
        #cv2.imshow("Canny Edges", edges)

        # Assume LineSegmentDetectionED is a function defined elsewhere
        lines = LineSegmentDetectionED(edges, min_line_len=20, line_fit_err_thres=1.4)
        #lines = LineSegmentDetectionED(gaussian, min_line_len=20, line_fit_err_thres=1.4)
        
        '''
        # Debugging: Draw all detected lines before filtering
        debug_image = edges.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = map(int, line[:4])
                cv2.line(debug_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.imshow("All Detected Lines", debug_image)
        '''

        left_lines = []
        right_lines = []
        center_x = width // 2

        if lines is not None and len(lines) > 0:
            # 根據角度忽略水平線
            lines = filter_lines_by_angle(lines)
            # 根據影像灰值過濾線段
            lines = filter_lines_by_intensity(gaussian, lines, 100)
            # 根據距離閥值過濾線段 min-max
            lines = filter_lines_by_distance(lines, 100, 200)

            
            for line in lines:
                x1, y1, x2, y2 = line[:4]

                cv2.line(gaussian, (x1, y1), (x2, y2), (255, 255, 255), 2)
                if x1 < center_x and x2 < center_x:
                    left_lines.append(line)
                elif x1 > center_x and x2 > center_x:
                    right_lines.append(line)

            print(f"Left lines: {len(left_lines)}, Right lines: {len(right_lines)}")

            if left_lines:
                left_line = np.median(left_lines, axis=0).astype(int)
                left_smoothed = smooth_lines(left_line_history, left_line)
                left_extend = extend_line(*left_smoothed[:4], height, gaussian)
                cv2.line(gaussian, (left_smoothed[0], left_smoothed[1]), (left_smoothed[2], left_smoothed[3]), (0, 255, 0), 2)

            if right_lines:
                right_line = np.median(right_lines, axis=0).astype(int)
                right_smoothed = smooth_lines(right_line_history, right_line)
                right_extend = extend_line(*right_smoothed[:4], height, gaussian)
                cv2.line(gaussian, (right_smoothed[0], right_smoothed[1]), (right_smoothed[2], right_smoothed[3]), (0, 255, 0), 2)

            '''
            # Output the distances for debugging
            for i, line1 in enumerate(lines):
                for j, line2 in enumerate(lines):
                    if i != j:
                        distance = calculate_line_distance(line1, line2)
                        midpoint_x = (line1[0] + line1[2] + line2[0] + line2[2]) // 8
                        midpoint_y = (line1[1] + line1[3] + line2[1] + line2[3]) // 8
                        cv2.putText(gaussian, f"{distance:.2f}", (midpoint_x, midpoint_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            '''
        
        left_line = np.mean(left_lines, axis=0).astype(int) if left_lines else [0, 0, 0, 0]
        right_line = np.mean(right_lines, axis=0).astype(int) if right_lines else [width, 0, width, 0]

        left_x1, left_y1, left_x2, left_y2 = left_line
        right_x1, right_y1, right_x2, right_y2 = right_line

        left_x_intercept = left_x1 + (left_x2 - left_x1) * (height - left_y1) / (left_y2 - left_y1) if left_y2 != left_y1 else left_x1
        right_x_intercept = right_x1 + (right_x2 - right_x1) * (height - right_y1) / (right_y2 - right_y1) if right_y2 != right_y1 else right_x1

        lane_center = (left_x_intercept + right_x_intercept) / 2
        offset = center_x - lane_center

        cv2.line(gaussian, (center_x, 0), (center_x, height), (0, 255, 0), 2)    
        return gaussian, offset

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
        #lines = LineSegmentDetectionED(edges, min_line_len=min_line_len, line_fit_err_thres=1.4)
        # 使用 EDLines 進行線段檢測（直向）
        vertical_lines = LineSegmentDetectionED(gaussian, min_line_len=min_line_len_vertical, line_fit_err_thres=1.4)
        # 使用 EDLines 進行線段檢測（横向）
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
                        if angle is not None and (60 < angle < 80):
                            detected_right_angle = True
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 0, 255), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 0, 255), 2)
                            #print(f"Detected right angle at intersection: {intersection}, angle: {angle}")
                        else:
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 255, 0), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 255, 0), 2)
                
        return warped, vertical_lines, horizontal_lines, detected_right_angle


    def callback(self, msg):
        # convert JPEG bytes to CV image
        left_steering_angle = 0
        right_steering_angle = 0
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        dynamic_min_line_len_vertical = 10
        dynamic_min_line_len_horizontal = 20
        
        right_detected_right_angle = False
        left_detected_right_angle = False

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
            '''if self.check_straight(image):
                self.state = "STRAIGHT"
                self.turn_direction = "NONE"'''


        left_processed_image, left_vertical_lines, left_horizontal_lines, left_detected_right_angle = self.detect_lane(image.copy(), self.left_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)
        right_processed_image, right_vertical_lines, right_horizontal_lines, right_detected_right_angle = self.detect_lane(image.copy(), self.right_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)

        left_steering_angle = calculate_steering_angle(left_vertical_lines)
        right_steering_angle = calculate_steering_angle(right_vertical_lines)
        print(f"right_steering_angle: {right_steering_angle}")
        
        processed_image, offset = self.process_image(image.copy())
        self.offset_pub.publish(offset)
        print(f"Current offset: {offset}")

        height, width = processed_image.shape[:2]

        #print(f"Left Steering Angle: {left_steering_angle:.2f} degrees")
        #print(f"Right Steering Angle: {right_steering_angle:.2f} degrees")

        # 發布目前狀態
        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")


        if right_detected_right_angle:
            print("Detected RIGHT_ROI right angle.")
            dist = dist_from_bottom_center(right_vertical_lines[0][0], right_vertical_lines[0][1], width, height)
            print(f"RIGHT_ROI Intersection Distance: {dist:.2f}")
            self.right_inter_dist_pub.publish(Float32(dist))

        if left_detected_right_angle:
            print("Detected LEFT_ROI right angle.")
            dist = dist_from_bottom_center(left_vertical_lines[0][0], left_vertical_lines[0][1], width, height)
            print(f"LEFT_ROI Intersection Distance: {dist:.2f}")
            self.left_inter_dist_pub.publish(Float32(dist))

        self.angle_pub.publish(Float32((left_steering_angle + right_steering_angle) / 2))
        
        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_left, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_right, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window, processed_image)
        #cv2.imshow(self._window_left, left_processed_image)
        #cv2.imshow(self._window_right, right_processed_image)
        cv2.waitKey(1)

    def check_straight(self, image):
        height, width = image.shape[:2]
        center_x = width / 2
        center_threshold = 20  # 閥值

        left_processed_image, left_lines, _ = self.detect_lane(image.copy(), self.left_roi_points)
        right_processed_image, right_lines, _ = self.detect_lane(image.copy(), self.right_roi_points)

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
