#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
import gc
from collections import deque


# [直線]計算線段與水平線的夾角，根據設定參數決定是忽略還是接受水平線
def angle_with_horizontal(x1, y1, x2, y2, mode):
    # 使用 arctan2 計算線段的角度，角度範圍在 [0, 180)
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
    # mode = 0: 忽略水平線，mode = 1: 只接受水平線
    if mode == 0:  # 忽略水平線
        if 30 < angle < 150:
            return True
        else:
            return False
    elif mode == 1:  # 只接受水平線
        if (angle < 10 or angle > 170) and (angle < 20 or angle > 160):
            #print(f"mode 1 Calculated angle: {angle}")
            return True
        else:
            return False

# [直線]根據角度過濾線段，新增一個參數來控制忽略或接受水平線
def filter_lines_by_angle(lines, mode):
    """
    mode:
    0 - 忽略水平線
    1 - 只接受水平線
    """
    filtered_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[:4]  # 提取線段座標
        if angle_with_horizontal(x1, y1, x2, y2, mode):
            filtered_lines.append(line)
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

# 根據灰度值過濾線段
def filter_lines_by_intensity(image, lines, intensity_threshold):
    intensity1 = 0
    intensity2 = 0
    height, width = image.shape[:2]
    filtered_lines = []
    
    for line in lines:
        x1, y1, x2, y2 = map(int, line[:4])
        # Ensure coordinates are within image bounds
        if 0 <= x1 < width and 0 <= y1 < height and 0 <= x2 < width and 0 <= y2 < height:
            # 取得線段兩端點的灰度值
            intensity1 = image[y1, x1]
            intensity2 = image[y2, x2]
            #print(f"Intensity of line endpoints: ({intensity1}, {intensity2})")
            # 如果兩端點的灰度值都低於閥值 則保留該線段 顏色越深灰值越低
        if intensity1 < intensity_threshold and intensity2 < intensity_threshold:
            filtered_lines.append(line)
    #print(f"Number of lines after intensity filtering: {len(filtered_lines)}")
    return filtered_lines


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

# [直線]計算停止線是否在左右車道線的中間
def is_between_lane_lines(stop_line, left_lines, right_lines, image_width):
    x1, y1, x2, y2 = stop_line
    stop_line_x_center = (x1 + x2) / 2


    # 如果没有检测到左车道线，假设它在图像最左侧
    if len(left_lines) > 0:
        left_median = np.median(np.array(left_lines), axis=0).astype(int)
        left_x_mean = left_median[0]  # 左车道线的x坐标
    else:
        left_x_mean = 0  # 没有左车道线，假设在最左侧

    # 如果没有检测到右车道线，假设它在图像最右侧
    if len(right_lines) > 0:
        right_median = np.median(np.array(right_lines), axis=0).astype(int)
        right_x_mean = right_median[2]  # 右车道线的x坐标
    else:
        right_x_mean = image_width  # 没有右车道线，假设在最右侧

    #print(f"Stop line center: {stop_line_x_center}, Left line mean: {left_x_mean}, Right line mean: {right_x_mean}")
    # 判断停止线是否位于左右车道线之间
    return left_x_mean < stop_line_x_center < right_x_mean


# 定義一個滑動窗格大小
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

        # initialize dynamic reconfigure client to adjust shutter speed
        #self._client = Client("/camera_node")

        # create window
        self._window = "[STRAIGHT] Main"
        self._window2 = "[STRAIGHT] STOP LINE"

        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        #publisher angle
        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_straight/angles", Float32, queue_size=10)

        #publisher offset
        self.offset_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_straight/offset", Float32, queue_size=10)
        
        # 初始狀態是直線
        self.state = "IDLE"
        self.turn_direction = "NONE"
        # 初始化 left_lines 和 right_lines
        self.left_lines = []
        self.right_lines = []

    # 調整鏡頭快門速度
    def set_shutter_speed(self, shutter_speed_value):
        # Adjust the shutter speed (exposure time) dynamically
        try:
            rospy.wait_for_service("shutter_speed", timeout=5)  # 等待服務最多5秒
            params = {"shutter_speed": shutter_speed_value}  # Shutter speed in microseconds
            self._client.update_configuration(params)
        except rospy.ROSException as e:
            rospy.logerr(f"Service is not available: {e}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call service : {e}")


    # [直線]主判斷程式
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


        center_x = width // 2

        if lines is not None and len(lines) > 0:
            # 根據角度忽略水平線
            lines = filter_lines_by_angle(lines, 0)
            # 根據影像灰值過濾線段
            lines = filter_lines_by_intensity(gaussian, lines, 100)
            # 根據距離閥值過濾線段 min-max
            lines = filter_lines_by_distance(lines, 100, 200)

            
            for line in lines:
                x1, y1, x2, y2 = line[:4]

                cv2.line(gaussian, (x1, y1), (x2, y2), (255, 255, 255), 2)
                if x1 < center_x and x2 < center_x:
                    self.left_lines.append(line)
                elif x1 > center_x and x2 > center_x:
                    self.right_lines.append(line)

            #print(f"Left lines: {len(left_lines)}, Right lines: {len(right_lines)}")

            if self.left_lines:
                left_line = np.median(self.left_lines, axis=0).astype(int)
                left_smoothed = smooth_lines(left_line_history, left_line)
                left_extend = extend_line(*left_smoothed[:4], height, gaussian)
                cv2.line(gaussian, (left_smoothed[0], left_smoothed[1]), (left_smoothed[2], left_smoothed[3]), (0, 255, 0), 2)

            if self.right_lines:
                right_line = np.median(self.right_lines, axis=0).astype(int)
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
        
        left_line = np.mean(self.left_lines, axis=0).astype(int) if self.left_lines else [0, 0, 0, 0]
        right_line = np.mean(self.right_lines, axis=0).astype(int) if self.right_lines else [width, 0, width, 0]

        left_x1, left_y1, left_x2, left_y2 = left_line
        right_x1, right_y1, right_x2, right_y2 = right_line

        left_x_intercept = left_x1 + (left_x2 - left_x1) * (height - left_y1) / (left_y2 - left_y1) if left_y2 != left_y1 else left_x1
        right_x_intercept = right_x1 + (right_x2 - right_x1) * (height - right_y1) / (right_y2 - right_y1) if right_y2 != right_y1 else right_x1

        lane_center = (left_x_intercept + right_x_intercept) / 2
        offset = center_x - lane_center

        # Calculate steering angle
        steering_angle = calculate_steering_angle(lines)

        cv2.line(gaussian, (center_x, 0), (center_x, height), (0, 255, 0), 2)    
        return gaussian, offset, steering_angle

    # [直線]停止線判斷程式
    def detect_stop_line(self, src):
        height, width = src.shape[:2]
        cropped_src = src[height//2:height, :]
        red = cropped_src[:, :, 2]
        gaussian = cv2.GaussianBlur(red, (5, 5), 0)
        edges = cv2.Canny(gaussian, 50, 150)

        # Debugging: Show intermediate images
        #cv2.imshow("Red Channel", red)
        #cv2.imshow("Gaussian Blur", gaussian)
        #cv2.imshow("Canny Edges", edges)

        # Assume LineSegmentDetectionED is a function defined elsewhere
        lines = LineSegmentDetectionED(edges, min_line_len=20, line_fit_err_thres=1.4)

        
        debug_image = red.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = map(int, line[:4])
                cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.imshow("Initial Detected Lines", debug_image)
        

         # 如果檢測到線段，則進行過濾
        if lines is not None and len(lines) > 0:
            # 根據角度水平線
            lines = filter_lines_by_angle(lines, 1)
            # 根據影像灰度值過濾線段
            lines = filter_lines_by_intensity(gaussian, lines, 100)
            # 根據距離閥值過濾線段 (最小50像素，最大100像素)
            lines = filter_lines_by_distance(lines, 30, 150)

            filtered_stop_lines = []
            
            # 過濾線段
            for line in lines:
                if is_between_lane_lines(line, self.left_lines, self.right_lines, width):
                    filtered_stop_lines.append(line)

            # 移除相距太遠的停止線
            final_stop_lines = []
            min_distance = 50  # 設定最小距離
            max_distance = 50  # 設定最大距離

            for line in filtered_stop_lines:
                current_distance = max_distance + 1  # 初始设置为大于min_distance
                
                if not final_stop_lines:
                    final_stop_lines.append(line)
                    continue

                # 检查当前线段与已添加线段的距离
                for added_line in final_stop_lines:
                    distance = calculate_line_distance(line, added_line)
                    print(f"Distance between lines: {distance}")

                    if distance < current_distance:
                        current_distance = distance
                
                # 只有当当前线段与所有已添加线段的距离都小于min_distance时才添加
                if current_distance <= max_distance:
                    final_stop_lines.append(line)


            # 繪製線段
            for line in final_stop_lines:
                x1, y1, x2, y2 = line[:4]
                #print(f"Filtered stop line: {line}")
                cv2.line(gaussian, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        return gaussian



    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image, offset, steering_angle = self.process_image(image.copy())
        self.offset_pub.publish(offset)
        self.angle_pub.publish(Float32(steering_angle))
        #print(f"Current offset: {offset}")
        #print(f"Steering angle: {steering_angle}")

        '''
        # 發布目前狀態
        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")
        '''

        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window2, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window, processed_image)
        cv2.imshow(self._window2, self.detect_stop_line(image.copy()))
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # create the node
        node = CameraReaderNode(node_name='camera_node_straight')
        #node.set_shutter_speed(15000)
        # keep spinning
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

'''
2024.09.14

'''