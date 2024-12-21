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
    if x2 == x1:  # 垂直線
        x_top = x1
        x_bottom = x2
        y_top = 0
        y_bottom = height
    else:
        slope = (y2 - y1) / (x2 - x1)
        if slope == 0:  # 水平線
            x_top = x1
            x_bottom = x2
            y_top = y1
            y_bottom = y2
        else:
            y_top = 0
            x_top = int(x1 + (y_top - y1) / slope)
            y_bottom = height
            x_bottom = int(x1 + (y_bottom - y1) / slope)

    cv2.line(image, (x_top, y_top), (x_bottom, y_bottom), (255, 0, 0), 5) # 顯示時藍線粗度5
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
    """
    根據灰度值過濾線段，保留灰度值低於閾值的線段
    """
    height, width = image.shape[:2]
    filtered_lines = []

    for line in lines:
        x1, y1, x2, y2 = map(int, line[:4])
        # 確保線段的座標在圖像範圍內
        if 0 <= x1 < width and 0 <= y1 < height and 0 <= x2 < width and 0 <= y2 < height:
            # 獲取線段兩端點的灰度值
            intensity1 = image[y1, x1] if image.ndim == 2 else np.mean(image[y1, x1])
            intensity2 = image[y2, x2] if image.ndim == 2 else np.mean(image[y2, x2])

            # 判斷是否保留線段（灰度值低於閾值）
            if intensity1 < intensity_threshold and intensity2 < intensity_threshold:
                filtered_lines.append(line)

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

# [直線]計算橫向偏移量 不使用
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


    # 如果沒有檢測到左側車道線就假設在影像最左邊
    if len(left_lines) > 0:
        left_median = np.median(np.array(left_lines), axis=0).astype(int)
        left_x_mean = left_median[0]  # 左側車道線的x座標
    else:
        left_x_mean = 0  # 沒有左側車道線，假設在最左側 = 0

    # 如果沒有檢測到右側車道線就假設在影像最右邊
    if len(right_lines) > 0:
        right_median = np.median(np.array(right_lines), axis=0).astype(int)
        right_x_mean = right_median[2]  # 右側車道線的x座標
    else:
        right_x_mean = image_width  # 沒有左側車道線，假設在最左側 = 影像寬度

    #print(f"Stop line center: {stop_line_x_center}, Left line mean: {left_x_mean}, Right line mean: {right_x_mean}")
    # 用來判斷停止線是否位於左右車道之間
    return left_x_mean < stop_line_x_center < right_x_mean


# 定義一個滑動窗格大小
WINDOW_SIZE = 20

# 儲存之前前幾筆數據的陣列
left_line_history = deque(maxlen=WINDOW_SIZE)
right_line_history = deque(maxlen=WINDOW_SIZE)

def smooth_lines(line_history, new_line, weight=0.6):
    # 加入新線段
    line_history.append(new_line)
    
    # 使用加權平均進行平滑
    weighted_average = np.average(line_history, axis=0, weights=[weight**i for i in range(len(line_history), 0, -1)])
    return weighted_average.astype(int)

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
        self._window3 = "[STRAIGHT] Yellow"

        # sliding windows for shrinking line detection
        global LINE_LENGTH_HISTORY, LINE_ANGLE_HISTORY
        LINE_LENGTH_HISTORY = deque(maxlen=20)  # 儲存線段長度的滑動窗口
        LINE_ANGLE_HISTORY = deque(maxlen=20)  # 儲存角度的滑動窗口

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

    # [無法使用]調整鏡頭快門速度
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


    # 單次影像預處理和線段偵測
    def preprocess_and_detect_lines(self, image, min_line_len=20, line_fit_err_thres=1.4):
        """
        影像預處理並進行線段偵測
        - Gaussian 模糊
        - Canny 邊緣偵測
        - LineSegmentDetectionED 偵測線段
        """
        height, width = image.shape[:2]
        # 灰度轉換，僅使用紅色通道
        red_channel = image[height//2:height, :, 2]  # 只保留下半部的紅色通道
        blurred = cv2.GaussianBlur(red_channel, (5, 5), 0)  # 模糊去噪
        edges = cv2.Canny(blurred, 70, 210)  # Canny 邊緣偵測

        # 線段偵測
        lines = LineSegmentDetectionED(edges, min_line_len=min_line_len, line_fit_err_thres=line_fit_err_thres)
        
        return lines, blurred  # 返回檢測到的線段及邊緣處理後的影像

    # [直線]主判斷程式
    def process_image(self, src):
        # 使用共用的影像處理和線段偵測
        lines, processed_image = self.preprocess_and_detect_lines(src)
        
        height, width = processed_image.shape[:2]

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
            lines = filter_lines_by_intensity(processed_image, lines, 100)
            # 根據距離閥值過濾線段 min-max
            lines = filter_lines_by_distance(lines, 100, 200)

            # 根據影像中間位置區分左、右線段
            for line in lines:
                x1, y1, x2, y2 = line[:4]
                cv2.line(processed_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
                if x1 < center_x and x2 < center_x:
                    self.left_lines.append(line)
                elif x1 > center_x and x2 > center_x:
                    self.right_lines.append(line)

            #print(f"Left lines: {len(left_lines)}, Right lines: {len(right_lines)}")

            if self.left_lines:
                left_line = np.median(self.left_lines, axis=0).astype(int)
                left_smoothed = smooth_lines(left_line_history, left_line)
                left_extend = extend_line(*left_smoothed[:4], height, processed_image)
                cv2.line(processed_image, (left_smoothed[0], left_smoothed[1]), (left_smoothed[2], left_smoothed[3]), (0, 255, 0), 2)

            if self.right_lines:
                right_line = np.median(self.right_lines, axis=0).astype(int)
                right_smoothed = smooth_lines(right_line_history, right_line)
                right_extend = extend_line(*right_smoothed[:4], height, processed_image)
                cv2.line(processed_image, (right_smoothed[0], right_smoothed[1]), (right_smoothed[2], right_smoothed[3]), (0, 255, 0), 2)

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

        cv2.line(processed_image, (center_x, 0), (center_x, height), (0, 255, 0), 2)    
        return processed_image, offset, steering_angle

    # [直線]停止線判斷程式
    def detect_stop_line(self, src):
        height, width = src.shape[:2]
        # 使用共用的影像處理和線段偵測
        lines, processed_image = self.preprocess_and_detect_lines(src)

        '''
        debug_image = processed_image.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = map(int, line[:4])
                cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.imshow("Initial Detected Lines", debug_image)
        '''

         # 如果檢測到線段，則進行過濾
        if lines is not None and len(lines) > 0:
            # 根據角度水平線
            lines = filter_lines_by_angle(lines, 1)
            # 根據影像灰度值過濾線段
            lines = filter_lines_by_intensity(processed_image, lines, 100)
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
                current_distance = max_distance + 1  # 初始設置為大於 min_distance
                
                if not final_stop_lines:
                    final_stop_lines.append(line)
                    continue

                # 檢查目前線段與已新增線段的距離
                for added_line in final_stop_lines:
                    distance = calculate_line_distance(line, added_line)
                    #print(f"Distance between lines: {distance}")

                    if distance < current_distance:
                        current_distance = distance
                
                # 只有當前線段與所有已新增線段的距離都小於min_distance時才新增
                if current_distance <= max_distance:
                    final_stop_lines.append(line)


            # 繪製線段
            for line in final_stop_lines:
                x1, y1, x2, y2 = line[:4]
                #print(f"Filtered stop line: {line}")
                cv2.line(processed_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        return processed_image


    def detect_shrinking_lines(self, image):
        """
        改進版：結合內部黑框的數量、分佈和總面積比例進行更嚴謹的槽化線判斷
        """
        height, width = image.shape[:2]

        # 提取紅色通道（只保留下半部分）
        red_channel = image[height // 2:height, :, 2]

        # 增強局部對比度 (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_red_channel = clahe.apply(red_channel)

        # 對紅色通道進行二值化，檢測高亮區域（黃色外框）
        _, binary = cv2.threshold(enhanced_red_channel, 160, 255, cv2.THRESH_BINARY)

        # 反轉二值化影像，用於檢測內部的黑框
        inverted_binary = cv2.bitwise_not(binary)

        # 形態學操作，增強輪廓結構
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilated = cv2.dilate(binary, kernel, iterations=2)
        eroded = cv2.erode(dilated, kernel, iterations=1)

        # 將紅色通道轉換為彩色影像以顯示結果
        color_red_channel = cv2.cvtColor(enhanced_red_channel, cv2.COLOR_GRAY2BGR)

        # 檢測外框輪廓
        contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        black_box_threshold = 3  # 黑框數量閾值
        black_area_ratio_threshold = 0.05  # 黑框總面積比例閾值

        for contour in contours:
            # 計算外框的面積
            area = cv2.contourArea(contour)
            if area < 500:  # 忽略過小的框
                continue

            # 計算邊界框
            x, y, w, h = cv2.boundingRect(contour)

            # 限定綠框只能出現在影像左半邊
            if x + w > width // 2:  # 如果框的右邊界超過影像中點，忽略該框
                continue
            
            # 分析內部區域的白色比例
            roi = binary[y:y + h, x:x + w]
            white_pixel_count = cv2.countNonZero(roi)
            white_ratio = white_pixel_count / (w * h)

            # 調整白色比例的閾值
            if white_ratio < 0.2:  # 如果白色區域占比過低，忽略該框
                continue

            # 繪製外框
            cv2.rectangle(color_red_channel, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 檢測內部黑框
            roi_inverted = inverted_binary[y:y + h, x:x + w]
            inner_contours, _ = cv2.findContours(roi_inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            black_box_count = 0
            total_black_area = 0
            black_box_centers = []

            for inner_contour in inner_contours:
                inner_area = cv2.contourArea(inner_contour)
                if 50 < inner_area < 2000:  # 放寬黑框面積的範圍
                    black_box_count += 1
                    total_black_area += inner_area

                    # 計算黑框中心
                    inner_x, inner_y, inner_w, inner_h = cv2.boundingRect(inner_contour)
                    center_x = x + inner_x + inner_w // 2
                    center_y = y + inner_y + inner_h // 2
                    black_box_centers.append((center_x, center_y))

                    # 繪製內部黑框
                    cv2.rectangle(color_red_channel, (x + inner_x, y + inner_y),
                                (x + inner_x + inner_w, y + inner_y + inner_h), (0, 0, 255), 1)

            # 判斷內部黑框總面積比例
            black_area_ratio = total_black_area / (w * h)

            # 檢查黑框分佈是否集中
            if len(black_box_centers) > 1:
                center_distances = [
                    np.linalg.norm(np.array(black_box_centers[i]) - np.array(black_box_centers[j]))
                    for i in range(len(black_box_centers))
                    for j in range(i + 1, len(black_box_centers))
                ]
                average_distance = np.mean(center_distances) if center_distances else 0
            else:
                average_distance = 0

            # 綜合條件判斷
            if (black_box_count > black_box_threshold and
                black_area_ratio > black_area_ratio_threshold and
                average_distance < max(w, h) / 2):  # 黑框分佈集中
                detected = True
                cv2.putText(color_red_channel, "Shrinking line detected", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return detected, color_red_channel






    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        processed_image, offset, steering_angle = self.process_image(image.copy())
        self.offset_pub.publish(Float32(offset))
        self.angle_pub.publish(Float32(steering_angle))
        #print(f"Current offset: {offset}")
        #print(f"Steering angle: {steering_angle}")

        '''
        # 發布目前狀態
        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")
        '''
        # 處理縮小線段檢測
        shrinking_detected, shrinking_image = self.detect_shrinking_lines(image.copy())

        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        #cv2.namedWindow(self._window2, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window3, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window, processed_image)
        #cv2.imshow(self._window2, self.detect_stop_line(image.copy()))
        cv2.imshow(self._window3, shrinking_image)
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
2024.11.24

'''