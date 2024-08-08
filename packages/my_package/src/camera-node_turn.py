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

width = 640
height = 480

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
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    angle = np.arccos(dot_product)
    angle = np.degrees(angle)
    
    return angle

# [轉彎]利用線段交點找到路口
def find_intersection(line1, line2, tolerance=2.0):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom != 0:
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        #print(f"Intersection point: ({px:.2f}, {py:.2f})")
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


# [轉彎]計算與影像底部正中間的距離
def dist_from_bottom_center(x1, y1, width, height):
    button_center_x = width / 2
    button_center_y = height
    distance = np.sqrt((x1 - button_center_x) ** 2 + (y1 - button_center_y) ** 2)
    return distance


# [轉彎]判斷轉彎輔助線-虛線
def is_dashed_line(points, length_min, length_max, gap_min, gap_max):
    """
    判断是否为虚线。
    如果线段之间的间隔大于gap_threshold且线段长度小于length_threshold，则认为是虚线。
    """
    points = np.array(points)  # 将列表转换为numpy数组

    # 計算每個線段的長度
    lengths = np.sqrt(np.sum((points[1::2] - points[::2])**2, axis=1))
    # 計算相鄰線段的間隔
    distances = np.sqrt(np.sum(np.diff(points[::2], axis=0)**2, axis=1))
    
    # 判斷是否為虛線
    dashed_lines = []
    for i in range(len(lengths)):
        if length_min <= lengths[i] <= length_max and (i == 0 or gap_min <= distances[i-1] <= gap_max):
            x1, y1 = points[2*i]
            x2, y2 = points[2*i+1]
            angle = np.arctan2(abs(y2 - y1), abs(x2 - x1)) * 180 / np.pi  # 计算斜率的角度
            #print(f"angle: {angle}")

            # 过滤掉接近水平或接近垂直的线段
            if 3 < angle < 15:
                dashed_lines.append((points[2*i], points[2*i+1]))

    #print(f"Detected line lengths: {lengths}")
    #print(f"Detected line distances: {distances}")
    #print(f"Dashed lines: {dashed_lines}")

    return len(dashed_lines) >= 4, dashed_lines



def detect_curved_lane(src, inter_dist_pub):
    # 檢查影像是否成功加入
    if src is None:
        print("Error: Image not found or unable to load.")
        return None, False

    # Get dimensions of the image
    height, width = src.shape[:2]
    # Keep only the lower half of the image
    cropped_src = src[height//2:height, :]
    # Isolate the red channel
    red = cropped_src[:, :, 2]
    # Apply Gaussian blur to remove noise and shadows
    gaussian = cv2.GaussianBlur(red, (5, 5), 0)
    edges = cv2.Canny(gaussian, 50, 150)
    #cv2.imshow("Canny Edges", edges)
    # EDLines
    lines = LineSegmentDetectionED(edges, min_line_len=10, line_fit_err_thres=1.4)

    all_points = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = map(int, line[:4])
            all_points.append((x1, y1))
            all_points.append((x2, y2))
            #cv2.line(gaussian, (x1, y1), (x2, y2), (0, 255, 0), 1)
        
        # 设置参数
        length_min = 10
        length_max = 40
        gap_min = 20
        gap_max = 40

        # 檢查是否為虛線
        is_dashed, dashed_lines = is_dashed_line(all_points, length_min, length_max, gap_min, gap_max)
        
        if is_dashed:  # 如果檢測到虛線
            dashed_points = np.array([pt for segment in dashed_lines for pt in segment], dtype=np.int32)
            x = dashed_points[:, 0]
            y = dashed_points[:, 1]

            # 使用多項式擬合-1次多項式
            z = np.polyfit(x, y, 1)
            p = np.poly1d(z)

            # 控制弧線長度
            x_min = np.min(x)
            x_max = np.min(x) + (np.max(x) - np.min(x)) / 2  # 調整弧線長度為原来的1/2

            # 生成擬合曲線的點
            x_new = np.linspace(x_min, x_max, 100)
            y_new = p(x_new)

            for segment in dashed_lines:
                (x1, y1), (x2, y2) = segment
                cv2.line(gaussian, (x1, y1), (x2, y2), (0, 255, 0), 1)

                dist = dist_from_bottom_center(x1, y1, 640, 480)
                inter_dist_pub.publish(Float32(dist))
                print(f"bottom center: {dist}")

            # 繪製弧線
            for i in range(len(x_new) - 1):
                cv2.line(gaussian, (int(x_new[i]), int(y_new[i])), (int(x_new[i + 1]), int(y_new[i + 1])), (0, 0, 255), 3)
            #cv2.putText(gaussian, 'Curved Lane', (int(x_new[0]), int(y_new[0]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return gaussian, True
        else:
            #print("No dashed line detected or not enough points for fitting.")
            return gaussian, False
    else:
        print("No lines detected.")
        return gaussian, False


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

        self._window_left = "Left ROI"
        self._window_right = "Right ROI"
        self._window_curved = "Curved Line"

        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        #publisher angle
        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/angles", Float32, queue_size=10)

        #publisher 距離左/右側路口的距離
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/left_dist", Float32, queue_size=10)

        #publisher 距離路口虛線的距離
        self.inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/inter_dist", Float32, queue_size=10)

        #publisher straight status
        self.straight_status_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/straight_status", String, queue_size=10)
        
        #publisher offset
        self.offset_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/offset", Float32, queue_size=10)
        
        # 定義左右平行四邊形區域 ROI 寬640 高480
        self.left_roi_points = np.array([[100, 240], [200, 240], [200, 480], [100, 480]], np.int32).reshape((-1, 1, 2))
        self.right_roi_points = np.array([[450, 240], [550, 240], [550, 480], [450, 480]], np.int32).reshape((-1, 1, 2))

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
        vertical_lines = LineSegmentDetectionED(edges, min_line_len=min_line_len_vertical, line_fit_err_thres=1.4)
        horizontal_lines = LineSegmentDetectionED(edges, min_line_len=min_line_len_horizontal, line_fit_err_thres=1.4)
        
        # 判斷路口直角
        detected_right_angle = False

        if vertical_lines is not None and horizontal_lines is not None:
            for v_line in vertical_lines:
                for h_line in horizontal_lines:
                    intersection = find_intersection(v_line, h_line)
                    if intersection:
                        angle = angle_between_lines(v_line, h_line)
                        #print(f"angle: {angle}")
                        if angle is not None and (60 < angle < 80):
                            detected_right_angle = True
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 0, 255), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 0, 255), 2)
                            print(f"Detected right angle at intersection: {intersection}, angle: {angle}")
                        else:
                            cv2.line(warped, (v_line[0], v_line[1]), (v_line[2], v_line[3]), (0, 255, 0), 2)
                            cv2.line(warped, (h_line[0], h_line[1]), (h_line[2], h_line[3]), (0, 255, 0), 2)
                
        return warped, vertical_lines, horizontal_lines, detected_right_angle


    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        #height, width = image.shape[:2]
        #print(f"height: {height}")
        #print(f"width: {width}")

        dynamic_min_line_len_vertical = 10
        dynamic_min_line_len_horizontal = 20
        
        right_detected_right_angle = False
        left_detected_right_angle = False

        left_processed_image, left_vertical_lines, left_horizontal_lines, left_detected_right_angle = self.detect_lane(image.copy(), self.left_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)
        right_processed_image, right_vertical_lines, right_horizontal_lines, right_detected_right_angle = self.detect_lane(image.copy(), self.right_roi_points, dynamic_min_line_len_vertical, dynamic_min_line_len_horizontal)

        # 狀態轉換-沒有用了
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
            if not right_detected_right_angle and not left_detected_right_angle:
                self.state = "TURN"

            '''if self.turn_direction == "RIGHT" or self.turn_direction == "LEFT":
                curved_lane_image, is_curved = detect_curved_lane(image.copy(), self.inter_dist_pub)
                if is_curved:
                    self.handle_curved_lane(curved_lane_image)
                    '''
        elif self.state == "TURN":
            pass
            '''if self.check_straight(image):
                self.state = "STRAIGHT"
                self.turn_direction = "NONE"'''

        # 發布目前狀態
        status_message = f"{self.state},{self.turn_direction}"
        self.straight_status_pub.publish(status_message)
        print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")

        height, width = image.shape[:2]
        '''
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
        '''
        # Detect curved lane
        curved_lane_image, is_curved = detect_curved_lane(image.copy(), self.inter_dist_pub)

        # Display the processed image
        cv2.namedWindow(self._window_curved, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_left, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_right, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window_curved, curved_lane_image)
        cv2.imshow(self._window_left, left_processed_image)
        cv2.imshow(self._window_right, right_processed_image)
        cv2.waitKey(1)

    def handle_curved_lane(self, image):
        # Calculate the steering angle for the curved path
        height, width = image.shape[:2]
        x_coords = []
        y_coords = []
        for i in range(width):
            for j in range(height // 2, height):
                if (image[j, i] == [0, 0, 255]).all():  # Check for the red color of the curve
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
