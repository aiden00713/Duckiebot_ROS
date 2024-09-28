#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from pylsd2 import LineSegmentDetectionED
from duckietown.dtros import DTROS, NodeType
from scipy.interpolate import CubicSpline, splprep, splev, PchipInterpolator
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32

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


def is_dashed_line(points, length_min, length_max, gap_min, gap_max, distance_threshold):
    """
    判断是否为虚线，并使用三次样条曲线拟合虚线的控制点。
    如果线段之间的间隔大于gap_threshold且线段长度小于length_threshold，则认为是虚线。
    """
    points = np.array(points)  # 轉換成numpy陣列

    # 計算每個線段的長度
    lengths = np.sqrt(np.sum((points[1::2] - points[::2])**2, axis=1))
    # 計算相鄰線段的間隔
    distances = np.sqrt(np.sum(np.diff(points[::2], axis=0)**2, axis=1))

    # 虛線x,y控制點陣列
    control_points_x = []
    control_points_y = []
    
    # 判斷是否為虛線
    dashed_lines = []

    for i in range(len(lengths)):
        if length_min <= lengths[i] <= length_max and (i == 0 or gap_min <= distances[i-1] <= gap_max):
            x1, y1 = points[2*i]
            x2, y2 = points[2*i+1]
            angle = np.arctan2(abs(y2 - y1), abs(x2 - x1)) * 180 / np.pi  # 計算斜率的角度

            # 過濾掉接近水平或垂直的線段
            if 3 < angle < 15:
                # 計算線段的中心點
                x_center_line = (x1 + x2) / 2
                y_center_line = (y1 + y2) / 2

                # 計算整體中心點（將每個控制點的平均值作為全局的中心點）
                overall_x_center = np.mean([p[0] for p in points])
                overall_y_center = np.mean([p[1] for p in points])

                # 只保留距離全局中心點在合理範圍內的線段
                dist_from_center = np.sqrt((x_center_line - overall_x_center)**2 + (y_center_line - overall_y_center)**2)
                if dist_from_center <= distance_threshold:
                    dashed_lines.append((points[2*i], points[2*i+1]))
                    control_points_x.extend([x1, x2])
                    control_points_y.extend([y1, y2])

    # 如果檢測到足夠的虛線段，擬合曲線
    if len(dashed_lines) >= 4 and len(control_points_x) > 3:
        control_points_x = np.array(control_points_x)
        control_points_y = np.array(control_points_y)

        # 對控制點進行排序，確保 x 軸遞增
        sorted_indices = np.argsort(control_points_x)
        control_points_x = control_points_x[sorted_indices]
        control_points_y = control_points_y[sorted_indices]

        # 移除過於接近的點，確保樣條曲線擬合不會出錯
        control_points_x, control_points_y = remove_close_points(control_points_x, control_points_y)

        # 使用 x 轴的中间值作为基准点
        x_center = (np.min(control_points_x) + np.max(control_points_x)) / 2

        # 过滤掉距离中心点过远的点
        filtered_x = []
        filtered_y = []
        for x, y in zip(control_points_x, control_points_y):
            if abs(x - x_center) <= distance_threshold:  # 只保留接近中心的点
                filtered_x.append(x)
                filtered_y.append(y)

        
        # 如果有足够的点进行拟合
        if len(filtered_x) > 3:
            # 擬合二次多項式曲線
            z = np.polyfit(filtered_x, filtered_y, 2)
            p = np.poly1d(z)

            # 返回三次樣條曲線的控制點和虛線段
            #return True, dashed_lines, control_points_x, control_points_y
            # 返回二次曲線的擬和結果
            return True, dashed_lines, p, control_points_x
        


    return False, dashed_lines, None, None

# 過濾掉x軸上過於接近的點，以保證x是嚴格遞增的
def remove_close_points(x, y, threshold=1e-6):
    
    filtered_x = [x[0]]
    filtered_y = [y[0]]
    
    for i in range(1, len(x)):
        if x[i] - filtered_x[-1] > threshold:  # 只保留x值之間距離足夠大的點
            filtered_x.append(x[i])
            filtered_y.append(y[i])
    
    return np.array(filtered_x), np.array(filtered_y)

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
        
        # 設置虛線檢測參數
        length_min = 10
        length_max = 40
        gap_min = 20
        gap_max = 40
        distance_threshold = 200

        # 檢查是否為虛線
        #is_dashed, dashed_lines, control_points_x, control_points_y = is_dashed_line(all_points, length_min, length_max, gap_min, gap_max)
        
        is_dashed, dashed_lines, poly_fit, control_points_x = is_dashed_line(all_points, length_min, length_max, gap_min, gap_max, distance_threshold)
        
        if is_dashed:  # 如果檢測到虛線
            '''
            dashed_points = np.array([pt for segment in dashed_lines for pt in segment], dtype=np.int32)
            x = dashed_points[:, 0]
            y = dashed_points[:, 1]

            # 使用多項式擬合-2次多項式
            z = np.polyfit(x, y, 2)
            p = np.poly1d(z)

            # 控制弧線長度
            x_min = np.min(x)
            x_max = np.min(x) + (np.max(x) - np.min(x)) / 2  # 調整弧線長度為原来的1/2

            # 生成擬合曲線的點
            x_new = np.linspace(x_min, x_max, 100)
            y_new = p(x_new)

            for segment in dashed_lines:
                (x1, y1), (x2, y2) = segment
                cv2.line(gaussian, (x1, y1), (x2, y2), (255, 0, 0), 2)

                dist = dist_from_bottom_center(x1, y1, 640, 480)
                inter_dist_pub.publish(Float32(dist))
                #print(f"bottom center: {dist}")

            # 繪製弧線
            for i in range(len(x_new) - 1):
                cv2.line(gaussian, (int(x_new[i]), int(y_new[i])), (int(x_new[i + 1]), int(y_new[i + 1])), (0, 0, 255), 3)
                cv2.putText(gaussian, str(dist), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            #cv2.putText(gaussian, 'Curved Lane', (int(x_new[0]), int(y_new[0]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return gaussian, True
            '''

            # 使用二次多項式擬合
            try:
                #spline = CubicSpline(control_points_x, control_points_y)
                x_fine = np.linspace(min(control_points_x), max(control_points_x), 100)
                #y_fine = spline(x_fine)
                y_fine = poly_fit(x_fine)

                for segment in dashed_lines:
                    (x1, y1), (x2, y2) = segment
                    cv2.line(gaussian, (x1, y1), (x2, y2), (255, 0, 0), 2)

                    dist = dist_from_bottom_center(x1, y1, 640, 480)
                    inter_dist_pub.publish(Float32(dist))

                # 繪製曲線
                for i in range(len(x_fine) - 1):
                    cv2.line(gaussian, (int(x_fine[i]), int(y_fine[i])), (int(x_fine[i + 1]), int(y_fine[i + 1])), (0, 0, 255), 3)

                return gaussian, True
            
            except Exception as e:
                print(f"Error in spline fitting: {e}")
                return gaussian, False

        else:
            #print("No dashed line detected or not enough points for fitting.")
            return gaussian, False
    else:
        print("No lines detected.")
        return gaussian, False

# 專門處理大轉彎虛線轉彎輔助線的影像
def BIG_detect_curved_lane(src):
    # 檢查影像是否成功加入
    if src is None:
        print("Error: Image not found or unable to load.")
        return None, False, None, None

    # Get dimensions of the image
    height, width = src.shape[:2]
    # Keep only the lower half of the image
    cropped_src = src[height//2:height, :]
    # Isolate the red channel
    red = cropped_src[:, :, 2]
    # Apply Gaussian blur to remove noise and shadows
    gaussian = cv2.GaussianBlur(red, (5, 5), 0)
    # edges detect
    edges = cv2.Canny(gaussian, 30, 100)

    # cv2.imshow("Canny Edges", edges)
    # EDLines
    lines = LineSegmentDetectionED(edges, min_line_len=5, line_fit_err_thres=1.0)
    
    left_lines = []
    right_lines = []

    height_cropped = height // 2  # 裁剪后图像的高度

    if lines is not None:
        # 第一次偵測靠近底部的線段，篩選高度靠近圖像底部的線段
        bottom_threshold = height_cropped - 50  # 設定靠近底部的高度閾值
        bottom_lines = [line for line in lines if min(line[1], line[3]) > bottom_threshold]

        if len(bottom_lines) > 0:
            # 找出線段中最小和最大的 X 和 Y 值
            x_min = min(min(line[0], line[2]) for line in bottom_lines) - 10  # X軸擴展10個像素
            x_max = max(max(line[0], line[2]) for line in bottom_lines) + 10  # X軸擴展10個像素
            roi_y_min = min(min(line[1], line[3]) for line in bottom_lines) - 50  # Y軸擴展50個像素
            roi_y_min = max(roi_y_min, 0)  # 防止越界
            roi_y_max = height_cropped + 50  # Y軸擴展50個像素

            # 確保 x_min 和 x_max 在圖像範圍內
            x_min = max(x_min, 0)
            x_max = min(x_max, width)

            # 在整張圖像上畫出 ROI 區域範圍
            cv2.rectangle(gaussian, (x_min, roi_y_min), (x_max, roi_y_max), (255, 0, 0), 2)  # 使用藍色框出ROI區域

            cropped_src_roi = cropped_src[int(roi_y_min):roi_y_max, int(x_min):int(x_max)]

            # 重新處理ROI中的影像
            red_roi = cropped_src_roi[:, :, 2]
            gaussian_roi = cv2.GaussianBlur(red_roi, (5, 5), 0)
            edges_roi = cv2.Canny(gaussian_roi, 30, 100)

            # 第二次偵測ROI區域中的線段
            lines_roi = LineSegmentDetectionED(edges_roi, min_line_len=5, line_fit_err_thres=1.0)

            # 在 gaussian 上畫出 edges_roi 偵測到的線段並進行篩選
            if lines_roi is not None:
                min_length = 5    # 最小線段長度
                max_length = 50   # 最大線段長度
                brightness_threshold = 50  # 亮度差異閥值

                for line in lines_roi:
                    x1, y1, x2, y2 = map(int, line[:4])

                    # 將ROI區域內的座標轉換回整個圖像的座標
                    x1_global = x1 + int(x_min)
                    x2_global = x2 + int(x_min)
                    y1_global = y1 + int(roi_y_min)
                    y2_global = y2 + int(roi_y_min)

                    # 在 gaussian 上畫線
                    cv2.line(gaussian, (x1_global, y1_global), (x2_global, y2_global), (0, 255, 0), 2)  # 使用綠色畫出線段

                    # 計算線段長度
                    length = np.hypot(x2 - x1, y2 - y1)

                    # 過濾過短或過長的線段
                    if length < min_length or length > max_length:
                        continue  # 跳過該線段

                    # 計算線段的斜率和角度
                    if x2 - x1 == 0:
                        continue  # 忽略垂直線段
                    slope = (y2 - y1) / (x2 - x1 + 1e-6)
                    angle = np.arctan(slope) * 180 / np.pi

                    # 根據斜率和位置過濾線段
                    if -80 < angle < -10 and x1 < width / 2 and x2 < width / 2:
                        side = 'left'
                    elif 10 < angle < 80 and x1 > width / 2 and x2 > width / 2:
                        side = 'right'
                    else:
                        continue  # 不符合左/右車道線條件

                    # 計算線段上的平均亮度 (在 ROI 内的 gaussian)
                    line_brightness = get_line_brightness(gaussian_roi, x1, y1, x2, y2)

                    # 計算線段周圍區域的平均亮度 (在 ROI 内的 gaussian)
                    surrounding_brightness = get_surrounding_brightness(gaussian_roi, x1, y1, x2, y2)

                    # 比較亮度差異
                    brightness_diff = line_brightness - surrounding_brightness

                    # 如果亮度差異大於閥值，認為是虛線的一部份
                    if brightness_diff > brightness_threshold:
                        if side == 'left':
                            left_lines.append((x1, y1_global, x2, y2_global))
                        elif side == 'right':
                            right_lines.append((x1, y1_global, x2, y2_global))


    # 擬合左車道線
    left_fit_fn = None
    if len(left_lines) > 0:
        # 擬合左車道線
        left_fit_fn = fit_lane_lines(left_lines)
        if left_fit_fn is not None:
            # 使用 left_lines 的 y 值範圍來動態設置 y_min 和 y_max
            y_min_left = int(min(min(y1, y2) for (x1, y1, x2, y2) in left_lines))
            y_max_left = int(max(max(y1, y2) for (x1, y1, x2, y2) in left_lines))

            # 根據實際需求調整 y_min_left 和 y_max_left，防止越界
            y_min_left = max(y_min_left, 0)  # 防止超出圖像上邊界
            y_max_left = min(y_max_left + 50, height)  # 防止超出圖像下邊界

            if y_max_left > y_min_left:
                # 根據 y_min_left 和 y_max_left 調整 y_vals 的範圍
                y_vals_left = np.linspace(y_min_left, y_max_left, num=(y_max_left - y_min_left + 1))
                # 使用擬合函數計算對應的 x 值
                x_vals_left = left_fit_fn(y_vals_left)
                # 防止 x 值超出圖像寬度
                x_vals_left = np.clip(x_vals_left, 0, width - 1)
                # 將擬合結果轉換為點並繪製
                pts_left = np.array([np.column_stack((x_vals_left, y_vals_left))], dtype=np.int32)
                cv2.polylines(gaussian, [pts_left], isClosed=False, color=(255, 0, 0), thickness=5)

    # 擬合右車道線
    right_fit_fn = None
    if len(right_lines) > 0:
        right_fit_fn = fit_lane_lines(right_lines)
        if right_fit_fn is not None:
            # 使用 right_lines 的 y 值範圍來動態設置 y_min 和 y_max
            y_min_right = int(min(min(y1, y2) for (x1, y1, x2, y2) in right_lines))
            y_max_right = int(max(max(y1, y2) for (x1, y1, x2, y2) in right_lines))

            # 防止越界
            y_min_right = max(y_min_right, 0)  # 防止超出圖像上邊界
            y_max_right = min(y_max_right, height)  # 防止超出圖像下邊界

            if y_max_right > y_min_right:
                # 調整 y_vals_right 的範圍
                y_vals_right = np.linspace(y_min_right, y_max_right, num=(y_max_right - y_min_right + 1))
                # 使用擬合函數計算對應的 x 值
                x_vals_right = right_fit_fn(y_vals_right)
                # 防止 x 值超出圖像寬度
                x_vals_right = np.clip(x_vals_right, 0, width - 1)
                # 將擬合結果轉換為點並繪製
                pts_right = np.array([np.column_stack((x_vals_right, y_vals_right))], dtype=np.int32)
                cv2.polylines(gaussian, [pts_right], isClosed=False, color=(0, 0, 255), thickness=5)

    # 返回处理后的图像、检测状态、左车道线拟合函数、右车道线拟合函数
    return gaussian, True, left_fit_fn, right_fit_fn

def get_line_brightness(image, x1, y1, x2, y2):
    # 计算线段上的像素坐标
    line_length = int(np.hypot(x2 - x1, y2 - y1))
    x_vals = np.linspace(x1, x2, line_length)
    y_vals = np.linspace(y1, y2, line_length)
    coords = np.vstack((x_vals, y_vals)).astype(np.int32).T

    # 获取线段上的像素值
    brightness_values = image[coords[:, 1], coords[:, 0]]

    # 返回平均亮度
    return np.mean(brightness_values)

def get_surrounding_brightness(image, x1, y1, x2, y2, offset=5):
    # 计算线段方向的垂直向量
    dx = x2 - x1
    dy = y2 - y1
    line_length = np.hypot(dx, dy)
    if line_length == 0:
        return 0
    dx /= line_length
    dy /= line_length

    # 垂直向量
    nx = -dy
    ny = dx

    # 在线段延长线上，偏移一定距离，采样周围区域的亮度
    x1_offset = int(x1 + nx * offset)
    y1_offset = int(y1 + ny * offset)
    x2_offset = int(x2 + nx * offset)
    y2_offset = int(y2 + ny * offset)

    # 确保坐标在图像范围内
    x1_offset = np.clip(x1_offset, 0, image.shape[1] - 1)
    y1_offset = np.clip(y1_offset, 0, image.shape[0] - 1)
    x2_offset = np.clip(x2_offset, 0, image.shape[1] - 1)
    y2_offset = np.clip(y2_offset, 0, image.shape[0] - 1)

    # 计算偏移线段上的像素坐标
    offset_length = int(np.hypot(x2_offset - x1_offset, y2_offset - y1_offset))
    x_vals = np.linspace(x1_offset, x2_offset, offset_length)
    y_vals = np.linspace(y1_offset, y2_offset, offset_length)
    coords = np.vstack((x_vals, y_vals)).astype(np.int32).T

    # 获取周围区域的像素值
    brightness_values = image[coords[:, 1], coords[:, 0]]

    # 返回平均亮度
    return np.mean(brightness_values)

def fit_lane_lines(lines):
    x_coords = []
    y_coords = []
    for x1, y1, x2, y2 in lines:
        x_coords.extend([x1, x2])
        y_coords.extend([y1, y2])

    '''
    if len(x_coords) > 0:  # 至少需要4个点进行B-Spline拟合
        try:
            # 将坐标点转换为数组
            x_coords = np.array(x_coords)
            y_coords = np.array(y_coords)

            # 对控制点进行排序，确保Y坐标递增
            sorted_indices = np.argsort(y_coords)
            x_coords = x_coords[sorted_indices]
            y_coords = y_coords[sorted_indices]

            # 使用B-Spline曲线拟合，s=0表示通过所有点 s设置得更大，拟合曲线会更加平滑
            tck, u = splprep([y_coords, x_coords], s=10)

            # 生成拟合后的B-Spline曲线点
            u_new = np.linspace(0, 1, num=200)

            # 返回拟合函数（用tck作为参数的可调用函数）和拟合的tck参数
            def b_spline_fn(y_vals):
                # 根据y_vals使用splev生成相应的x_vals
                x_vals, _ = splev(np.linspace(0, 1, len(y_vals)), tck)
                return x_vals
            
            return b_spline_fn

        except Exception as e:
            print(f"Error in fitting lane lines with B-Spline: {e}")
            return None
    else:
        return None
    '''

    
    if len(x_coords) > 0:
        try:
            # 使用加权2次多项式拟合
            fit = np.polyfit(y_coords, x_coords, 2)
            fit_fn = np.poly1d(fit)
            return fit_fn
        except Exception as e:
            print(f"Error in fitting lane lines: {e}")
            return None
    else:
        return None
    

    '''
    if len(x_coords) > 3:
        try:
            # 使用三次樣條插值 CubicSpline
            # 对 y_coords 进行递增排序，并同步排序 x_coords
            sorted_indices = np.argsort(y_coords)
            y_coords_sorted = np.array(y_coords)[sorted_indices]
            x_coords_sorted = np.array(x_coords)[sorted_indices]
            
             # 确保 x_coords 递增（移除重复的 x_coords）
            unique_mask = np.diff(x_coords_sorted) > 0  # 找出不同的 x 值
            unique_mask = np.insert(unique_mask, 0, True)  # 保留第一个点
            
            y_coords_filtered = y_coords_sorted[unique_mask]
            x_coords_filtered = x_coords_sorted[unique_mask]
            
            # 使用三次样条插值进行拟合
            cs = CubicSpline(y_coords_filtered, x_coords_filtered)

            # 返回拟合函数（可以直接调用该函数来计算 x_vals）
            def cubic_spline_fn(y_vals):
                return cs(y_vals)
            
            return cubic_spline_fn
        except Exception as e:
            print(f"Error in fitting lane lines: {e}")
            return None
    else:
        return None
    '''
    '''
    if len(x_coords) > 3:
        try:  #PCHIP 拟合
            # 对 y_coords 进行递增排序，并同步排序 x_coords
            sorted_indices = np.argsort(y_coords)
            y_coords_sorted = np.array(y_coords)[sorted_indices]
            x_coords_sorted = np.array(x_coords)[sorted_indices]
            
            # 移除相同的 x_coords 值，确保严格递增
            unique_mask = np.diff(x_coords_sorted) > 0  # 找到递增部分
            unique_mask = np.insert(unique_mask, 0, True)  # 保留第一个点
            
            y_coords_filtered = y_coords_sorted[unique_mask]
            x_coords_filtered = x_coords_sorted[unique_mask]
            
            # 使用 PCHIP 进行插值拟合
            pchip = PchipInterpolator(y_coords_filtered, x_coords_filtered)
            
            # 返回拟合函数
            def pchip_fn(y_vals):
                return pchip(y_vals)
            
            return pchip_fn


        except Exception as e:
            print(f"Error in fitting lane lines: {e}")
            return None
    else:
        return None
    '''





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

        self._window_left = "[TURN] Left ROI"
        self._window_right = "[TURN] Right ROI"
        self._window_curved = "[TURN] Curved Line"
        self._window_curved2 = "[TURN] BIG Curved Line"

        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        #publisher angle 可能不需要
        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_turn/angles", Float32, queue_size=10)

        #publisher 距離左/右側路口的距離
        self.right_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_turn/right_dist", Float32, queue_size=10)
        self.left_inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_turn/left_dist", Float32, queue_size=10)

        #publisher 距離路口虛線的距離
        self.inter_dist_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_turn/inter_dist", Float32, queue_size=10)

        #publisher straight status
        self.straight_status_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node_turn/straight_status", String, queue_size=10)
        
     
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
                            #print(f"Detected right angle at intersection: {intersection}, angle: {angle}")
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
        #print(f"Current state: {self.state}, Turn direction: {self.turn_direction}")

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
        BIG_curved_lane_image, lanes_detected, left_fit_fn, right_fit_fn = BIG_detect_curved_lane(image.copy())
        
        
        
        # Display the processed image
        cv2.namedWindow(self._window_curved, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_curved2, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_left, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._window_right, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self._window_curved, curved_lane_image)
        cv2.imshow(self._window_curved2, BIG_curved_lane_image)
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
        node = CameraReaderNode(node_name='camera_node_turn')
        # keep spinning
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
