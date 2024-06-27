import cv2
import numpy as np
from pylsd2 import LineSegmentDetectionED

def is_dashed_line(points, length_threshold=100, gap_threshold=30):
    """
    判断是否为虚线。
    如果线段之间的间隔大于gap_threshold且线段长度小于length_threshold，则认为是虚线。
    """
    # 计算每个线段的长度
    lengths = np.sqrt(np.sum((points[1::2] - points[::2])**2, axis=1))
    # 计算相邻线段的间隔
    distances = np.sqrt(np.sum(np.diff(points[::2], axis=0)**2, axis=1))
    
    print(f"Line lengths: {lengths}")
    print(f"Line gaps: {distances}")
    
    # 判断是否为虚线
    dashed_lines = []
    for i in range(len(lengths)):
        if lengths[i] < length_threshold and (i == 0 or distances[i-1] > gap_threshold):
            dashed_lines.append((points[2*i], points[2*i+1]))
    
    print(f"Dashed line segments: {dashed_lines}")
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
        
        print(f"Detected points: {all_points}")

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

# 测试代码
image_path = 'E:\\exp_image\\20240313_cut image\\2024-03-13 12 06 23.png'  # 使用正确的图像路径
image = cv2.imread(image_path)
result = detect_curved_lane(image)

if result is not None:
    cv2.imshow('Result', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Failed to detect lanes.")
