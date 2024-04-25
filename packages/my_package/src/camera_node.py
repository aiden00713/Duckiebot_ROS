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
        
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        self.angle_pub = rospy.Publisher(f"/{self._vehicle_name}/camera_node/angles", Float32, queue_size=10)


    def callback(self, msg):

        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # Process the image
        processed_image = self.process_image(image)
        
        processed_image_grid = draw_grid(processed_image)
        # Display the processed image
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        #cv2.imshow(self._window, processed_image)
        cv2.imshow(self._window, processed_image_grid)
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
