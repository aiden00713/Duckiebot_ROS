#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np
from pylsd2 import LineSegmentDetectionED
import time

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # performance tracking
        self.start_time = None
        self.frame_count = 0
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Start timing for performance
        if self.start_time is None:
            self.start_time = time.time()
        
        # Process the image
        processed_image = self.process_image(image)
        
        # Display the processed frame
        cv2.imshow(self._window, processed_image)
        cv2.waitKey(1)
        
        # Increment frame count and calculate FPS
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            fps = self.frame_count / elapsed_time
            rospy.loginfo(f"FPS: {fps:.2f}")

    def process_image(self, image):
        # Extract the lower half of the image
        height, width, _ = image.shape
        cropped_image = image[height // 2:, :]  # Only keep the lower half
        
        # Extract the red channel
        red_channel = cropped_image[:, :, 2]
        
        # Line detection with pylsd2
        min_line_len = 20
        line_fit_err_thres = 1.4
        lines = LineSegmentDetectionED(red_channel, min_line_len=min_line_len, line_fit_err_thres=line_fit_err_thres)
        
        # Filter lines by intensity
        min_intensity = 90
        max_intensity = 255
        filtered_lines = self.filter_lines_by_intensity(red_channel, lines, min_intensity, max_intensity)
        
        # Draw filtered lines on the image
        result_image = cv2.cvtColor(red_channel, cv2.COLOR_GRAY2BGR)
        for line, intensity in filtered_lines:
            x1, y1, x2, y2 = map(int, line)
            cv2.line(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green lines
        return result_image

    def filter_lines_by_intensity(self, image, lines, min_intensity, max_intensity):
        filtered_lines = []
        for line in lines:
            x1, y1, x2, y2 = map(int, line)
            mask = np.zeros_like(image, dtype=np.uint8)
            cv2.line(mask, (x1, y1), (x2, y2), 255, 1)
            mean_intensity = cv2.mean(image, mask)[0]
            if min_intensity <= mean_intensity <= max_intensity:
                filtered_lines.append((line, mean_intensity))
        return filtered_lines

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()