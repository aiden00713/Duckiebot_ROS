#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from cv_bridge import CvBridge
from pylsd2 import LineSegmentDetectionED
import time
import matplotlib.pyplot as plt


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
        self.fps_data = []  # Store FPS data
        self.time_data = []  # Store time data
        self.duration = 30  # Record for 30 seconds
        # create window
        self._window = "Processed Image"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # Convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Start timing for performance tracking
        if self.start_time is None:
            self.start_time = time.time()
        
        # Process the image
        processed_image = self.process_image_with_blur(image)
        
        # Display the processed frame
        cv2.imshow(self._window, processed_image)
        cv2.waitKey(1)
        
        # Increment frame count and calculate FPS
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            fps = self.frame_count / elapsed_time
            rospy.loginfo(f"FPS: {fps:.2f}")
            
            # Record FPS and time
            self.fps_data.append(fps)
            self.time_data.append(elapsed_time)

        # Stop recording after 30 seconds
        if elapsed_time > self.duration:
            rospy.signal_shutdown("Finished 30 seconds of recording.")
            self.plot_fps()

    def process_image_with_blur(self, image):
        """
        Apply Gaussian blur and perform line detection
        """
        height, width = image.shape[:2]
        red_channel = image[height // 2:, :, 2]  # Keep lower half red channel

        # Apply Gaussian blur
        blur_param = ((5, 5), 1)
        blurred_red_channel = cv2.GaussianBlur(red_channel, blur_param[0], blur_param[1])

        # Perform line detection
        min_line_len = 20
        line_fit_err_thres = 1.4
        lines = LineSegmentDetectionED(blurred_red_channel, min_line_len=min_line_len, line_fit_err_thres=line_fit_err_thres)

        # Draw lines on the image
        output_image = cv2.cvtColor(blurred_red_channel, cv2.COLOR_GRAY2BGR)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = map(int, line[:4])
                cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), 1)

        return output_image

    def plot_fps(self):
        """
        Plot FPS data as a line graph
        """
        plt.figure(figsize=(10, 5))
        plt.plot(self.time_data, self.fps_data, marker='o', label="FPS")
        plt.xlabel("Time (s)")
        plt.ylabel("FPS")
        plt.title("FPS Over Time (0-30 seconds)_Gaussian")
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()