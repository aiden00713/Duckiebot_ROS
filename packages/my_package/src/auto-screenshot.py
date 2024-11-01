#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os

# 初始化保存目錄
save_dir = "/data/images"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 設定拍攝間隔
capture_interval = 5.0  # 每5秒拍攝一次

def image_callback(msg):
    # 將壓縮的影像訊息轉換為OpenCV圖像
    np_arr = np.frombuffer(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # 獲取當前時間並生成文件名
    timestamp = rospy.Time.now().to_sec()
    filename = os.path.join(save_dir, f"image_{int(timestamp)}.jpg")
    
    # 保存圖像
    cv2.imwrite(filename, cv_image)
    rospy.loginfo(f"Saved image: {filename}")

def main():
    rospy.init_node("camera_image_capture_node")
    
    # 訂閱Duckiebot相機的壓縮影像話題
    rospy.Subscriber("/duckiebot06/camera_node/image/compressed", CompressedImage, image_callback)
    
    # 使用計時器來定期觸發圖像捕獲
    rospy.Timer(rospy.Duration(capture_interval), lambda event: rospy.loginfo("Capturing image..."))
    
    rospy.loginfo("Camera image capture node started.")
    rospy.spin()

if __name__ == "__main__":
    main()
