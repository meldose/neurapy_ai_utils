#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageExtractor:
    def __init__(self):
        rospy.init_node('image_extractor_node', anonymous=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.output_dir = 'output_images'
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribing to RGB image topic
        rospy.Subscriber('/camera/camera/color/image_raw', Image, self.image_callback)

        rospy.loginfo("Image extractor started. Waiting for image messages...")
        rospy.spin()

    def image_callback(self, msg):
        try:
            # Convert the ROS image to OpenCV format (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.output_dir, f"frame_{self.frame_count:04d}.jpg")
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved {filename}")
            self.frame_count += 1
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

if __name__ == '__main__':
    try:
        extractor = ImageExtractor()
    except rospy.ROSInterruptException:
        pass
