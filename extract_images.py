#!/usr/bin/env python

#ROS1

import rospy # importerd rospy
from sensor_msgs.msg import Image # imported Image
from cv_bridge import CvBridge # imported cvbridge
import cv2 #imported cv2
import os # imported os moudule

# class for ImageExctractor
class ImageExtractor:
    def __init__(self):
    
        # initliase the node 
        rospy.init_node('image_extractor_node', anonymous=True)

        self.bridge = CvBridge() 
        self.frame_count = 0
        self.output_dir = 'output_images' #settign up the output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribing to RGB image topic
        rospy.Subscriber("/camera/camera/color/image_raw", Image, self.image_callback) # setting up the subscriber

        rospy.loginfo("Image extractor started. Waiting for image messages...")
        rospy.spin()

# function for getting up the image callback
    def image_callback(self, msg: Image):
        try:
            # Convert the ROS image to OpenCV format (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.output_dir, f"frame_{self.frame_count:04d}.jpg")
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved {filename}")
            self.frame_count += 1 #  incrementing up the frame count
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

# calling up the main function 
if __name__ == '__main__':
    try:
        extractor = ImageExtractor() # 
        rospy.spin(extractor)
    except rospy.ROSInterruptException:
        pass
