import rospy # importing rospy module 

from sensor_msgs.msg import Image # importing sensor msg

from cv_bridge import CvBridge #importing cv_bridge

import cv2  # importing cv2 module 


# creating dummy instance segmentation
def dummy_instance_segmentation(cv_image):
    h,w=cv_image.shape[:2]
    
    mask=cv2.rectangle(cv_image.copy(),(w//4,h//4),(3*w//4),(3*h//4),(0,255,0),3)
    return mask

# class for InstanceSegNode
class InstanceSegNode:
    def __init__(self):
        
        # initializing node
        rospy.init_node("instance_segmentation_node")
        self.bridge=CvBridge()
        
        rospy.Subscriber("/camera/image_raw",Image,self.callback) # creating subscriber 
        
        self.pub=rospy.Publisher("/instance_segementation",Image,queue_size=1) # creating publisher 
        
        rospy.loginfo("Instance segmentation node started")
        
    
# creating fucntion for callback 
    def callback(self,msg:Image):
        
        try:
            cv_image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            segmented=dummy_instance_segmentation(cv_image)
            
            self.pub.publish(self.bridge.cv2_to_imgmsg(segmented,"bgr8"))
            
        except Exception as e:
            rospy.logerr("Error in callback: "+str(e))

# calling main function     
if __name__=="__main__":
    InstanceSegNode()
    rospy.spin()
