import rospy #imported rospy
from audio_common_msgs.msg import AudioData # importing AudioData
from sensor_msgs.msg import CameraInfo, JointState # importing the cameraInfo and Jointstate


# calling the main function
if __name__ == "__main__":
    rospy.init_node("test_all_alive")

# creatig different services
    services = [
        "/neura_edge_refiner/refine_line",
        "/neura_marker_detection/detect_charuco",
        "/neura_marker_detection/detect_aruco",
        "/pose_estimation/estimate_poses",
        "/instance_segmentation/segment_instances",
        "/neura_motion_planning_server/plan",
    ]

# created different rosnode topics 

    topics = {
        "/camera/color/camera_info": CameraInfo, # setting the cameraInfo
        "/joint_states": JointState, # setting the Jointstates
        "/audio": AudioData, # setting the AudioData
    }

    for service in services:
        try:
            rospy.wait_for_service(service, 1)
        except:
            rospy.logerr("Service not available: %s ", service)
    for topic, topic_type in topics.items():
        try:
            rospy.wait_for_message(topic, topic_type, 1)
        except:
            rospy.logerr("Topic not available: %s ", topic)
