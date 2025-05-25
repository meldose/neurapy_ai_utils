import rospy
from geometry_msgs.msg import PointStamped
from neurapy_ai.clients.edge_refiner_client import EdgeRefinerClient
from neurapy_ai.clients.hand_detection_client import HandDetectionClient
from neurapy_ai.return_codes import ReturnCodes

from neurapy_ai_utils.functions import utils
from neurapy_ai_utils.grippers.gripper_interface import DummyGripper
from neurapy_ai_utils.robot.moveit_kinematics import MoveitKinematics
from neurapy_ai_utils.robot.robot import Robot

# created an fuction for getting the rviz point 

def get_rviz_point():
    msg = rospy.wait_for_message("/clicked_point", PointStamped) # created and message 
    return msg.point

# calling the main function

if __name__ == "__main__":
    hand_detection_client = HandDetectionClient() # defining the hand detection client 
    rospy.init_node("application_run") # initialise the node 
    robot = Robot(MoveitKinematics(), DummyGripper())
    # observe_position = [0, 1, 1, 0, 0, 0, 1]
    # drop_position = [0, 0.1, 1, 0, 0, 0, 1]

    observe_position = [0, 0, 0, 1.5, -1.5, 0, 0] # default observe position

    robot.move_joint_to_joint(observe_position) # robot moving to the observer position


    waypoints = [] # creating an waypoints 
    point = get_rviz_point() # settiing up the point
    waypoints.append([point.x, point.y, point.z, 0, 0, 0, 1]) # appendignt he waypoint wiht point (x,y,z)
    point = get_rviz_point() # setting up the rviz point
    waypoints.append([point.x, point.y, point.z, 0, 0, 0, 1])
    edge_detection_client = EdgeRefinerClient() # defning the edge detection client
    [
        return_code,
        point_begining,
        point_end,
    ] = edge_detection_client.refine_line(waypoints[0], waypoints[1], False)

    # if the return code is not eual to the Returncodes.SUCCESS
    if return_code != ReturnCodes.SUCCESS: 
        print("Something went bad")
        exit(0) # exit the window 

    pre_beg = utils.get_pose_with_offset(point_begining, [0, 0, -0.1])
    post_end = utils.get_pose_with_offset(point_end, [0, 0, -0.1])
    robot.move_joint_to_cartesian(pre_beg)

    # robot moving linearly to the defied positions 
    robot.move_linear(point_begining)
    robot.move_linear(point_end)
    robot.move_linear(post_end)
