import rospy # imported rospy module
from neurapy_ai.clients.edge_refiner_client import EdgeRefinerClient
from neurapy_ai.clients.hand_detection_client import HandDetectionClient
from neurapy_ai.utils.return_codes import ReturnCodes

from neurapy_ai_utils.grippers.gripper_interface import DummyGripper
from neurapy_ai_utils.robot.moveit_kinematics import MoveitKinematics
from neurapy_ai_utils.robot.robot import Robot

# calling the main function
if __name__ == "__main__":
    hand_detection_client = HandDetectionClient() # setting up the hand detection client 
    rospy.init_node("application_run") # initilase the node 
    robot = Robot(MoveitKinematics(), DummyGripper())

    observe_position = [0, 0, 0, 1.5, -1.5, 0, 0] # setting up the observe position

    robot.move_joint_to_joint(observe_position) # moving robot to the observe position

    return_code, waypoints = hand_detection_client.detect_hand_poses(2) # gets two sets of hand poses
    
    # if the return code is not success
    if return_code != ReturnCodes.SUCCESS or len(waypoints) == 0:
        print("Something went bad")
        exit(0)

    print(waypoints)

    edge_detection_client = EdgeRefinerClient() # setting up the EdgeRefinerClient 
    [
        return_code,
        point_begining,
        point_end,
    ] = edge_detection_client.refine_line(waypoints[0], waypoints[1], False)

# if the return code is not success
    if return_code != ReturnCodes.SUCCESS or len(waypoints) == 0:
        print("Something went bad") # not success
        exit(0) # exit the window 
    print(point_begining) 
    print(point_end)
    robot.move_joint_to_cartesian(point_begining)  # robot moving to the point begining 
    robot.move_linear(point_end) # robot moving to the point end 
