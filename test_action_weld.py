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
    msg = rospy.wait_for_message("/clicked_point", PointStamped)
    return msg.point

# calling the main function

if __name__ == "__main__":
    hand_detection_client = HandDetectionClient()
    rospy.init_node("application_run")
    robot = Robot(MoveitKinematics(), DummyGripper())
    # observe_position = [0, 1, 1, 0, 0, 0, 1]
    # drop_position = [0, 0.1, 1, 0, 0, 0, 1]

    observe_position = [0, 0, 0, 1.5, -1.5, 0, 0]

    robot.move_joint_to_joint(observe_position)

    # return_code, waypoits = hand_detection_client.detect_hand_poses(2)
    # if return_code != ReturnCodes.SUCCESS or len(waypoits) == 0:
    #     print("Something went bad")
    #     exit(0)

    # print(waypoits)
    waypoints = []
    point = get_rviz_point()
    waypoints.append([point.x, point.y, point.z, 0, 0, 0, 1])
    point = get_rviz_point()
    waypoints.append([point.x, point.y, point.z, 0, 0, 0, 1])
    edge_detection_client = EdgeRefinerClient()
    [
        return_code,
        point_begining,
        point_end,
    ] = edge_detection_client.refine_line(waypoints[0], waypoints[1], False)
    if return_code != ReturnCodes.SUCCESS:
        print("Something went bad")
        exit(0)

    pre_beg = utils.get_pose_with_offset(point_begining, [0, 0, -0.1])
    post_end = utils.get_pose_with_offset(point_end, [0, 0, -0.1])
    robot.move_joint_to_cartesian(pre_beg)
    robot.move_linear(point_begining)
    robot.move_linear(point_end)
    robot.move_linear(post_end)
