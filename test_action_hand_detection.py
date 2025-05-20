import rospy
from neurapy_ai.clients.edge_refiner_client import EdgeRefinerClient
from neurapy_ai.clients.hand_detection_client import HandDetectionClient
from neurapy_ai.utils.return_codes import ReturnCodes

from neurapy_ai_utils.grippers.gripper_interface import DummyGripper
from neurapy_ai_utils.robot.moveit_kinematics import MoveitKinematics
from neurapy_ai_utils.robot.robot import Robot

# calling the main function
if __name__ == "__main__":
    hand_detection_client = HandDetectionClient()
    rospy.init_node("application_run")
    robot = Robot(MoveitKinematics(), DummyGripper())
    # observe_position = [0, 1, 1, 0, 0, 0, 1]
    # drop_position = [0, 0.1, 1, 0, 0, 0, 1]

    observe_position = [0, 0, 0, 1.5, -1.5, 0, 0]

    robot.move_joint_to_joint(observe_position)

    return_code, waypoits = hand_detection_client.detect_hand_poses(2)
    if return_code != ReturnCodes.SUCCESS or len(waypoits) == 0:
        print("Something went bad")
        exit(0)

    print(waypoits)

    edge_detection_client = EdgeRefinerClient()
    [
        return_code,
        point_begining,
        point_end,
    ] = edge_detection_client.refine_line(waypoits[0], waypoits[1], False)
    if return_code != ReturnCodes.SUCCESS or len(waypoits) == 0:
        print("Something went bad")
        exit(0)
    print(point_begining)
    print(point_end)
    robot.move_joint_to_cartesian(point_begining)
    robot.move_linear(point_end)
