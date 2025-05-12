import rospy
from neurapy_ai.clients.marker_detection_client import MarkerDetectionClient
from neurapy_ai.return_codes import ReturnCodes

from neurapy_ai_utils.grippers.gripper_interface import DummyGripper
from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
# from neurapy_ai_utils.robot.moveit_kinematics import MoveitKinematics
from neurapy_ai_utils.robot.robot import Robot

if __name__ == "__main__":
    marker_detection_client = MarkerDetectionClient()
    rospy.init_node("application_run")
    # robot = Robot(MoveitKinematics(), DummyGripper())
    robot = Robot(MairaKinematics(), DummyGripper())
    observe_position = [0, 0, 0, 1.5, -1.5, 0, 0]
    drop_position = observe_position

    robot.move_joint_to_joint(observe_position)
    return_code, tags, ids = marker_detection_client.getMarkers(10)

    if return_code != ReturnCodes.SUCCESS or len(tags) == 0:
        print("Something went bad")
        exit(0)

    print("---- Picking the object ---- ")
    # robot.actions.pick([0.2, 0.2, 1, 0,0,0], -0.1, -0.1)
    robot.actions.pick(tags[0], 0.1, -0.1)

    print("---- Droping the object ---- ")

    robot.move_joint_to_joint(drop_position)
    robot.gripper.open()
