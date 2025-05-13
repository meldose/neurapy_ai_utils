import rospy
from neurapy_ai_utils.robot.moveit_kinematics import MoveitKinematics
# from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
import math


if __name__ == "__main__":

    rospy.init_node("test_moveit")
    # choose Kinematics Interface, 1. MoveItKinematics, 2. Mairakinemaatics
    robot = MoveitKinematics()
    # robot = MairaKinematics()

    goal_pose = [0.0, 1.0, 0.5, -math.pi, 0.0, -math.pi]
    joint_pose = [
        100 * math.pi / 180,
        25 * math.pi / 180,
        160 * math.pi / 180,
        -75 * math.pi / 180,
        9 * math.pi / 180,
        -78 * math.pi / 180,
        -96 * math.pi / 180,
    ]

    joint_pose1 = [
        103 * math.pi / 180,
        25 * math.pi / 180,
        160 * math.pi / 180,
        -75 * math.pi / 180,
        9 * math.pi / 180,
        -78 * math.pi / 180,
        -96 * math.pi / 180,
    ]
    joint_pose2 = [
        24 * math.pi / 180,
        33 * math.pi / 180,
        -11 * math.pi / 180,
        48 * math.pi / 180,
        -1 * math.pi / 180,
        99 * math.pi / 180,
        6 * math.pi / 180,
    ]
    joint_poses = [joint_pose1, joint_pose2]

    # --------- test moving to a goal ------------
    print("Moving to cartesian goal...")
    robot.move_joint_to_cartesian(goal_pose)
    print("Moving to joint goal...")
    robot.move_joint_to_joint(joint_pose)
    robot.wait(time_s=5)
    print("Moving to joint goals...")
    robot.move_joint_via_points(joint_poses)
    print("Moving to joint goals...")

    # --------- test get curren status ------------
    print(robot.get_current_cartesian_tcp_pose())
    print(robot.get_current_joint_state())

    # --------- test planning and executing ------------
    print("Plan and execute to cartesian goal ...")
    success, id, joint_states = robot.plan_motion_joint_to_cartesian(
        goal_pose, speed=1, acc=1, reusable=True
    )
    robot.execute([id], True)
    print("Plan and execute to joint goal ...")
    success, id, joint_states = robot.plan_motion_joint_to_joint(
        joint_pose, speed=10, reusable=True
    )
    robot.execute([id], True)
    print("Plan and execute linear movement ...")
    success, id, joint_states = robot.plan_motion_linear(
        goal_pose, reusable=True
    )
    robot.execute([id], True)
    print("Clearing ids...")
    robot.clear_ids([id])

    print("DONE!")
