#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32MultiArray
from control_msgs.action import FollowJointTrajectory


class Mairakinematics(Node):

    def __init__(self):
        super().__init__('maira_kinematics')

        # Single ActionServer for FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_position_controller/follow_joint_trajectory',
            self.execute_callback)

        # default motion parameters
        self.speed_move_joint = 1.0
        self.speed_move_linear = 1.0
        self.acc_move_joint = 1.0
        self.acc_move_linear = 1.0

    def execute_callback(self, goal_handle):
        traj: JointTrajectory = goal_handle.request.trajectory
        n_pts = len(traj.points)
        self.get_logger().info(f"Received trajectory with {n_pts} point(s).")

        # Validate the trajectory
        try:
            self._throw_if_trajectory_invalid(traj.points)
        except TypeError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result().INVALID_GOAL
            result.error_string = str(e)
            return result

        # Execute via the helper
        success = self.move_joint_via_points(traj)

        # Build and return the result
        result = FollowJointTrajectory.Result()
        if success:
            goal_handle.succeed()
            result.error_code = 0
            result.error_string = ""
            self.get_logger().info("Motion succeeded")
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result().PATH_TOLERANCE_VIOLATED
            result.error_string = "Motion failed"
            self.get_logger().info("Motion failed")

        return result

    # ——— Your motion helpers below ———

    def move_joint_to_cartesian(self, msg: Pose) -> bool:
        self.get_logger().info(
            f"Executing move to Cartesian position {msg.position} and orientation {msg.orientation}"
        )
        return True

    def move_joint_to_joint(self, msg: JointState) -> bool:
        self.get_logger().info(
            f"Executing move joint to joint positions: {msg.position}"
        )
        return True

    def move_linear(self, msg: Pose) -> bool:
        self.get_logger().info(
            f"Executing linear move to position {msg.position} orientation {msg.orientation}"
        )
        return True

    def move_linear_via_points(self, msg: PoseArray) -> bool:
        poses = [
            [p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in msg.poses
        ]
        self.get_logger().info(f"Executing linear move via points: {poses}")
        return True

    def move_joint_via_points(self, msg: JointTrajectory) -> bool:
        # extract positions-only from each trajectory point
        traj = [pt.positions for pt in msg.points]
        self.get_logger().info(
            f"Executing joint move via trajectory points: {traj}"
        )
        return True

    def execute(self, msg: Int32MultiArray) -> None:
        ids = list(msg.data)
        self.get_logger().info(f"Executing tasks with ids {ids}")

    def change_gripper_to(self, end_effector) -> None:
        self.get_logger().info(f"Changing gripper to {end_effector.name}")

    def set_motion_param(self,
                         speed_mj: float,
                         speed_ml: float,
                         acc_mj: float,
                         acc_ml: float) -> None:
        self.speed_move_joint = speed_mj
        self.speed_move_linear = speed_ml
        self.acc_move_joint = acc_mj
        self.acc_move_linear = acc_ml
        self.get_logger().info("Motion parameters set")

    def wait(self, time_s: float) -> None:
        time.sleep(time_s)
        self.get_logger().info(f"Waited for {time_s} seconds")

    # ——— Validation helpers ———

    def _throw_if_trajectory_invalid(self, trajectory):
        if not isinstance(trajectory, list):
            raise TypeError("[ERROR] trajectory should be a list")
        if any(not hasattr(pt, 'positions') for pt in trajectory):
            raise TypeError("[ERROR] each trajectory point must have .positions")
        self.get_logger().info("Trajectory validated")

    def _throw_if_joint_invalid(self, joint_states):
        if not isinstance(joint_states, list):
            raise TypeError("[ERROR] joint_states should be a list")
        self.get_logger().info("Joint states validated")

    def _throw_if_pose_invalid(self, pose):
        if not isinstance(pose, list) or len(pose) != 6:
            raise TypeError("[ERROR] goal_pose should be a list of length 6")
        self.get_logger().info("Pose validated")

    def _throw_if_list_poses_invalid(self, goal_poses):
        if not isinstance(goal_poses, list):
            raise TypeError("[ERROR] goal_poses should be a List[List[float]]")
        self.get_logger().info("List of poses validated")

    def _speed_to_percent(self, speed_mps):
        return 50

    def _acc_to_percent(self, acc):
        return 50

    def get_current_joint_state(self):
        return [0.0] * 6

    def get_current_cartesian_tcp_pose(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def finish(self) -> None:
        self.get_logger().info("Finished execution")


def main(args=None):
    rclpy.init(args=args)
    node = Mairakinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
