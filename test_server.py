import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState, JointTrajectory
from std_msgs.msg import Int32MultiArray
from control_msgs.action import FollowJointTrajectory
import time

class Mairakinematics(Node):

    def __init__(self):
        super().__init__('maira_kinematics')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_position_controller/follow_joint_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        target_pose: Pose = goal_handle.request.target

        self.get_logger().info(f'Received Cartesian goal: {target_pose}')

        success = self.move_joint_to_cartesian(target_pose)

        if success:
            goal_handle.succeed()
            feedback_str = "Motion succeeded"
        else:
            goal_handle.abort()
            feedback_str = "Motion failed"

        result = FollowJointTrajectory.Result()
        result.success = success

        self.get_logger().info(feedback_str)
        return result

    def move_joint_to_cartesian(self, msg: Pose) -> bool:
        self.get_logger().info(f"Executing move to Cartesian position {msg.position} and orientation {msg.orientation}")
        return True

    def move_joint_to_joint(self, msg: JointState) -> bool:
        self.get_logger().info(f"Executing move joint to joint positions: {msg.position}")
        return True

    def move_linear(self, msg: Pose) -> bool:
        self.get_logger().info(f"Executing linear move to position {msg.position} orientation {msg.orientation}")
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
        traj = [pt.positions for pt in msg.points]
        self.get_logger().info(f"Executing joint move via trajectory points: {traj}")
        return True

    def execute(self, msg: Int32MultiArray) -> None:
        ids = list(msg.data)
        res = [True] * len(ids)
        self.get_logger().info(f"Executing tasks with ids {ids}")

    def change_gripper_to(self, end_effector) -> None:
        self.get_logger().info(f"Changing gripper to {end_effector.name}")

    def set_motion_param(self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float) -> None:
        self.speed_move_joint = speed_mj
        self.speed_move_linear = speed_ml
        self.acc_move_joint = acc_mj
        self.acc_move_linear = acc_ml
        self.get_logger().info("Motion parameters set")

    def wait(self, time_s: float) -> None:
        time.sleep(time_s)
        self.get_logger().info(f"Waited for {time_s} seconds")

    def _throw_if_trajectory_invalid(self, trajectory):
        if not isinstance(trajectory, list):
            raise TypeError("[ERROR] trajectory should be a list")
        self.get_logger().info("Trajectory validated")

    def _throw_if_joint_invalid(self, joint_states):
        if not isinstance(joint_states, list):
            raise TypeError("[ERROR] joint_states should be a list")
        self.get_logger().info("Joint states validated")

    def _throw_if_pose_invalid(self, pose):
        if not isinstance(pose, list) or len(pose) != 6:
            raise TypeError("[ERROR] goal_pose should be a list with length 6")
        self.get_logger().info("Pose validated")

    def _throw_if_list_poses_invalid(self, goal_poses):
        if not isinstance(goal_poses, list):
            raise TypeError("[ERROR] goal_pose should be a List[List[float*6]]")
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

    action_server = Mairakinematics()

    rclpy.spin(action_server)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
