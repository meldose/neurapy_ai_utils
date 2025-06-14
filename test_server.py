
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from std_msgs.msg import String, Bool
from typing import List, Tuple


class MairaKinematics(Node):
    def __init__(self):
        super().__init__('maira_kinematics')

        self.declare_parameter('cartesian_pose', [0.0] * 7)


        # Publishers and subscribers
        self.pub_plan_mlp = self.create_publisher(String, 'plan_motion_linear_via_points', 10)
        self.sub_pose_array = self.create_subscription(PoseArray, 'pose_array', self.plan_motion_linear_via_points, 10)

        self.pub_plan_mjv = self.create_publisher(String, 'plan_motion_joint_via_points', 10)
        self.sub_joint_traj = self.create_subscription(JointTrajectory, 'joint_trajectory', self.plan_motion_joint_via_points, 10)

        self.pub_plan_mjj = self.create_publisher(String, 'plan_motion_joint_to_joint', 10)
        self.sub_joint_state = self.create_subscription(JointState, 'joint_state', self.plan_motion_joint_to_joint, 10)

        self.pub_plan_ml = self.create_publisher(String, 'plan_motion_linear', 10)
        self.sub_single_pose = self.create_subscription(Pose, 'pose', self.plan_motion_linear, 10)

        self.pub_mjc_res = self.create_publisher(Bool, 'move_joint_to_cartesian_result', 10)
        self.sub_pose_mjc = self.create_subscription(Pose, 'move_joint_to_cartesian', self.move_joint_to_cartesian, 10)

        # Action server for FollowJointTrajectory
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received new trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing trajectory goal...')
        trajectory = goal_handle.request.trajectory
        points: List[JointTrajectoryPoint] = trajectory.points

        positions_list = [pt.positions for pt in points]
        ok_plan, plan_id, last_idx = self.motion_joint_via_points(positions_list)
        if not ok_plan:
            self.get_logger().error('Failed to plan goal trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # 2) Execute each waypoint, publish feedback, handle cancel
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        for idx, pt in enumerate(points):
            if goal_handle.is_cancel_requested:
                self.group.stop()
                goal_handle.canceled()
                self.get_logger().info(f'Goal canceled at point {idx}')
                return result

            # Send robot to this waypoint
            self.group.set_joint_value_target(pt.positions)
            success = self.group.go(wait=True)
            if not success:
                self.get_logger().error(f'Execution failed at point {idx}')
                goal_handle.abort()
                return result

            # Publish feedback
            feedback.actual.positions = list(self.group.get_current_joint_values())
            feedback.actual.time_from_start = pt.time_from_start
            goal_handle.publish_feedback(feedback)

        # 3) Cartesian post-move if configured
        cart = self.get_parameter('cartesian_pose').value
        cart_success = self._move_joint_to_cartesian([
            cart[0], cart[1], cart[2], cart[3], cart[4], cart[5], 0.0
        ])
        if not cart_success:
            self.get_logger().error('Cartesian post-move failed')
            goal_handle.abort()
            return result

        # 4) Succeed
        goal_handle.succeed()
        self.get_logger().info('Trajectory executed successfully')
        return result

    def plan_motion_linear_via_points(self, msg: PoseArray) -> None:
        poses = [[p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                 for p in msg.poses]
        ok, plan_id, last_index = self.motion_linear_via_points(poses)
        payload = f"{ok},{plan_id},{last_index}"
        self.pub_plan_mlp.publish(String(data=payload))
        self.get_logger().info(f"Published linear-via-points plan: {payload}")

    def motion_linear_via_points(self, poses: List[List[float]]) -> Tuple[bool, int, int]:
        (traj, fraction) = self.group.compute_cartesian_path(
            [p[:3] for p in poses], 0.01, 0.0)
        success = fraction >= 0.99
        return success, id(traj), len(poses) - 1

    def plan_motion_joint_via_points(self, msg: JointTrajectory) -> None:
        ok, plan_id, last_index = self.motion_joint_via_points([pt.positions for pt in msg.points])
        payload = f"{ok},{plan_id},{last_index}"
        self.pub_plan_mjv.publish(String(data=payload))
        self.get_logger().info(f"Published joint-via-points plan: {payload}")

    def motion_joint_via_points(self, traj: List[List[float]]) -> Tuple[bool, int, int]:
        self.group.set_joint_value_target(traj[-1])
        plan = self.group.plan()
        success = bool(plan)
        return success, id(plan), len(traj) - 1

    def plan_motion_joint_to_joint(self, msg: JointState) -> None:
        ok, plan_id, last_index = self.motion_joint_to_joint(list(msg.position))
        payload = f"{ok},{plan_id},{last_index}"
        self.pub_plan_mjj.publish(String(data=payload))
        self.get_logger().info(f"Published joint-to-joint plan: {payload}")

    def motion_joint_to_joint(self, positions: List[float]) -> Tuple[bool, int, int]:
        self.group.set_joint_value_target(positions)
        plan = self.group.plan()
        success = bool(plan)
        return success, id(plan), 0

    def plan_motion_linear(self, msg: Pose) -> None:
        ok, plan_id, last_index = self.motion_linear([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        payload = f"{ok},{plan_id},{last_index}"
        self.pub_plan_ml.publish(String(data=payload))
        self.get_logger().info(f"Published linear-single plan: {payload}")

    def motion_linear(self, pose_list: List[float]) -> Tuple[bool, int, int]:
        target = Pose(
            position=Point(x=pose_list[0], y=pose_list[1], z=pose_list[2]),
            orientation=Quaternion(x=pose_list[3], y=pose_list[4], z=pose_list[5], w=pose_list[6])
        )
        self.group.set_pose_target(target)
        plan = self.group.plan()
        success = bool(plan)
        return success, id(plan), 0

    def move_joint_to_cartesian(self, msg: Pose) -> None:
        res = self._move_joint_to_cartesian([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        self.pub_mjc_res.publish(Bool(data=res))
        self.get_logger().info(f"move_joint_to_cartesian → {res}")

    def _move_joint_to_cartesian(self, pose_list: List[float]) -> bool:
        goal = Pose(
            position=Point(x=pose_list[0], y=pose_list[1], z=pose_list[2]),
            orientation=Quaternion(x=pose_list[3], y=pose_list[4], z=pose_list[5], w=pose_list[6])
        )
        self.group.set_pose_target(goal)
        plan = self.group.plan()
        if not plan:
            return False
        return self.group.go(wait=True)


def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
