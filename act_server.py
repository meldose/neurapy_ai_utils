import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from typing import List, Optional
import time


class MairaKinematics:

    def __init__(self):
        self.num_joints = 7
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
        self._current_joint_state: Optional[List[float]] = None

    def _get_current_joint_state(self) -> List[float]:
        if self._current_joint_state is None:
            raise ValueError("Current joint state not available")
        return self._current_joint_state

    def cartesian_to_joint(self, pose_msg: PoseStamped) -> Optional[List[float]]:
        """
        Replace this method's logic with your IK solver.
        This method receives a PoseStamped and returns joint angles.
        """
        pos = pose_msg.pose.position
        ori = pose_msg.pose.orientation
        goal = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
        try:
            # Replace this with your actual IK logic
            # For now, return dummy values as placeholder
            return [0.0] * self.num_joints
        except Exception as e:
            print(f"IK computation failed: {e}")
            return None


class CartesianToJointActionServer(Node):

    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing Cartesian→Joint action server')

        self._kin = MairaKinematics()
        self._joint_pub = self.create_publisher(JointState, '/ik_joint_states', 10)
        self._flag_pub = self.create_publisher(Bool, '/joint_state_received_flag', 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, '/cmd_pose', self.on_pose_msg, 10)

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state, 10)
        self._latest_state: Optional[JointState] = None

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def on_pose_msg(self, msg: PoseStamped) -> None:
        self.get_logger().info('Received Cartesian pose')
        joint_positions = self._kin.cartesian_to_joint(msg)
        if joint_positions is None:
            self.get_logger().error('IK failed: could not compute joint positions')
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base_link'
        js.name = self._kin.joint_names
        js.position = joint_positions
        self._joint_pub.publish(js)
        self.get_logger().info(f'Published IK joint positions: {joint_positions}')

    def joint_state(self, msg: JointState) -> None:
        if len(msg.position) != self._kin.num_joints:
            self.get_logger().warn('Received joint state of unexpected size.')
            return

        self._latest_state = msg
        self._kin._current_joint_state = list(msg.position)
        pos_str = ', '.join(f'{n}={p:.3f}' for n, p in zip(msg.name, msg.position))
        self.get_logger().debug(f'Received joint states → {pos_str}')

        flag = Bool()
        flag.data = True
        self._flag_pub.publish(flag)

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        self.get_logger().info('Received FollowJointTrajectory goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        self.get_logger().info('Executing trajectory')
        traj = goal_handle.request.trajectory
        n = self._kin.num_joints

        if len(traj.joint_names) != n:
            self.get_logger().error(f'Expected {n} joints, got {len(traj.joint_names)}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        if not traj.points:
            self.get_logger().error('Received trajectory with no points.')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        for idx, pt in enumerate(traj.points):
            if len(pt.positions) != n:
                self.get_logger().error(f'Point #{idx} wrong size: {len(pt.positions)}')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

        prev_t = 0.0
        for idx, pt in enumerate(traj.points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            if dt > 0.0:
                time.sleep(dt)
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = traj.joint_names
            feedback.desired = pt
            feedback.actual = pt
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)
            self.get_logger().debug(f'Sent feedback for point {idx}')
            prev_t = t

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        self.get_logger().info('Trajectory execution completed successfully')
        return result


def main(args=None):
    rclpy.init(args=args)
    server = CartesianToJointActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


