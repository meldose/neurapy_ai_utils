import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class MairaKinematics:
    def __init__(self):
        self.num_joints = 6

    def cartesian_to_joint(self, pose: PoseStamped) -> list[float] | None:
        """
        Convert a Cartesian pose to joint angles.
        TODO: implement real inverse kinematics here.
        """
        # Dummy implementation: all zeros
        return [0.0] * self.num_joints


class CartesianToJointActionServer(Node):
    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing Cartesian→Joint action server')

        self._kinematics = MairaKinematics()

        # Listen for raw Cartesian pose goals
        self._pose_sub = self.create_publisher(
            PoseStamped,
            '/cmd_pose',
            self.on_pose_msg,
            10,
        )

        # Publish resulting joint states
        self._joint_pub = self.create_subscription(
            JointState,
            '/joint_positions',
            10,
        )

        # FollowJointTrajectory Action server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def on_pose_msg(self, msg: PoseStamped) -> None:
        """
        Handle incoming PoseStamped messages, convert to joint angles, and publish.
        """
        self.get_logger().info('Received PoseStamped')
        joint_positions = self._kinematics.cartesian_to_joint(msg)
        if joint_positions is None:
            self.get_logger().error('IK failed: could not compute joint positions')
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'joint{i+1}' for i in range(self._kinematics.num_joints)]
        js.position = joint_positions
        self._joint_pub.publish(js)
        self.get_logger().info(f'Published joint positions: {joint_positions}')

    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info('Received FollowJointTrajectory goal request')
        # You can inspect goal_request.trajectory here
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        self.get_logger().info('Executing trajectory')
        trajectory = goal_handle.request.trajectory

        # Validate joint count
        if len(trajectory.joint_names) != self._kinematics.num_joints:
            self.get_logger().error(
                f'Expected {self._kinematics.num_joints} joints, got {len(trajectory.joint_names)}'
            )
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        # Iterate through trajectory points
        for idx, point in enumerate(trajectory.points):
            # Here, you'd send each point to your robot controller
            # e.g., self._joint_pub.publish(JointState(...))
            if len(point.positions) != self._kinematics.num_joints:
                self.get_logger().error(
                    f'Point #{idx} has wrong position length ({len(point.positions)})'
                )
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = trajectory.joint_names
            js.position = list(point.positions)
            self._joint_pub.publish(js)
            self.get_logger().info(f'Point #{idx} → positions: {point.positions}')

            # Optionally add delays based on point.time_from_start
            # rclpy.sleep(point.time_from_start.to_sec())

        # Succeeded
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
