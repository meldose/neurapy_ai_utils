import rclpy  # imported rclpy module 
from rclpy.node import Node # imported Node module 
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported Actionserver, CancelResponse, Goalresponse module 
from geometry_msgs.msg import PoseStamped # imported Posestamped module 
from sensor_msgs.msg import JointState # imported Jointstate module 
from control_msgs.action import FollowJointTrajectory # imported FollowjointTrajectory module 

# class Mairakinematics
class MairaKinematics:
    """
    Simple kinematics stub: convert Cartesian pose to joint angles.
    """
    def __init__(self):
        self.num_joints = 7
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]

# function for cartesain to joint 
    def cartesian_to_joint(self, pose: PoseStamped) -> list[float] | None:
        # TODO: implement real IK. Here we return zeros as placeholder.
        return [0.0] * self.num_joints

# class CartesiantojointActionServer
class CartesianToJointActionServer(Node):
    """
    Action server that accepts FollowJointTrajectory goals,
    converts Cartesian commands to joint states, and publishes JointState messages.
    Also listens for direct Cartesian PoseStamped messages and JointState updates.
    """
    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing Cartesian→Joint action server')

        # Kinematics solver
        self._kinematics = MairaKinematics()


# # Publisher for joint states generated from IK
#         self._joint_pub = self.create_publisher(
#             PoseStamped,
#             '/cmd_pose',
#             10,
#         )


# Subscribe to joint states
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_state,
            10,
        )


# FollowJointTrajectory action server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

# function for on pose msg
    def on_pose_msg(self, msg: PoseStamped) -> None:
        """
        Handle incoming Cartesian PoseStamped messages,
        convert to joint angles, and publish JointState.
        """
        self.get_logger().info('Received Cartesian pose')
        joint_positions = self._kinematics.cartesian_to_joint(msg)
        if joint_positions is None:
            self.get_logger().error('IK failed: could not compute joint positions')
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self._kinematics.joint_names
        js.position = joint_positions
        self._joint_pub.publish(js)
        self.get_logger().info(f'Published IK joint positions: {joint_positions}')

# function for join state
    def on_joint_state(self, msg: JointState) -> None:
        """
        Handle incoming JointState messages by logging or processing.
        """
        self.get_logger().info(f'Received joint states: {dict(zip(msg.name, msg.position))}')

# function for goal callback 
    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        self.get_logger().info('Received FollowJointTrajectory goal request')
        return GoalResponse.ACCEPT

# function for cancel callback 
    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

# function executing callback 
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

        # Succeeded
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        self.get_logger().info('Trajectory execution completed successfully')
        return result

# main function
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

# calling main function 
if __name__ == '__main__':
    main()
