import rclpy  # imported rclpy module
from rclpy.node import Node # imported Node module
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported ACtionServerm Goalresponse, CancelResponse module
from geometry_msgs.msg import PoseStamped # imported Posestamped module
from sensor_msgs.msg import JointState # imported JOinstate module
from control_msgs.action import FollowJointTrajectory # imported FollowJointTrajectory module
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# class Mairakinematics
class MairaKinematics:
    def __init__(self):
        self.num_joints = 7
        self.joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

# function cartesain to joint
    def cartesian_to_joint(self, pose: PoseStamped) -> list[float] | None:
        
        return [0.0] * self.num_joints

# class CartesaintoJointActionserver
class CartesianToJointActionServer(Node):
    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing Cartesian→Joint action server')

        self._kinematics = MairaKinematics()

        self._pose_sub = self.create_subscription(
            PoseStamped,    
            '/cmd_pose',       
            self.on_pose_msg,  
            10,               
        )
        # Publish resulting joint states
        self._joint_pub = self.create_publisher(
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


# function  for on pose msg
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

# function for goal callback
    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info('Received FollowJointTrajectory goal request')
        # You can inspect goal_request.trajectory here
        return GoalResponse.ACCEPT

# function for cancel callback
    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT


# function for executing callback
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

# amin function
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
