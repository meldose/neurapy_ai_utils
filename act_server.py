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

# class for Mairakinematics
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
            return [0.0] * self.num_joints  # Dummy IK solution
        except Exception as e:
            print(f"IK computation failed: {e}")
            return None

# class Cartesina to joint Action server 
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

# created ActionServer 
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

# callback for fucntion cartesian to joint conversion
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

# function for joint states
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

# callback functions 

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        self.get_logger().info('Received FollowJointTrajectory goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        self.get_logger().info('Executing Cartesian trajectory via IK')

        traj = goal_handle.request.trajectory
        n = self._kin.num_joints

        if not traj.points:
            self.get_logger().error('Received trajectory with no points.')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        joint_trajectory_points = []
        for idx, pt in enumerate(traj.points):
            if len(pt.positions) != 7:
                self.get_logger().error(f'Point #{idx} is not a valid Cartesian pose (expected 7 values)')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            # Convert Cartesian to PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = pt.positions[0]
            pose_msg.pose.position.y = pt.positions[1]
            pose_msg.pose.position.z = pt.positions[2]
            pose_msg.pose.orientation.x = pt.positions[3]
            pose_msg.pose.orientation.y = pt.positions[4]
            pose_msg.pose.orientation.z = pt.positions[5]
            pose_msg.pose.orientation.w = pt.positions[6]

            joint_positions = self._kin.cartesian_to_joint(pose_msg)
            if joint_positions is None:
                self.get_logger().error(f'IK failed for point #{idx}')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            joint_point = JointTrajectoryPoint()
            joint_point.positions = joint_positions
            joint_point.time_from_start = pt.time_from_start
            joint_trajectory_points.append(joint_point)

        # Simulated execution using time.sleep
        prev_t = 0.0
        for idx, pt in enumerate(joint_trajectory_points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            if dt > 0.0:
                time.sleep(dt)
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self._kin.joint_names
            feedback.desired = pt
            feedback.actual = pt
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)
            prev_t = t
            self.get_logger().debug(f'Executed IK for point #{idx}')

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        self.get_logger().info('IK-based trajectory execution completed successfully')
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

# calling the main function 
if __name__ == '__main__':
    main()
