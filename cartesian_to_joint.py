
# CARTESIAN TO JOINT

import rclpy  # imported rclpy module 
from rclpy.node import Node # imported Node module 
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported CancelResponse and GoalResponse module 
from geometry_msgs.msg import PoseStamped # imported Posestamped module 
from sensor_msgs.msg import JointState # imported Joinstate module 
from std_msgs.msg import Bool 
from control_msgs.action import FollowJointTrajectory  
from trajectory_msgs.msg import JointTrajectoryPoint 
from typing import List, Optional 
import time # imported time module

# class Mairakinematic
class MairaKinematics:

    def __init__(self):
        self.num_joints = 7
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
        self._current_joint_state: Optional[List[float]] = None

    def cartesian_to_joint(self, pose_msg: PoseStamped) -> Optional[List[float]]:
                
        """Get the inverse kinematics for a given pose and current joint state

        Parameters
        ----------
        goal_pose_cartesian : List[float]
            Cartesian pose of tcp frame
        reference_joint_states : Optional[List[float]], optional
            Joint positions to seed IK solver. Default is None, to use current
            joint positions.

        Returns
        -------
        List[float]
            Joint positions from IK solution.

        Raises
        ------
        TypeError
            Wrong input type

        ValueError
            IK solver failed

        """

        pos = pose_msg.pose.position
        ori = pose_msg.pose.orientation
        goal = [pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w]

class CartesianToJointActionServer(Node):

    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing Cartesian→Joint action server')

        # Kinematics solver
        self._kin = MairaKinematics()

        # Publisher for IK-generated joint states
        self._joint_pub = self.create_publisher(JointState, '/ik_joint_states', 10)

        # Publisher for joint-state-received flag
        self._flag_pub = self.create_publisher(Bool, '/joint_state_received_flag', 10)

        # Subscriber for Cartesian pose commands
        self._pose_sub = self.create_subscription(
            PoseStamped,
            '/cmd_pose',
            self.on_pose_msg,
            10,
        )

        # Subscriber for robot’s actual joint states with sensor QoS
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state,
            10,
        )
        self._latest_state: Optional[JointState] = None

        # FollowJointTrajectory action server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
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

        # Validate joint names and points
        if len(traj.joint_names) != n:
            self.get_logger().error(f'Expected {n} joints, got {len(traj.joint_names)}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        for idx, pt in enumerate(traj.points):
            if len(pt.positions) != n:
                self.get_logger().error(f'Point #{idx} wrong size: {len(pt.positions)}')
                goal_handle.abort()
                res = FollowJointTrajectory.Result()
                res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return res

        # Simulate execution with feedback
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

        # Finished
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
