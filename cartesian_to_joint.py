
# import rclpy  # imported rclpy module 
# from rclpy.node import Node # imported Node module 
# from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported CancelResponse and GoalResponse module 
# from geometry_msgs.msg import PoseStamped # imported Posestamped module 
# from sensor_msgs.msg import JointState # imported Joinstate module 
# from std_msgs.msg import Bool 
# from control_msgs.action import FollowJointTrajectory  
# from trajectory_msgs.msg import JointTrajectoryPoint 
# from typing import List, Optional y
# import time # imported time module

# # class Mairakinematic
# class MairaKinematics:

#     def __init__(self):
#         self.num_joints = 7
#         self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
#         self._current_joint_state: Optional[List[float]] = None

#     def cartesian_to_joint(self, pose_msg: PoseStamped) -> Optional[List[float]]:
                
#         """Get the inverse kinematics for a given pose and current joint state

#         Parameters
#         ----------
#         goal_pose_cartesian : List[float]
#             Cartesian pose of tcp frame
#         reference_joint_states : Optional[List[float]], optional
#             Joint positions to seed IK solver. Default is None, to use current
#             joint positions.

#         Returns
#         -------
#         List[float]
#             Joint positions from IK solution.

#         Raises
#         ------
#         TypeError
#             Wrong input type

#         ValueError
#             IK solver failed

#         """

#         pos = pose_msg.pose.position
#         ori = pose_msg.pose.orientation
#         goal = [pos.x, pos.y, pos.z,
#                 ori.x, ori.y, ori.z, ori.w]

# class CartesianToJointActionServer(Node):

#     def __init__(self):
#         super().__init__('cartesian_to_joint_action_server')
#         self.get_logger().info('Initializing Cartesian→Joint action server')

#         # Kinematics solver
#         self._kin = MairaKinematics()

#         # Publisher for IK-generated joint states
#         self._joint_pub = self.create_publisher(JointState, '/ik_joint_states', 10)

#         # Publisher for joint-state-received flag
#         self._flag_pub = self.create_publisher(Bool, '/joint_state_received_flag', 10)

#         # Subscriber for Cartesian pose commands
#         self._pose_sub = self.create_subscription(
#             PoseStamped,
#             '/cmd_pose',
#             self.on_pose_msg,
#             10,
#         )

#         # Subscriber for robot’s actual joint states with sensor QoS
#         self._joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state,
#             10,
#         )
#         self._latest_state: Optional[JointState] = None

#         # FollowJointTrajectory action server
#         self._action_server = ActionServer(
#             node=self,
#             action_type=FollowJointTrajectory,
#             action_name='joint_trajectory_position_controller/follow_joint_trajectory',
#             execute_callback=self.execute_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#         )

#     def on_pose_msg(self, msg: PoseStamped) -> None:
#         self.get_logger().info('Received Cartesian pose')
#         joint_positions = self._kin.cartesian_to_joint(msg)
#         if joint_positions is None:
#             self.get_logger().error('IK failed: could not compute joint positions')
#             return

#         js = JointState()
#         js.header.stamp = self.get_clock().now().to_msg()
#         js.header.frame_id = 'base_link'
#         js.name = self._kin.joint_names
#         js.position = joint_positions
#         self._joint_pub.publish(js)
#         self.get_logger().info(f'Published IK joint positions: {joint_positions}')

#     def joint_state(self, msg: JointState) -> None:
#         self._latest_state = msg
#         self._kin._current_joint_state = list(msg.position)
#         pos_str = ', '.join(f'{n}={p:.3f}' for n, p in zip(msg.name, msg.position))
#         self.get_logger().debug(f'Received joint states → {pos_str}')

#         flag = Bool()
#         flag.data = True
#         self._flag_pub.publish(flag)

#     def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
#         self.get_logger().info('Received FollowJointTrajectory goal request')
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle) -> CancelResponse:
#         self.get_logger().info('Cancel request received')
#         return CancelResponse.ACCEPT

#     def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
#         self.get_logger().info('Executing trajectory')
#         traj = goal_handle.request.trajectory
#         n = self._kin.num_joints

#         # Validate joint names and points
#         if len(traj.joint_names) != n:
#             self.get_logger().error(f'Expected {n} joints, got {len(traj.joint_names)}')
#             goal_handle.abort()
#             result = FollowJointTrajectory.Result()
#             result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
#             return result

#         for idx, pt in enumerate(traj.points):
#             if len(pt.positions) != n:
#                 self.get_logger().error(f'Point #{idx} wrong size: {len(pt.positions)}')
#                 goal_handle.abort()
#                 res = FollowJointTrajectory.Result()
#                 res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
#                 return res

#         # Simulate execution with feedback
#         prev_t = 0.0
#         for idx, pt in enumerate(traj.points):
#             t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
#             dt = t - prev_t
#             if dt > 0.0:
#                 time.sleep(dt)
#             feedback = FollowJointTrajectory.Feedback()
#             feedback.joint_names = traj.joint_names
#             feedback.desired = pt
#             feedback.actual = pt
#             feedback.error = JointTrajectoryPoint()
#             goal_handle.publish_feedback(feedback)
#             self.get_logger().debug(f'Sent feedback for point {idx}')
#             prev_t = t

#         # Finished
#         result = FollowJointTrajectory.Result()
#         result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
#         goal_handle.succeed()
#         self.get_logger().info('Trajectory execution completed successfully')
#         return result

# # calling the main function 
# def main(args=None):
#     rclpy.init(args=args)
#     server = CartesianToJointActionServer()
#     try:
#         rclpy.spin(server)
#     except KeyboardInterrupt:
#         server.get_logger().info('Shutting down...')
#     finally:
#         server.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy  # ROS2 Python client library
from rclpy.node import Node  # imported Node module  
from rclpy.executors import MultiThreadedExecutor # imported module  
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState # imported the jointstate
from geometry_msgs.msg import PoseStamped  #imported Posestamed module
from control_msgs.action import FollowJointTrajectory # imported ros2 action module 
from trajectory_msgs.msg import JointTrajectoryPoint # imported module  
from typing import List, Optional,Tuple # imported module  
import cmd_interface as cmd  # hypothetical command interface module

# class Mairakinematics
class MairaKinematics:
    """
    Handles low-level joint-space motion commands for a 7-DOF manipulator.
    """
    def __init__(
        self,
        speed_move_joint: float = 1.0,
        acc_move_joint: float = 1.0
    ):
        self.num_joints: int = 7
        self.joint_names: List[str] = [
            f'joint{i+1}' for i in range(self.num_joints) # setting up the joint names ["joint1","joint2",.....]
        ]
        self._current_joint_state: Optional[List[float]] = None
        self.speed_move_joint: float = speed_move_joint
        self.acc_move_joint: float = acc_move_joint

        # Initialize command interface and ID manager
        self._program: cmd.ProgramInterface = cmd.ProgramInterface()
        self._id_manager: cmd.IdManager = cmd.IdManager()

# function move joint to joint 
    def move_joint_to_joint(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None
    ) -> bool:
        """Plan and execute a joint-space motion to the goal_pose."""
        self.throw_if_joint_invalid(goal_pose)
        plan_id: int = self._id_manager.update_id()

        properties = {
            'target_joint': [goal_pose],
            'speed': self.speed_move_joint if speed is None else speed,
            'acceleration': self.acc_move_joint if acc is None else acc,
            'interpolator': 1,
            'enable_blending': True,
        }

        self._program.set_command(
            cmd.Joint,
            **properties,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0
        )

        return self.execute_if_successful(plan_id)

# function to throw if the joint isinvalid or not 
    def throw_if_joint_invalid(self, pose: List[float]) -> None:
        """Ensure the pose list length matches the expected joint count."""
        if not isinstance(pose, list) or len(pose) != self.num_joints:
            raise TypeError(
                f"Expected {self.num_joints} joint values, got {pose}"
            )

# function to get the current joint state
    def get_current_joint_state(self) -> List[float]:
        """Return the latest joint state or raise if unknown."""
        if self._current_joint_state is None:
            raise RuntimeError("Current joint state unknown")
        return self._current_joint_state

# function to execute if the id is succesful 
    def execute_if_successful(self, plan_id: int) -> bool:
        """Execute a previously planned trajectory and raise on failure."""
        success: bool = self._program.execute_plan(plan_id)
        if not success:
            raise RuntimeError(f"Execution of plan {plan_id} failed")
        return True

# class Actionserver
class MovejointtoJointActionServer(Node):
    """
    ROS2 node that accepts PoseStamped commands whose position (x,y,z)
    plus orientation (x,y,z,w) together encode a 7-element joint target,
    and also provides a FollowJointTrajectory action interface.
    """
    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing action server...')

        # Kinematics + motion interface
        self._kin: MairaKinematics = MairaKinematics()

        # Subscribe for PoseStamped commands on /cmd_pose
        self.create_subscription(
            PoseStamped,
            '/cmd_pose',
            self.on_pose_cmd,
            10
        )

        # Subscribe to /joint_states for feedback
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # FollowJointTrajectory action server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

# callback
    def on_pose_cmd(self, msg: PoseStamped) -> None:
        """
        Handle incoming PoseStamped commands by interpreting
        [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w] as joint angles.
        """
        self.get_logger().info("Recieved the joint target on cmd pose")
        try:
            j = msg.pose
            joint_angles = [
                j.position.x,
                j.position.y,
                j.position.z,
                j.orientation.x,
                j.orientation.y,
                j.orientation.z,
                j.orientation.w,
            ]
            self._kin.move_joint_to_joint(joint_angles)
            self.get_logger().info('Motion command completed successfully')
        except Exception as e:
            self.get_logger().error(f'Motion failed: {e}')

# joint state callback
    def joint_state_callback(self, msg: JointState) -> None:
        """Update the cached current joint state from sensors."""
        self._kin._current_joint_state = list(msg.position)

# function for callback
    def goal_callback(
        self,
        goal_request: FollowJointTrajectory.Goal
    ) -> GoalResponse:
        """Validate incoming FollowJointTrajectory goals."""
        self.get_logger().info('FollowJointTrajectory goal received')
        names = goal_request.trajectory.joint_names

        if len(names) != self._kin.num_joints:
            self.get_logger().error(
                f'Invalid joint count: {len(names)} (expected {self._kin.num_joints})'
            )
            return GoalResponse.REJECT

        if set(names) != set(self._kin.joint_names):
            self.get_logger().error(
                f'Joint name mismatch: {names} vs {self._kin.joint_names}'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept all cancel requests."""
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(
        self,
        goal_handle
    ) -> FollowJointTrajectory.Result:
        """
        Execute each point in the trajectory sequentially, publishing feedback.
        Blocking delays run in their own threads under a MultiThreadedExecutor.
        """
        self.get_logger().info('Executing trajectory...')
        traj = goal_handle.request.trajectory
        n = self._kin.num_joints

        # Validate each point's joint count
        for pt in traj.points:
            if len(pt.positions) != n:
                self.get_logger().error('Point length mismatch')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

        prev_t = 0.0
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            if dt > 0.0:
                self.move_delay(dt)

            feedback = FollowJointTrajectory.Feedback(
                joint_names=traj.joint_names,
                desired=pt,
                actual=pt,
                error=JointTrajectoryPoint()
            )
            goal_handle.publish_feedback(feedback)
            prev_t = t

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def move_delay(self, dt: float) -> None:
        """
        Blocking delay—executed in its own thread under MultiThreadedExecutor,
        so other callbacks (e.g., cancel) remain responsive.
        """
        import time
        time.sleep(dt)

# main function 
def main(args=None):

    rclpy.init(args=args)
    node = MovejointtoJointActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

# calling the main function
if __name__ == '__main__':
    main()
