
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


# import rclpy  # ROS2 Python client library
# from rclpy.node import Node  # imported Node module  
# from rclpy.executors import MultiThreadedExecutor # imported module  
# from rclpy.action import ActionServer, CancelResponse, GoalResponse 
# from sensor_msgs.msg import JointState # imported the jointstate
# from geometry_msgs.msg import PoseStamped  #imported Posestamed module
# from control_msgs.action import FollowJointTrajectory # imported ros2 action module 
# from trajectory_msgs.msg import JointTrajectoryPoint # imported module  
# from typing import List, Optional # imported module  
# import cmd_interface as cmd  # hypothetical command interface module

# # class Mairakinematics
# class MairaKinematics:
#     """
#     Handles low-level joint-space motion commands for a 7-DOF manipulator.
#     """
#     def __init__(
#         self,
#         speed_move_joint: float = 1.0,
#         acc_move_joint: float = 1.0
#     ):
#         self.num_joints: int = 7
#         self.joint_names: List[str] = [
#             f'joint{i+1}' for i in range(self.num_joints)
#         ]
#         self._current_joint_state: Optional[List[float]] = None
#         self.speed_move_joint: float = speed_move_joint
#         self.acc_move_joint: float = acc_move_joint

#         # Initialize command interface and ID manager
#         self._program: cmd.ProgramInterface = cmd.ProgramInterface()
#         self._id_manager: cmd.IdManager = cmd.IdManager()

# # function move joint to joint 
#     def move_joint_to_joint(
#         self,
#         goal_pose: List[float],
#         speed: Optional[float] = None,
#         acc: Optional[float] = None
#     ) -> bool:
#         """Plan and execute a joint-space motion to the goal_pose."""
#         self.throw_if_joint_invalid(goal_pose)
#         plan_id: int = self._id_manager.update_id()

#         props = {
#             'target_joint': [goal_pose],
#             'speed': self.speed_move_joint if speed is None else speed,
#             'acceleration': self.acc_move_joint if acc is None else acc,
#             'interpolator': 1,
#             'enable_blending': True,
#         }

#         self._program.set_command(
#             cmd.Joint,
#             **props,
#             cmd_id=plan_id,
#             current_joint_angles=self.get_current_joint_state(),
#             reusable_id=0
#         )

#         return self.execute_if_successful(plan_id)

# # function to throw if the joint isinvalid or not 
#     def throw_if_joint_invalid(self, pose: List[float]) -> None:
#         """Ensure the pose list length matches the expected joint count."""
#         if not isinstance(pose, list) or len(pose) != self.num_joints:
#             raise TypeError(
#                 f"Expected {self.num_joints} joint values, got {pose}"
#             )

# # function to get the current joint state
#     def get_current_joint_state(self) -> List[float]:
#         """Return the latest joint state or raise if unknown."""
#         if self._current_joint_state is None:
#             raise RuntimeError("Current joint state unknown")
#         return self._current_joint_state

# # function to execute if the id is succesful 
#     def execute_if_successful(self, plan_id: int) -> bool:
#         """Execute a previously planned trajectory and raise on failure."""
#         success: bool = self._program.execute_plan(plan_id)
#         if not success:
#             raise RuntimeError(f"Execution of plan {plan_id} failed")
#         return True

# # class Actionserver
# class CartesianToJointActionServer(Node):
#     """
#     ROS2 node that accepts PoseStamped commands whose position (x,y,z)
#     plus orientation (x,y,z,w) together encode a 7-element joint target,
#     and also provides a FollowJointTrajectory action interface.
#     """
#     def __init__(self):
#         super().__init__('cartesian_to_joint_action_server')
#         self.get_logger().info('Initializing action server...')

#         # Kinematics + motion interface
#         self._kin: MairaKinematics = MairaKinematics()

#         # Subscribe for PoseStamped commands on /cmd_pose
#         self.create_subscription(
#             PoseStamped,
#             '/cmd_pose',
#             self.on_pose_cmd,
#             10
#         )

#         # Subscribe to /joint_states for feedback
#         self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_cb,
#             10
#         )

#         # FollowJointTrajectory action server
#         self._action_server = ActionServer(
#             node=self,
#             action_type=FollowJointTrajectory,
#             action_name='joint_trajectory_position_controller/follow_joint_trajectory',
#             execute_callback=self.execute_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback
#         )

# # callback
#     def on_pose_cmd(self, msg: PoseStamped) -> None:
#         """
#         Handle incoming PoseStamped commands by interpreting
#         [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w] as joint angles.
#         """
#         self.get_logger().info("Recieved the joint target on cmd pose")
#         try:
#             j = msg.pose
#             joint_angles = [
#                 j.position.x,
#                 j.position.y,
#                 j.position.z,
#                 j.orientation.x,
#                 j.orientation.y,
#                 j.orientation.z,
#                 j.orientation.w,
#             ]
#             self._kin.move_joint_to_joint(joint_angles)
#             self.get_logger().info('Motion command completed successfully')
#         except Exception as e:
#             self.get_logger().error(f'Motion failed: {e}')

# # joint state callback
#     def joint_state_cb(self, msg: JointState) -> None:
#         """Update the cached current joint state from sensors."""
#         self._kin._current_joint_state = list(msg.position)


#     def goal_callback(
#         self,
#         goal_request: FollowJointTrajectory.Goal
#     ) -> GoalResponse:
#         """Validate incoming FollowJointTrajectory goals."""
#         self.get_logger().info('FollowJointTrajectory goal received')
#         names = goal_request.trajectory.joint_names

#         if len(names) != self._kin.num_joints:
#             self.get_logger().error(
#                 f'Invalid joint count: {len(names)} (expected {self._kin.num_joints})'
#             )
#             return GoalResponse.REJECT

#         if set(names) != set(self._kin.joint_names):
#             self.get_logger().error(
#                 f'Joint name mismatch: {names} vs {self._kin.joint_names}'
#             )
#             return GoalResponse.REJECT

#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle) -> CancelResponse:
#         """Accept all cancel requests."""
#         self.get_logger().info('Cancel request received')
#         return CancelResponse.ACCEPT

#     def execute_callback(
#         self,
#         goal_handle
#     ) -> FollowJointTrajectory.Result:
#         """
#         Execute each point in the trajectory sequentially, publishing feedback.
#         Blocking delays run in their own threads under a MultiThreadedExecutor.
#         """
#         self.get_logger().info('Executing trajectory...')
#         traj = goal_handle.request.trajectory
#         n = self._kin.num_joints

#         # Validate each point's joint count
#         for pt in traj.points:
#             if len(pt.positions) != n:
#                 self.get_logger().error('Point length mismatch')
#                 goal_handle.abort()
#                 result = FollowJointTrajectory.Result()
#                 result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
#                 return result

#         prev_t = 0.0
#         for pt in traj.points:
#             t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
#             dt = t - prev_t
#             if dt > 0.0:
#                 self.move_delay(dt)

#             feedback = FollowJointTrajectory.Feedback(
#                 joint_names=traj.joint_names,
#                 desired=pt,
#                 actual=pt,
#                 error=JointTrajectoryPoint()
#             )
#             goal_handle.publish_feedback(feedback)
#             prev_t = t

#         goal_handle.succeed()
#         result = FollowJointTrajectory.Result()
#         result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
#         return result

#     def move_delay(self, dt: float) -> None:
#         """
#         Blocking delay—executed in its own thread under MultiThreadedExecutor,
#         so other callbacks (e.g., cancel) remain responsive.
#         """
#         import time
#         time.sleep(dt)

# # main function 
# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = CartesianToJointActionServer()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# # calling the main function
# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer , CancelResponse,GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint 
from typing import List, Optional
import copy 
import cmd

#from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
from neurapy_ai_utils.functions.utils import init_logger

import sys
sys.path.append("/home/neura/neurapy_alpha/")
from neurapy.robot import Robot
# from neurapy.commands.state.robot_status import RobotStatus
# from neurapy.utils import CmdIDManager

class MairaKinematics(Node):
    def __init__(self, joint_names_param: List[str],
                 id_manager: int = 0, 
                 speed_move_joint: int = 20,
                 speed_move_linear: float = 0.1,
                 rot_speed_move_linear: float = 0.87266463,
                 acc_move_joint: int = 20,
                 acc_move_linear: float = 0.1,
                 rot_acc_move_linear: float = 1.74532925,
                 blending_radius: float = 0.005,
                 require_elbow_up: bool = True,
                ):
        self._logger = init_logger(name=self.__class__.__name__)
        
        # Set joint names
        if not joint_names_param or len(joint_names_param) == 0:
            self._joint_names = [f'joint{i+1}' for i in range(7)]
            self._logger.info("Using default joint names")
        else:
            self._joint_names = joint_names_param

        self._logger.info(f"Joint names: {self._joint_names}")
        self.num_joints = len(self._joint_names)

        self.id_manager = id_manager
        self.speed_move_joint = speed_move_joint
        self.speed_move_linear = speed_move_linear
        self.rot_speed_move_linear = rot_speed_move_linear
        self.acc_move_joint = acc_move_joint
        self.acc_move_linear = acc_move_linear
        self.rot_acc_move_linear = rot_acc_move_linear
        self.blending_radius = blending_radius
        self.require_elbow_up = require_elbow_up

        self._robot = Robot()
        self._logger.info("[MairaKinematics]: init success.")

    def move_joint_to_joint(
        self,
        goal_pose: List[float],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """Move the robot's joints to a specified joint configuration.

        This method commands the robot to move its joints to the specified
        angular positions.

        Parameters
        ----------
        goal_pose: list[float]
            Goal joint positions, represented as a list of floats (in radians).
        speed: Optional[int], optional
            Joint speed as a percentage value [0, 100]. If None, the class'
            default value is used.
        acc: Optional[int], optional
            Joint acceleration as a percentage value [0, 100]. If None, the
            class' default value is used.

        Returns
        -------
        bool
            True if the move command was successful; False otherwise.

        Raises
        ------
        TypeError
            If the input type is incorrect.
        RuntimeError
            If the action fails.
        """
        self._throw_if_joint_invalid(goal_pose)
        if self.id_manager < 1:
            self._robot.switch_to_automatic_mode()
            self._robot.init_program()

        success = False
        self.id_manager = self.id_manager + 1
        motion_id = self.id_manager
        joint_property = {
            "target_joint": [self.get_current_joint_state(), goal_pose],
            # TODO this adds only a joint motion no p2p motion, if this is wanted this is okay, else we could add the cartesian values, see comment
            # "target_pose": [self._get_current_cartesian_pose(), #add value here] # if p2p is wanted instead of move joint
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "blending": True,
            "control_mode": 0,
            "current_joint_angles": self.get_current_joint_state(),
        }
        try:
            self._robot.move_joint(**joint_property, motion_id=motion_id)
            self._robot.wait_motion_finished()
            success = True
        except Exception as e:
            self._logger.error(e)
            success = False
        return success

    def move_joint_via_points(
        self,
        trajectory: List[List[float]],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """Move the robot's joints through a series of joint configurations.

        This method commands the robot to move its joints through the specified
        joint configurations.

        Note: Blending is not supported for joint interpolation in NeuraPy.

        Parameters
        ----------
        trajectory : List[List[float]]
            A list of joint configurations. Each configuration is represented
            as a list of floats (in radians).
        speed: Optional[int], optional
            Joint speed as a percentage value [0, 100]. If None, the class'
            default value is used.
        acc: Optional[int], optional
            Joint acceleration as a percentage value [0, 100]. If None, the
            class' default value is used.

        Returns
        -------
        bool
            True if the move command was successful; False otherwise.

        Raises
        ------
        TypeError
            If the input type is incorrect.
        RuntimeError
            If the action fails.
        """
        self._throw_if_trajectory_invalid(trajectory)
        if self.id_manager < 1:
            self._robot.switch_to_automatic_mode()
            self._robot.init_program()

        if len(trajectory) > 50:
            trajectory = trajectory[0::5]
            chunk_size = 20

            self._logger.info(f"Processing total points {len(trajectory)}")
            for index in range(0, len(trajectory), chunk_size):
                end = min(index + chunk_size, len(trajectory))
                self._logger.info(f"Processing points {index} to {end-1}")

                self.id_manager = self.id_manager + 1
                motion_id = self.id_manager
                joint_property = {
                    "target_joint": trajectory[index:end],
                    "speed": self.speed_move_joint if speed is None else speed,
                    "acceleration": self.acc_move_joint if acc is None else acc,
                    "interpolator": 1,
                    "blending": True,
                    "enable_blending": True,
                    "control_mode": 0,
                    "current_joint_angles": self.get_current_joint_state(),
                    "blocking": False
                }
                try:
                    self._robot.move_joint(**joint_property, motion_id=motion_id)
                    self._robot.wait_motion_finished()
                    success = True
                except Exception as e:
                    self._logger.error(e)
                    success = False
                    return False
        else:
            self.id_manager = self.id_manager + 1
            motion_id = self.id_manager
            joint_property = {
                "target_joint": trajectory,
                "speed": self.speed_move_joint if speed is None else speed,
                "acceleration": self.acc_move_joint if acc is None else acc,
                "interpolator": 1,
                "blending": True,
                "enable_blending": True,
                "control_mode": 0,
                "current_joint_angles": self.get_current_joint_state(),
            }
            try:
                self._robot.move_joint(**joint_property, motion_id=motion_id)
                self._robot.wait_motion_finished()
                success = True
            except Exception as e:
                self._logger.error(e)
                success = False
        return success

    def get_current_joint_state(self) -> List[float]:
        return self._robot.get_current_joint_angles()

    def wait_motion_finish(self, target: List[float], threshold: float = 0.01, timeout: float = 50.0, rate: float = 10.0) -> bool:
        """
        Waits until all joint values are within the threshold between current and target positions.

        Args:
            start (List[float]): Starting joint values (initial positions).
            target (List[float]): Target joint values (desired positions).
            threshold (float): Allowed difference to consider the joint has reached the target.
            timeout (float): Maximum time to wait in seconds.
            rate (float): Check frequency in Hz.

        Returns:
            bool: True if target was reached within threshold, False if timeout occurred.
        """
        start_time = time.time()
        period = 1.0 / rate
        while time.time() - start_time < timeout:
            # Replace this with actual current joint readings from robot/state
            current = self._robot.get_current_joint_angles()
            self._logger.info(f"[wait_motion_finish]: current joint states: {current}")
            self._logger.info(f"[wait_motion_finish]: target joint states: {target}")
            if all(abs(c - t) <= threshold for c, t in zip(current, target)):
                return True
            time.sleep(period)
        return False

    def _throw_if_joint_invalid(self, joint_states: List[float]) -> None:
        """Raise an error if the given joint states are invalid.

        This internal method validates the provided joint states to ensure they
        meet the required format and dimensions. It checks whether the joint
        states are a list of floats with a length equal to the number of joints
        in the robot.

        Parameters
        ----------
        joint_states : List[float]
            Joint states, represented as a list of floats.

        Raises
        ------
        TypeError:
            If the joint states are not a list or if they do not have the
            correct length (number of joints).

        """
        if isinstance(joint_states, tuple):
            joint_states = list(joint_states)

        if not (
            (isinstance(joint_states, list))
            and len(joint_states) == self.num_joints
        ):
            raise TypeError(
                f"[ERROR] joint_states should be a list with length {self.num_joints}!"
            )
    
    def _throw_if_trajectory_invalid(
        self, trajectory: List[List[float]]
    ) -> None:
        """Raise an error if the given joint trajectory is invalid.

        This internal method checks whether the provided trajectory meets the
        required format and dimensions. It ensures that the trajectory is a list
        of joint states, where each joint state is a list of floats representing
        the joint angles.

        Parameters
        ----------
        trajectory : List[List[float]]
            Joint trajectory, represented as a list of joint states. Each joint
            state is a list of floats representing the joint angles.

        Raises
        ------
        TypeError:
            If the trajectory is not a list or if the joint states within the
            trajectory do not have the correct length (number of joints).

        """
        if not (isinstance(trajectory, list)):
            raise TypeError(
                f"[ERROR] trajectory should be a List[List[float*\
                    {self.num_joints}]]!"
            )
        for joint_states in trajectory:
            self._throw_if_joint_invalid(joint_states)

class SimpleJointTrajectoryServer(Node):
    """Simplified ROS2 Action Server for joint trajectory control using MairaKinematics."""

    def __init__(self):
        """Initialize the Simple Joint Trajectory Server."""
        super().__init__('simple_joint_trajectory_server')
        
        self._logger = init_logger(name=self.__class__.__name__)
        
        # Declare parameters
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
        self.declare_parameter('action_name', 'follow_joint_trajectory')
        
        # Get joint names from parameters
        joint_names_param = self.get_parameter('joint_names').get_parameter_value().string_array_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        
        # Initialize MairaKinematics
        self._maira_kinematics = MairaKinematics(joint_names_param=joint_names_param)
        
        # Create action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            self._execute_callback
        )
        
        # Joint state publisher
        self._joint_state_publisher = self.create_publisher(JointState, '/internal_kin_joint_states', 10)
        self._joint_state_timer = self.create_timer(0.02, self._publish_joint_states)  # 50Hz
        
        self._logger.info("Simple Joint Trajectory Action Server initialized")

    def _execute_callback(self, goal_handle):
        """Execute the trajectory following action.
        
        Parameters
        ----------
        goal_handle : ServerGoalHandle
            The goal handle containing the trajectory to execute
        """
        self._logger.info("Executing trajectory")
        
        trajectory = goal_handle.request.trajectory
        # Basic validation
        if not trajectory.points:
            self._logger.error("Empty trajectory received")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
            
        if len(trajectory.joint_names) != self._maira_kinematics.num_joints:
            self._logger.error(f"Joint names mismatch. Expected {self._maira_kinematics.num_joints}, got {len(trajectory.joint_names)}")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Convert trajectory to joint positions
        joint_positions = []
        for point in trajectory.points:
            if len(point.positions) != self._maira_kinematics.num_joints:
                self._logger.error(f"Point has {len(point.positions)} positions, expected {self._maira_kinematics.num_joints}")
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            joint_positions.append(list(point.positions))
        
        self._logger.info(f"Executing trajectory with {len(joint_positions)} points")
        
        try:
            # Execute trajectory
            if len(joint_positions) == 1:
                # Single point trajectory
                success = self._maira_kinematics.move_joint_to_joint(joint_positions[0]) #self._maira_kinematics.move_joint_to_joint(joint_positions[0])
            else:
                # Multi-point trajectory
                success = self._maira_kinematics.move_joint_via_points(joint_positions)
            
            # Prepare result
            result = FollowJointTrajectory.Result()
            
            if success:
                self._logger.info("Trajectory execution completed successfully")
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
            else:
                self._logger.error("Trajectory execution failed")
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                goal_handle.abort()
                
            return result
            
        except Exception as e:
            self._logger.error(f"Exception during trajectory execution: {e}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

    def _publish_joint_states(self):
        """Publish current joint states."""
        try:
            current_joints = self._maira_kinematics.get_current_joint_state()
            
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self._maira_kinematics._joint_names
            joint_state.position = current_joints
            joint_state.velocity = [0.0] * len(current_joints)
            joint_state.effort = [0.0] * len(current_joints)
            
            self._joint_state_publisher.publish(joint_state)
            
        except Exception as e:
            self._logger.debug(f"Error publishing joint states: {e}")

    def destroy_node(self):
        """Clean up resources when destroying the node."""
        self._logger.info("Shutting down Simple Joint Trajectory Action Server")
        
        # Clean up MairaKinematics
        if hasattr(self._maira_kinematics, 'finish'):
            self._maira_kinematics.finish()
            
        super().destroy_node()