#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint 
from std_msgs.msg import Header
import time
from typing import List, Optional

#from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
from neurapy_ai_utils.functions.utils import init_logger

import sys
sys.path.append("/home/neura/neurapy_alpha/")
import neurapy
from neurapy.robot import Robot

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

        success = False
        self.id_manager = self.id_manager + 1
        motion_id = self.id_manager
        joint_property = {
            "target_joint": [self.get_current_joint_state(), goal_pose],
            # TODO this adds only a joint motion no p2p motion, if this is wanted this is okay, else we could add the cartesian values, see comment
            # "target_pose": [self._get_current_cartesian_pose(), #add value here] # if p2p is wanted instead of move joint
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "enable_blending": True,
            "control_mode": 0,
            "current_joint_angles": self.get_current_joint_state(),
        }
        try:
            self._robot.move_joint(**joint_property, motion_id=motion_id)
            #self._robot.wait_motion_finished()
            #success = True
            success = self.wait_motion_finish(target=goal_pose)
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
        self.id_manager = self.id_manager + 1
        motion_id = self.id_manager

        joint_property = {
            "target_joint": trajectory,
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "interpolator": 1,
            "enable_blending": True,
            "control_mode": 0,
        }
        try:
            self._robot.move_joint(**joint_property, motion_id=motion_id)
            #self._robot.wait_motion_finished()
            #success = True
            success = self.wait_motion_finish(target=trajectory[-1])
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

def main(args=None):
    """Main function to run the simple joint trajectory action server."""
    rclpy.init(args=args)
    
    try:
        # Create and run the action server
        action_server = SimpleJointTrajectoryServer()
        
        try:
            rclpy.spin(action_server)
        finally:
            action_server.destroy_node()
            
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 