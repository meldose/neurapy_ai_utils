import time  # standard library
from copy import deepcopy # imported copy
from typing import List, Optional, Union, Tuple

import numpy as np  # third-party
import rclpy  # imported ros2 module 
from rclpy.node import Node # imported the Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported the action server, CancelRespose and GoalResponse
from builtin_interfaces.msg import Duration #imported Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # imported Jointrajectory
from sensor_msgs.msg import JointState # imported Jointstate
from control_msgs.action import FollowJointTrajectory # immported FollowJoinTrajecotory
import cmd # imported cmd


# class InvalidTrajectory
class InvalidTrajectory(Exception):
    """Raised when trajectory validation fails."""
    pass

# class Mairakinematics
class MairaKinematics(Node):
    def __init__(self):
        super().__init__('maira_kinematics')

        # --- Declare parameters ---
        self.declare_parameter('speed_move_joint', 1.0)
        self.declare_parameter('speed_move_linear', 1.0)
        self.declare_parameter('acc_move_joint', 1.0)
        self.declare_parameter('acc_move_linear', 1.0)
        self.declare_parameter('joint_names', [
            'joint1','joint2','joint3','joint4',
            'joint5','joint6','joint7'
        ])
        self.declare_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory',
            '/follow_joint_trajectory'
        )
        self.declare_parameter('command_topic', '/joint_command')

        # --- Retrieve parameters ---
        self.speed_move_joint: float = self.get_parameter('speed_move_joint').value
        self.speed_move_linear: float = self.get_parameter('speed_move_linear').value
        self.acc_move_joint: float = self.get_parameter('acc_move_joint').value
        self.acc_move_linear: float = self.get_parameter('acc_move_linear').value
        self.joint_names: List[str] = self.get_parameter('joint_names').value
        self.num_joints: int = len(self.joint_names)

        action_name: str = self.get_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory'
        ).value
        command_topic: str = self.get_parameter('command_topic').value

        # --- Robot interfaces ---
        self._robot = None              # IK/FK solver & tool manager
        self._elbow_checker = None      # elbow orientation checker
        self._program = None            # motion program interface
        self._id_manager = None         # plan ID manager

        # --- Publisher & Subscriber ---
        self.cmd_pub = self.create_publisher(
            JointTrajectory, command_topic, 10
        )
        self._current_joint_state: List[float] = [0.0] * self.num_joints
        self.create_subscription(
            JointState, 'joint_states', self.joint_state, 10
        )

        # --- Action Server ---
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    # --- ROS Callbacks ---
    def joint_state(self, msg: JointState) -> None:
        self._current_joint_state = list(msg.position)

    def goal_callback(self, request) -> GoalResponse:
        self.get_logger().info('Goal request received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        traj = goal_handle.request.trajectory
        try:
            self.validate_trajectory(traj.points)
        except InvalidTrajectory as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = result.INVALID_GOAL
            result.error_string = str(e)
            return result

        start_time = self.get_clock().now()
        for pt in traj.points:
            if goal_handle.is_cancel_requested():
                self.get_logger().info('Trajectory execution cancelled')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = start_time + pt.time_from_start
            while self.get_clock().now() < target_time:
                if goal_handle.is_cancel_requested():
                    self.get_logger().info('Trajectory execution cancelled')
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                rclpy.spin_once(self, timeout_sec=0.01)

            self.publish_command(pt.positions, pt.time_from_start)
            feedback = FollowJointTrajectory.Feedback()
            feedback.actual.positions = self.get_current_joint_state()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESS
        self.get_logger().info('Trajectory execution completed')
        return result

    # --- Publishing & State ---
    def publish_command(self, positions: List[float], time_from_start: Duration) -> None:
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = time_from_start
        msg.points = [pt]
        self.cmd_pub.publish(msg)

    def get_current_joint_state(self) -> List[float]:
        self.get_logger().info("Successfully returned the current joint state")
        return self._current_joint_state

    def get_current_cartesian_tcp_pose(self) -> List[float]:
        status = self._robot.getRobotStatus('cartesianPosition')
        self.get_logger().info("Successfull returned the current tcp pose")
        return list(status)

    # --- Validation ---
    def validate_trajectory(self, points: List[JointTrajectoryPoint]) -> None:
        if not points:
            raise InvalidTrajectory('Trajectory has no points')
        last_time = None
        for idx, pt in enumerate(points):
            if len(pt.positions) != self.num_joints:
                raise InvalidTrajectory(
                    f'Point {idx} has {len(pt.positions)} positions; expected {self.num_joints}'
                )
            if last_time is not None and pt.time_from_start <= last_time:
                raise InvalidTrajectory(
                    f'Time from start not strictly increasing at point {idx}'
                )
            last_time = pt.time_from_start

    # --- IK Wrappers ---
    def _get_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float],
    ) -> List[float]:
        solution = []
        for i in range(100):
            np.random.seed(i)
            dummy_array = reference_joint_states + np.random.randn(self.num_joints) * 1e-2
            try:
                solution = self._robot.ik_fk(
                    "ik",
                    target_pose=goal_pose_cartesian,
                    current_joint=dummy_array,
                )
            except Exception as e:
                self.get_logger().debug(e)
                continue
            if not np.any(np.isnan(solution)):
                return solution
        raise ValueError('IK solution returned NaN')

    def _get_elbow_up_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float],
    ) -> List[float]:
        retry_states = [(0.0, 0.0), (0.0, np.pi), (np.pi, 0.0), (np.pi, np.pi)]
        for dx, dy in retry_states:
            seed = deepcopy(reference_joint_states)
            seed[5] += -dx if seed[5] >= 0 else dx
            seed[6] += -dy if seed[6] >= 0 else dy
            try:
                sol = self._get_ik_solution(goal_pose_cartesian, seed)
                if self._elbow_checker.is_up(sol):
                    self.get_logger().debug('Elbow up!')
                    return sol
            except ValueError:
                continue
        self.get_logger().debug('Elbow down!')
        raise ValueError('Could not find elbow up solution')

    def cartesian_2_joint(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: Optional[List[float]] = None
    ) -> List[float]:
        if not isinstance(goal_pose_cartesian, list) or len(goal_pose_cartesian) not in (6, 7):
            raise TypeError('Pose must be list of 6 or 7 floats')
        ref = reference_joint_states or self.get_current_joint_state()
        if self.require_elbow_up:
            return self._get_elbow_up_ik_solution(goal_pose_cartesian, ref)
        return self._get_ik_solution(goal_pose_cartesian, ref)

    # --- Motion Commands ---
    def set_motion_till_force(
        self,
        stopping_forces: List[float] = [0.0, 0.0, 1.0],
        reflex_mode_after_contact: bool = True
    ) -> None:
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=stopping_forces,
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=reflex_mode_after_contact
        )

    def move_joint_to_cartesian(
        self,
        goal_pose: List[float],
        reference_joint_states: Optional[List[float]] = None,
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        self.throw_if_pose_invalid(goal_pose)
        joint_pose = self.cartesian_2_joint(goal_pose, reference_joint_states)
        self.get_logger().info("Successfully returned moved from joint to cartesian")
        return self.move_joint_to_joint(joint_pose, speed, acc)


    def move_joint_to_joint(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        prop = {
            'target_joint': [goal_pose],
            'speed': self.speed_move_joint if speed is None else speed,
            'acceleration': self.acc_move_joint if acc is None else acc,
            'interpolator': 1,
            'enable_blending': True
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **prop,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0
        )
        self.get_logger.info("Successfully returned moved from joint to joint")
        return self._is_id_successful(plan_id)[0]

    def move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
    ) -> bool:
        prop = {
            'target_pose': [self.get_current_cartesian_tcp_pose(), goal_pose],
            'speed': self.speed_move_linear if speed is None else speed,
            'acceleration': self.acc_move_linear if acc is None else acc,
            'blending': False,
            'blend_radius': 0.0
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **prop,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0
        )
        self.get_logger.info("Successfully moved linearly")
        return self._is_id_successful(plan_id)[0]

    def move_linear_via_points(
        self,
        goal_poses: List[List[float]],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
        blending_radius: Optional[float] = None,
    ) -> bool:
        self._throw_if_list_poses_invalid(goal_poses)
        poses = [self.get_current_cartesian_tcp_pose()] + goal_poses
        prop = {
            'target_pose': poses,
            'speed': self.speed_move_linear if speed is None else speed,
            'acceleration': self.acc_move_linear if acc is None else acc,
            'blend_radius': 0.01 if blending_radius is None else blending_radius
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **prop,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0
        )
        self.get_logger.info("Successfully moved linearly through points")
        return self._is_id_successful(plan_id)[0]

    def move_joint_via_points(
        self,
        trajectory: List[List[float]],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        self.throw_if_trajectory_invalid(trajectory)
        prop = {
            'target_joint': trajectory,
            'speed': self.speed_move_joint if speed is None else speed,
            'acceleration': self.acc_move_joint if acc is None else acc,
            'interpolator': 1,
            'enable_blending': True
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **prop,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0
        )
        self.get_logger.info("Successfully moved through joints through points")
        return self._is_id_successful(plan_id)[0]

    def execute(self, ids: List[int], execution_feasibilities: List[bool]) -> None:
        executable = [i for i, ok in zip(ids, execution_feasibilities) if ok]
        self.get_logger().info(f"Executing IDs: {executable}")
        self._program.execute(executable)

    def wait(self, time_s: float) -> None:
        """Wait for given time in seconds."""
        time.sleep(time_s)
        self.get_logger().info("Succesfully waited for the specific time given")

    # --- Validation Helpers ---
    def throw_if_trajectory_invalid(self, trajectory: List[List[float]]) -> None:
        if not isinstance(trajectory, list):
            raise TypeError(
                f"[ERROR] trajectory should be a List[List[float*{self.num_joints}]]!"
            )
        for joint_states in trajectory:
            self.throw_if_joint_invalid(joint_states)

    def throw_if_joint_invalid(self, joint_states: List[float]) -> None:
        if isinstance(joint_states, tuple):
            joint_states = list(joint_states)
        if not (isinstance(joint_states, list) and len(joint_states) == self.num_joints):
            raise TypeError(
                f"[ERROR] joint_states should be a list with length {self.num_joints}!"
            )

    def throw_if_pose_invalid(self, pose: List[float]) -> None:
        if not (isinstance(pose, list) and len(pose) == 6):
            raise TypeError("[ERROR] goal_pose should be a list with length 6!")

    def _throw_if_list_poses_invalid(self, goal_poses: List[List[float]]) -> None:
        if not isinstance(goal_poses, list):
            raise TypeError("[ERROR] goal_poses should be a List[List[float*6]]!")
        for pose in goal_poses:
            self.throw_if_pose_invalid(pose)

    # TODO not implemented:

# function for speed to percent 
    def speed_to_percent(self, speed_mps):
        if speed_mps is None:
            speed_mps = self.speed_move_joint
        return 50

# function for acc to percent
    def acc_to_percent(self, acc):
        if acc is None:
            acc = self.acc_move_joint
        return 50

# main function 
def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

# calling main function
if __name__ == '__main__':
    main()
