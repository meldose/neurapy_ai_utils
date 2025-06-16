import time  # standard library
from copy import deepcopy # imported copy module
from typing import List, Optional, Tuple # imported List, Optional and Tuple

import numpy as np  # third-party
import rclpy #imported ros2 module
from rclpy.node import Node #imported Node module 
from rclpy.action import ActionServer, CancelResponse, GoalResponse #imported ActionServer , CancelResponse and GoalResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # imported JointTrajectory
from sensor_msgs.msg import JointState #imported Joinstate
from control_msgs.action import FollowJointTrajectory # imported FollowJointTrajectory


# class InvalidTrajectory
class InvalidTrajectory(Exception):
    """Raised when trajectory validation fails."""
    pass

# class Mairakinematics
class Mairakinematics(Node):
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
        self._robot = None  # IK/FK solver & tool manager
        self._elbow_checker = None  # elbow orientation checker
        self.require_elbow_up: bool = False

        # --- Publisher & Subscriber ---
        self._cmd_pub = self.create_publisher(
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

    # --- ROS callbacks ---
    def joint_state(self, msg: JointState) -> None:
        """Store latest joint positions for feedback."""
        self._current_joint_state = list(msg.position)

    def goal_callback(self, request) -> GoalResponse:
        """Accept all incoming trajectory goals."""
        self.get_logger().info('Goal request received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept cancellation requests."""
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        """
        Process FollowJointTrajectory goals: validate, execute waypoints,
        and publish feedback or cancellation.
        """
        traj = goal_handle.request.trajectory
        self.get_logger().info(f"Executing trajectory with {len(traj.points)} points")

        # Validate trajectory shape and timing
        try:
            self._validate_trajectory(traj.points)
        except InvalidTrajectory as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = result.INVALID_GOAL
            result.error_string = str(e)
            return result

        # Execute waypoints
        for idx, pt in enumerate(traj.points):
            if goal_handle.is_cancel_requested():
                self.get_logger().info('Trajectory execution cancelled')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

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
    def publish_command(self, positions: List[float], time_from_start) -> None:
        """Wrap and publish a single JointTrajectory waypoint."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = time_from_start
        msg.points = [pt]
        self._cmd_pub.publish(msg)

    def get_current_joint_state(self) -> List[float]:
        """Return the last received joint-state positions."""
        return self._current_joint_state

    def get_current_cartesian_tcp_pose(self) -> List[float]:
        """Get current TCP pose as [x,y,z,qx,qy,qz,qw]."""
        status = self._robot.getRobotStatus('cartesianPosition')
        return list(status)

    # --- Validation routines ---
    def _validate_trajectory(self, points: List[JointTrajectoryPoint]) -> None:
        """Ensure trajectory points are well-formed."""
        if not points:
            raise InvalidTrajectory('Trajectory has no points')

        last_time = None
        for idx, pt in enumerate(points):
            positions = pt.positions
            # Validate list length
            if len(positions) != self.num_joints:
                raise InvalidTrajectory(
                    f'Point {idx} has {len(positions)} positions; expected {self.num_joints}'
                )
            # Validate time monotonicity
            if last_time is not None and pt.time_from_start <= last_time:
                raise InvalidTrajectory(
                    f'Time from start not strictly increasing at point {idx}'
                )
            last_time = pt.time_from_start

    # --- IK Wrappers ---
    def get_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float]
    ) -> List[float]:
        """
        Try multiple seeds to find a non-NaN IK solution.
        Uses numpy Generator for reproducible perturbations.
        """
        ref = np.array(reference_joint_states)
        for seed in range(100):
            rng = np.random.default_rng(seed)
            trial = (ref + rng.normal(scale=1e-2, size=self.num_joints)).tolist()
            try:
                sol = self._robot.ik_fk(
                    'ik', target_pose=goal_pose_cartesian, current_joint=trial
                )
            except Exception as e:
                self.get_logger().debug(f'IK error seed={seed}: {e}')
                continue
            if not np.isnan(sol).any():
                return sol

        raise ValueError('IK solver failed after 100 seeds')

    def get_elbow_up_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float]
    ) -> List[float]:
        """Try elbow-up variations by offsetting joints 5 & 6."""
        deltas = [(0, 0), (0, np.pi), (np.pi, 0), (np.pi, np.pi)]
        for dx, dy in deltas:
            seed_joints = deepcopy(reference_joint_states)
            # Flip offsets based on sign
            seed_joints[5] += -dx if seed_joints[5] >= 0 else dx
            seed_joints[6] += -dy if seed_joints[6] >= 0 else dy
            try:
                sol = self.get_ik_solution(goal_pose_cartesian, seed_joints)
                if self._elbow_checker.is_up(sol):
                    return sol
            except ValueError:
                continue

        raise ValueError('Elbow-up IK not found')

    def cartesian_2_joint(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: Optional[List[float]] = None
    ) -> List[float]:
        """
        Convert Cartesian pose to joint space using IK.
        Honors `require_elbow_up` if set.
        """
        if not isinstance(goal_pose_cartesian, list) or len(goal_pose_cartesian) not in (6, 7):
            raise TypeError('Pose must be list of 6 or 7 floats')

        ref = reference_joint_states or self.get_current_joint_state()
        if not isinstance(ref, (list, tuple)) or len(ref) != self.num_joints:
            raise TypeError(f'Reference joints must be length {self.num_joints}')

        if self.require_elbow_up:
            return self.get_elbow_up_ik_solution(goal_pose_cartesian, ref)
        return self.get_ik_solution(goal_pose_cartesian, ref)


def main(args=None):
    rclpy.init(args=args)
    node = Mairakinematics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
