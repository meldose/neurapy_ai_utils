import time  # simulate delays
import numpy as np  # for IK perturbations
from copy import deepcopy  # imported  copy module
import rclpy # imported rclpy module
from rclpy.node import Node # imported Node module
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported GoalResponse , CancelResponse module

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # imported jointTrajectory, JointTrajectoryPoint module
from sensor_msgs.msg import JointState # imported Joinstate module
from geometry_msgs.msg import Pose # imported Pose module
from control_msgs.action import FollowJointTrajectory # imported FollowjointTrajectory module
from typing import List, Optional, Union, Tuple # imported List, Optional , Union module

class InvalidTrajectory(Exception):
    """Raised when trajectory validation fails."""
    pass

class Mairakinematics(Node):

    def __init__(self):
        super().__init__('maira_kinematics')  # init ROS node

        # --- Parameters ---
        self.declare_parameter('speed_move_joint', 1.0)
        self.declare_parameter('speed_move_linear', 1.0)
        self.declare_parameter('acc_move_joint', 1.0)
        self.declare_parameter('acc_move_linear', 1.0)
        self.declare_parameter('joint_names', [
            'joint1','joint2','joint3','joint4',
            'joint5','joint6','joint7'
        ])

        # --- Retrieve parameters ---
        self.speed_move_joint = self.get_parameter('speed_move_joint').value
        self.speed_move_linear = self.get_parameter('speed_move_linear').value
        self.acc_move_joint = self.get_parameter('acc_move_joint').value
        self.acc_move_linear = self.get_parameter('acc_move_linear').value
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)
        action_name = self.get_parameter('joint_trajectory_position_controller/follow_joint_trajectory').value
        command_topic = self.get_parameter('command_topic').value

        # --- Robot interfaces
        self._robot = None  # IK/FK solver & tool manager
        self._program = None  # motion program interface
        self._id_manager = None  # ID generator
        self._elbow_checker = None  # elbow orientation checker
        self.require_elbow_up = False # setting the elbow up as False value

        # --- Publisher & Subscriber ---
        self._cmd_pub = self.create_publisher(
            JointTrajectory, command_topic, 10)
        self._current_joint_state = [0.0]*self.num_joints
        self.create_subscription(
            JointState, 'joint_states',
            self.joint_state, 10)

        # --- ActionServer ---
        self._action_server = ActionServer(
            self, FollowJointTrajectory, action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    # --- ROS callbacks ---
    def joint_state(self, msg: JointState) -> None:
        """Store latest joint positions for feedback."""
        self._current_joint_state = list(msg.position)

    def goal_callback(self, request) -> GoalResponse:
        """Accept all incoming goals."""
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept cancellation requests."""
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        """
        Process FollowJointTrajectory goals by validating and sending
        each waypoint, checking for cancel and publishing feedback.
        """
        goal_handle.accept()
        traj = goal_handle.request.trajectory
        self.get_logger().info(f"Trajectory with {len(traj.points)} points")
        try:
            self._validate_trajectory(traj.points)
        except InvalidTrajectory as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = res.INVALID_GOAL
            res.error_string = str(e)
            return res
        for idx, pt in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Canceled')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            self.publish_command(pt.positions, pt.time_from_start)
            fb = FollowJointTrajectory.Feedback()
            fb.actual.positions = self.get_current_joint_state()
            goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = res.SUCCESS
        self.get_logger().info('Done')
        return res


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

        return self._current_joint_state

    def get_current_cartesian_tcp_pose(self) -> List[float]:

        pose = self._robot_state.getRobotStatus('cartesianPosition')
        return Pose(pose[:3], pose[-4:]).to_list()


    def throw_if_trajectory_invalid(self, trajectory: List[List[float]]) -> None:
        if not isinstance(trajectory, list):
            raise TypeError(f"Trajectory must be list of length-{self.num_joints} lists")
        for js in trajectory:
            self.throw_if_joint_invalid(js)

    def throw_if_joint_invalid(self, joint_states: List[float]) -> None:
        if not (isinstance(joint_states, (list,tuple)) and len(joint_states)==self.num_joints):
            raise TypeError(f"Joint_states must be length {self.num_joints}")

    def throw_if_pose_invalid(self, pose: List[float]) -> None:
        if not (isinstance(pose, list) and len(pose)==6):
            raise TypeError('Pose must be 6 floats')

    def throw_if_list_poses_invalid(self, poses: List[List[float]]) -> None:
        if not isinstance(poses, list):
            raise TypeError('List of poses must be list')
        for p in poses:
            self.throw_if_pose_invalid(p)

    def get_ik_solution(self, goal_pose_cartesian: List[float], reference_joint_states: List[float]) -> List[float]:

        for seed in range(100):
            np.random.seed(seed)
            trial = (np.array(reference_joint_states) + np.random.randn(self.num_joints)*1e-2).tolist()
            try:
                sol = self._robot.ik_fk('ik', target_pose=goal_pose_cartesian, current_joint=trial)
            except Exception as e:
                self.get_logger().debug(f'IK err {seed}: {e}')
                continue
            if not np.any(np.isnan(sol)):
                return sol
        raise ValueError('IK solver failed')

    def get_elbow_up_ik_solution(self, goal_pose_cartesian: List[float], reference_joint_states: List[float]) -> List[float]:
        """Try elbow-up variations by offsetting joint5/6 and checking with elbow_checker."""
        deltas = [[0,0],[0,np.pi],[np.pi,0],[np.pi,np.pi]]
        for dx, dy in deltas:
            seed = deepcopy(reference_joint_states)
            seed[5] += (-dx if seed[5]>=0 else dx)
            seed[6] += (-dy if seed[6]>=0 else dy)
            try:
                sol = self.get_ik_solution(goal_pose_cartesian, seed)
                if self._elbow_checker.is_up(sol):
                    return sol
            except ValueError:
                pass
        raise ValueError('Elbow-up IK not found')

    def cartesian_2_joint(self, goal_pose_cartesian: List[float], reference_joint_states: Optional[List[float]]=None) -> List[float]:
        """Convert Cartesian pose to joint via IK (elbow-up optional)."""
        self.throw_if_pose_invalid(goal_pose_cartesian)
        ref = reference_joint_states or self.get_current_joint_state()
        if not getattr(self, 'require_elbow_up', False):
            return self.get_ik_solution(goal_pose_cartesian, ref)
        return self.get_elbow_up_ik_solution(goal_pose_cartesian, ref)


    def change_gripper_to(self, end_effector) -> None:
        """Switch the active tool to the given end effector."""
        # Skip if already active
        for t in self._robot.get_tools():
            if t['name']==end_effector.name:
                return
        params = [0]*16
        pose = end_effector.tcp_pose.to_list()
        params[1:7] = [pose[3],pose[4],pose[5],pose[0],pose[1],pose[2]]
        self._robot.set_tool(tool_name=end_effector.name, tool_params=params)

    def set_motion_till_force(self, stopping_forces=[0,0,1], reflex_mode_after_contact=True,) -> None:
        """Configure motion to stop at force threshold and reflex optionally."""
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=stopping_forces,
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=reflex_mode_after_contact)

    def set_motion_param(self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float) -> None:
        """Update the default speed/acceleration parameters."""
        self.speed_move_joint = speed_mj
        self.speed_move_linear = speed_ml
        self.acc_move_joint = acc_mj
        self.acc_move_linear = acc_ml

    def wait(self, time_s: float) -> None:
        """Blocking wait for specified seconds."""
        time.sleep(time_s)

    def is_id_successful(self, id: int, timeout=10.0) -> Tuple[bool,bool]:
        """Poll plan status until success/fail or timeout."""
        # Implementation placeholder
        return True, True

    def execute_if_successful(self, id: int) -> bool:
        """Execute planned motion if plan succeeded."""
        plan_ok, exec_ok = self.is_id_successful(id)
        if plan_ok and exec_ok:
            self._program.execute([id])
        return plan_ok

    def clear_ids(self, ids: List[int]) -> bool:
        """Clear specified plan IDs from robot buffer."""

        return True

    def finish(self) -> None:
        """Cleanup and finish program on robot."""
        self._program.finish()


def main(args=None):
    rclpy.init(args=args)
    node = Mairakinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
