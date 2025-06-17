#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Empty, Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
import cmd  # external command definitions

class InvalidTrajectory(Exception):
    """Raised when trajectory validation fails."""
    pass

class MairaKinematics(Node):
    def __init__(self, node_name: str = 'maira_kinematics'):
        super().__init__(node_name)

        # --- Parameters ---
        self.declare_parameter('speed_move_joint', 1.0)
        self.declare_parameter('speed_move_linear', 1.0)
        self.declare_parameter('acc_move_joint', 1.0)
        self.declare_parameter('acc_move_linear', 1.0)
        self.declare_parameter('ik_max_retries', 100)
        self.declare_parameter('joint_names', [
            'joint1','joint2','joint3','joint4','joint5','joint6','joint7'
        ])
        self.declare_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory',
            '/joint_trajectory_position_controller/follow_joint_trajectory'
        )
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('require_elbow_up', False)

        # Load parameters
        self.speed_move_joint = self.get_parameter('speed_move_joint').value
        self.speed_move_linear = self.get_parameter('speed_move_linear').value
        self.acc_move_joint = self.get_parameter('acc_move_joint').value
        self.acc_move_linear = self.get_parameter('acc_move_linear').value
        self.ik_max_retries = self.get_parameter('ik_max_retries').value
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)
        self.require_elbow_up = self.get_parameter('require_elbow_up').value

        # Dynamic updates
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        traj_action_name = self.get_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory'
        ).value
        cmd_topic = self.get_parameter('command_topic').value

        # --- Robot interfaces (to inject externally) ---
        self._robot = None
        self._elbow_checker = None
        self._program = None
        self._id_manager = None

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(JointTrajectory, cmd_topic, 10)

        # Current joint state cache
        self._current_joint_state = [0.0] * self.num_joints
        self.create_subscription(JointState, 'joint_states', self._joint_state_callback, 10)

        # Request-response topics
        self._joints_pub = self.create_publisher(JointState, 'current_joint_state', 10)
        self.create_subscription(Empty, 'request_current_joint_state', self._request_joints_callback, 10)

        self._tcp_pub = self.create_publisher(Float64MultiArray, 'current_tcp_pose', 10)
        self.create_subscription(Empty, 'request_current_tcp_pose', self._request_tcp_callback, 10)

        self._cart2joint_pub = self.create_publisher(JointState, 'cartesian_to_joint_response', 10)
        self.create_subscription(Float64MultiArray, 'cartesian_to_joint', self._cart2joint_callback, 10)

        # Motion command topics
        self._set_force_pub = self.create_publisher(Bool, 'set_motion_till_force_response', 10)
        self.create_subscription(Float64MultiArray, 'set_motion_till_force', self._set_force_callback, 10)

        self._move_jtc_pub = self.create_publisher(Bool, 'move_joint_to_cartesian_response', 10)
        self.create_subscription(Float64MultiArray, 'move_joint_to_cartesian', self._move_jtc_callback, 10)

        self._move_jtj_pub = self.create_publisher(Bool, 'move_joint_to_joint_response', 10)
        self.create_subscription(Float64MultiArray, 'move_joint_to_joint', self._move_jtj_callback, 10)

        self._move_linear_pub = self.create_publisher(Bool, 'move_linear_response', 10)
        self.create_subscription(Float64MultiArray, 'move_linear', self._move_linear_callback, 10)

        self._move_lin_via_pub = self.create_publisher(Bool, 'move_linear_via_points_response', 10)
        self.create_subscription(Float64MultiArray, 'move_linear_via_points', self._move_lin_via_callback, 10)

        self._move_jnt_via_pub = self.create_publisher(Bool, 'move_joint_via_points_response', 10)
        self.create_subscription(Float64MultiArray, 'move_joint_via_points', self._move_jnt_via_callback, 10)

        # --- Trajectory ActionServer ---
        self._traj_server = ActionServer(
            self, FollowJointTrajectory, traj_action_name,
            execute_callback=self._execute_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel
        )

    # --- Parameter updates ---
    def _on_parameters_changed(self, params):
        for param in params:
            if param.name == 'speed_move_joint':
                self.speed_move_joint = param.value
            elif param.name == 'speed_move_linear':
                self.speed_move_linear = param.value
            elif param.name == 'acc_move_joint':
                self.acc_move_joint = param.value
            elif param.name == 'acc_move_linear':
                self.acc_move_linear = param.value
            elif param.name == 'ik_max_retries':
                self.ik_max_retries = param.value
        return SetParametersResult(successful=True)

    # --- Helpers ---
    @staticmethod
    def _duration_to_sec(dur: Duration) -> float:
        return dur.sec + dur.nanosec * 1e-9

    def _joint_state_callback(self, msg: JointState):
        self._current_joint_state = list(msg.position)

    # --- Request-Response Callbacks ---
    def _request_joints_callback(self, _: Empty):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self._current_joint_state
        self._joints_pub.publish(msg)

    def _request_tcp_callback(self, _: Empty):
        try:
            pose = self.get_current_cartesian_tcp_pose()
            arr = Float64MultiArray()
            arr.data = pose
            self._tcp_pub.publish(arr)
        except Exception as e:
            self.get_logger().error(f'TCP pose error: {e}')


    def _cart2joint_callback(self, msg: Float64MultiArray):
        try:
            sol = self.cartesian_2_joint(list(msg.data))
            resp = JointState()
            resp.header.stamp = self.get_clock().now().to_msg()
            resp.name = self.joint_names
            resp.position = sol
            self._cart2joint_pub.publish(resp)
        except Exception as e:
            self.get_logger().error(f'IK error: {e}')


    # --- Motion Callbacks ---
    def _set_force_callback(self, msg: Float64MultiArray):
        try:
            self.set_motion_till_force(list(msg.data))
            self._set_force_pub.publish(Bool(data=True))
        except Exception as e:
            self.get_logger().error(f'Force cmd error: {e}')

            self._set_force_pub.publish(Bool(data=False))

    def _move_jtc_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_joint_to_cartesian(list(msg.data))
            self._move_jtc_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error(f'Move J->C error: {e}')

            self._move_jtc_pub.publish(Bool(data=False))

    def _move_jtj_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_joint_to_joint(list(msg.data))
            self._move_jtj_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error(f'Move J->J error: {e}')

            self._move_jtj_pub.publish(Bool(data=False))

    def _move_linear_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_linear(list(msg.data))
            self._move_linear_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error(f'Move linear error: {e}')

            self._move_linear_pub.publish(Bool(data=False))

    def _move_lin_via_callback(self, msg: Float64MultiArray):
        try:
            data = list(msg.data)
            poses = [data[i:i+6] for i in range(0, len(data), 6)]
            ok = self.move_linear_via_points(poses)
            self._move_lin_via_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error(f'LinearVia error: {e}')

            self._move_lin_via_pub.publish(Bool(data=False))

    def _move_jnt_via_callback(self, msg: Float64MultiArray):
        try:
            data = list(msg.data)
            pts = [data[i:i+self.num_joints] for i in range(0, len(data), self.num_joints)]
            ok = self.move_joint_via_points(pts)
            self._move_jnt_via_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error(f'JointVia error: {e}')

            self._move_jnt_via_pub.publish(Bool(data=False))

    # --- Action Callbacks ---
    def _accept_goal(self, request) -> GoalResponse:
        self.get_logger().info('FollowJointTrajectory goal received')
        return GoalResponse.ACCEPT

    def _accept_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('FollowJointTrajectory cancel requested')
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle) -> FollowJointTrajectory.Result:
        traj = goal_handle.request.trajectory
        # Validate
        try:
            self._validate_trajectory(traj.points)
        except InvalidTrajectory as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = res.INVALID_GOAL
            res.error_string = str(e)
            return res

        start_time = self.get_clock().now()
        rate = self.create_rate(100)
        for pt in traj.points:
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                res = FollowJointTrajectory.Result()
                res.error_code = res.CANCELED
                res.error_string = ''
                return res
            target_sec = self._duration_to_sec(pt.time_from_start)
            while self._duration_to_sec(self.get_clock().now() - start_time) < target_sec:
                if goal_handle.is_cancel_requested():
                    goal_handle.canceled()
                    res = FollowJointTrajectory.Result()
                    res.error_code = res.CANCELED
                    res.error_string = ''
                    return res
                rclpy.spin_once(self, timeout_sec=0)
                rate.sleep()

            self.publish_command(pt.positions, pt.time_from_start)
            fb = FollowJointTrajectory.Feedback()
            fb.actual.positions = self._current_joint_state
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = res.SUCCESS
        res.error_string = ''
        return res

    # --- Core methods ---
    def publish_command(self, positions, tfs: Duration) -> None:
        if self._program is None:
            raise RuntimeError('Program not configured')
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = tfs
        msg.points = [pt]
        self.cmd_pub.publish(msg)

    def _validate_trajectory(self, points):
        if not points:
            raise InvalidTrajectory('No points')
        last_sec = None
        for i, pt in enumerate(points):
            if len(pt.positions) != self.num_joints:
                raise InvalidTrajectory(f'Bad point length at index {i}')
            cur_sec = self._duration_to_sec(pt.time_from_start)
            if last_sec is not None and cur_sec <= last_sec:
                raise InvalidTrajectory(f'Time non-monotonic at point {i}')
            last_sec = cur_sec

    def get_current_cartesian_tcp_pose(self) -> list:
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        status = self._robot.getRobotStatus('cartesianPosition')
        return list(status)

    def cartesian_2_joint(self, goal: list, ref=None) -> list:
        if not isinstance(goal, list) or len(goal) not in (6, 7):
            raise TypeError('Pose length must be 6 or 7')
        seed = ref or self._current_joint_state
        if self.require_elbow_up:
            return self._get_elbow_up_ik(goal, seed)
        return self._get_ik(goal, seed)

    def _get_ik(self, pose, seed) -> list:
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        for i in range(self.ik_max_retries):
            rng = np.random.RandomState(seed=i)
            trial = seed + rng.randn(self.num_joints) * 1e-2
            try:
                sol = self._robot.ik_fk('ik', target_pose=pose, current_joint=trial)
            except Exception:
                continue
            if not np.any(np.isnan(sol)):
                return sol
        self.get_logger().warn(f'IK failed after {self.ik_max_retries} retries')
        raise RuntimeError('IK failed')

    def _get_elbow_up_ik(self, pose, seed) -> list:
        if self._robot is None or self._elbow_checker is None:
            raise RuntimeError('IK/elbow not configured')
        offsets = [(0,0), (0,np.pi), (np.pi,0), (np.pi,np.pi)]
        for dx, dy in offsets:
            trial = seed.copy()
            trial[5] += -dx if trial[5] >= 0 else dx
            trial[6] += -dy if trial[6] >= 0 else dy
            try:
                sol = self._get_ik(pose, trial)
                if self._elbow_checker.is_up(sol):
                    return sol
            except Exception:
                continue
        raise RuntimeError('Elbow-up IK failed')

    def set_motion_till_force(self, forces=None, reflex=True):
        f = forces if forces else [0.0, 0.0, 1.0]
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=f,
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=reflex
        )

    def move_joint_to_cartesian(self, goal, ref=None) -> bool:
        jp = self.cartesian_2_joint(goal, ref)
        return self.move_joint_to_joint(jp)

    def move_joint_to_joint(self, goal) -> bool:
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {
            'target_joint': [goal],
            'speed': self.speed_move_joint,
            'acceleration': self.acc_move_joint,
            'interpolator': 1,
            'enable_blending': True
        }
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Joint, **prop, cmd_id=pid,
                                   current_joint_angles=self._current_joint_state,
                                   reusable_id=0)
        return self.is_id_successful(pid)[0]

    def move_linear(self, goal) -> bool:
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {
            'target_pose': [self.get_current_cartesian_tcp_pose(), goal],
            'speed': self.speed_move_linear,
            'acceleration': self.acc_move_linear,
            'blending': False,
            'blend_radius': 0.0
        }
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Linear, **prop, cmd_id=pid,
                                   current_joint_angles=self._current_joint_state,
                                   reusable_id=0)
        return self.is_id_successful(pid)[0]

    def move_linear_via_points(self, goals) -> bool:
        poses = [self.get_current_cartesian_tcp_pose()] + goals
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {
            'target_pose': poses,
            'speed': self.speed_move_linear,
            'acceleration': self.acc_move_linear,
            'blend_radius': 0.01
        }
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Linear, **prop, cmd_id=pid,
                                   current_joint_angles=self._current_joint_state,
                                   reusable_id=0)
        return self.is_id_successful(pid)[0]

    def move_joint_via_points(self, traj) -> bool:
        if (not isinstance(traj, list) or
            any(len(pt) != self.num_joints for pt in traj)):
            raise TypeError('Invalid joint trajectory')
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {
            'target_joint': traj,
            'speed': self.speed_move_joint,
            'acceleration': self.acc_move_joint,
            'interpolator': 1,
            'enable_blending': True
        }
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Joint, **prop, cmd_id=pid,
                                   current_joint_angles=self._current_joint_state,
                                   reusable_id=0)
        return self.is_id_successful(pid)[0]

    def is_id_successful(self, cmd_id: int):
        # stub: integrate real status checking
        return True, None


def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
