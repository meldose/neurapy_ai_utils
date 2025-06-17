import time
import cmd 
import rclpy
import numpy as np
from copy import deepcopy
from rclpy.node import Node
from typing import List, Optional,Tuple
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Empty, Float64MultiArray, Bool

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
        self.declare_parameter('joint_names', [
            'joint1','joint2','joint3','joint4','joint5','joint6','joint7'
        ])
        self.declare_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory',
            '/joint_trajectory_position_controller/follow_joint_trajectory'
        )
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('require_elbow_up', False)

        self.speed_move_joint = self.get_parameter('speed_move_joint').value
        self.speed_move_linear = self.get_parameter('speed_move_linear').value
        self.acc_move_joint = self.get_parameter('acc_move_joint').value
        self.acc_move_linear = self.get_parameter('acc_move_linear').value
        self.joint_names: List[str] = self.get_parameter('joint_names').value
        self.num_joints: int = len(self.joint_names)
        self.require_elbow_up: bool = self.get_parameter('require_elbow_up').value

        traj_action_name: str = self.get_parameter(
            'joint_trajectory_position_controller/follow_joint_trajectory'
        ).value
        cmd_topic: str = self.get_parameter('command_topic').value

        # --- Robot interfaces (inject externally) ---
        self._robot = None
        self._elbow_checker = None
        self._program = None
        self._id_manager = None

        # --- Joint command publisher & state subscriber ---
        self.cmd_pub = self.create_publisher(JointTrajectory, cmd_topic, 10)
        self._current_joint_state: List[float] = [0.0] * self.num_joints
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # --- Main trajectory ActionServer ---
        self._traj_server = ActionServer(
            self, FollowJointTrajectory, traj_action_name,
            execute_callback=self.execute_trajectory,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel
        )


        #  Current joint state
        self.create_subscription(Empty, 'request_current_joint_state', self.request_joints_callback, 10)
        self._joints_pub = self.create_publisher(JointState, 'current_joint_state', 10)

        #  Current TCP pose
        self.create_subscription(Empty, 'request_current_tcp_pose', self.request_tcp_callback, 10)
        self._tcp_pub = self.create_publisher(Float64MultiArray, 'current_tcp_pose', 10)

        #  Cartesian to joint
        self.create_subscription(Float64MultiArray, 'cartesian_to_joint_request', self.cart2joint_callback, 10)
        self._cart2joint_pub = self.create_publisher(Float64MultiArray, 'cartesian_to_joint_response', 10)

        #  Motion until force
        self.create_subscription(Float64MultiArray, 'set_motion_till_force', self.set_force_callback, 10)
        self._set_force_pub = self.create_publisher(Bool, 'set_motion_till_force_response', 10)

        #  Move joint to cartesian
        self.create_subscription(Float64MultiArray, 'move_joint_to_cartesian', self.move_jtc_callback, 10)
        self._move_jtc_pub = self.create_publisher(Bool, 'move_joint_to_cartesian_response', 10)

        #  Move joint to joint
        self.create_subscription(Float64MultiArray, 'move_joint_to_joint', self.move_jtj_callback, 10)
        self._move_jtj_pub = self.create_publisher(Bool, 'move_joint_to_joint_response', 10)

        #  Move linear
        self.create_subscription(Float64MultiArray, 'move_linear', self.move_linear_callback, 10)
        self._move_linear_pub = self.create_publisher(Bool, 'move_linear_response', 10)

        #  Move linear via points
        self.create_subscription(Float64MultiArray, 'move_linear_via_points', self.move_lin_via_callback, 10)
        self._move_lin_via_pub = self.create_publisher(Bool, 'move_linear_via_points_response', 10)

        #  Move joint via points
        self.create_subscription(Float64MultiArray, 'move_joint_via_points', self.move_jnt_via_callback, 10)
        self._move_jnt_via_pub = self.create_publisher(Bool, 'move_joint_via_points_response', 10)

    # --- Subscriber callbacks ---
    def joint_state_callback(self, msg: JointState):
        self._current_joint_state = list(msg.position)

    # --- Action callbacks ---
    def accept_goal(self, request) -> GoalResponse:
        self.get_logger().info('FollowJointTrajectory goal received')
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('FollowJointTrajectory cancel requested')
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle) -> FollowJointTrajectory.Result:
        traj = goal_handle.request.trajectory
        try:
            self.validate_trajectory(traj.points)
        except InvalidTrajectory as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = res.INVALID_GOAL
            res.error_string = str(e)
            return res

        start = self.get_clock().now()
        rate = self.create_rate(100)
        for pt in traj.points:
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            target = start + pt.time_from_start
            while self.get_clock().now() < target:
                if goal_handle.is_cancel_requested():
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                rclpy.spin_once(self, timeout_sec=0)
                rate.sleep()

            self.publish_command(pt.positions, pt.time_from_start)
            fb = FollowJointTrajectory.Feedback()
            fb.actual.positions = self._current_joint_state
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = res.SUCCESS
        return res

    # --- Topic callbacks for helpers ---
    def request_joints_callback(self, _: Empty):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self._current_joint_state
        self._joints_pub.publish(msg)

    def request_tcp_callback(self, _: Empty):
        try:
            pose = self.get_current_cartesian_tcp_pose()
            arr = Float64MultiArray()
            arr.data = pose
            self._tcp_pub.publish(arr)
        except Exception as e:
            self.get_logger().error('TCP pose error: ' + str(e))

    def cart2joint_callback(self, msg: Float64MultiArray):
        try:
            sol = self.cartesian_2_joint(list(msg.data))
            resp = Float64MultiArray()
            resp.data = sol
            self._cart2joint_pub.publish(resp)
        except Exception as e:
            self.get_logger().error('IK error: ' + str(e))

    def set_force_callback(self, msg: Float64MultiArray):
        try:
            forces = list(msg.data)
            self.set_motion_till_force(forces)
            self._set_force_pub.publish(Bool(data=True))
        except Exception as e:
            self.get_logger().error('Force cmd error: ' + str(e))
            self._set_force_pub.publish(Bool(data=False))

    def move_jtc_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_joint_to_cartesian(list(msg.data))
            self._move_jtc_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error('Move J->C error: ' + str(e))
            self._move_jtc_pub.publish(Bool(data=False))

    def move_jtj_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_joint_to_joint(list(msg.data))
            self._move_jtj_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error('Move J->J error: ' + str(e))
            self._move_jtj_pub.publish(Bool(data=False))

    def move_linear_callback(self, msg: Float64MultiArray):
        try:
            ok = self.move_linear(list(msg.data))
            self._move_linear_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error('Move linear error: ' + str(e))
            self._move_linear_pub.publish(Bool(data=False))

    def move_lin_via_callback(self, msg: Float64MultiArray):
        try:
            data = list(msg.data)
            poses = [data[i:i+6] for i in range(0, len(data), 6)]
            ok = self.move_linear_via_points(poses)
            self._move_lin_via_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error('LinearVia error: ' + str(e))
            self._move_lin_via_pub.publish(Bool(data=False))

    def move_jnt_via_callback(self, msg: Float64MultiArray):
        try:
            data = list(msg.data)
            pts = [data[i:i+self.num_joints] for i in range(0, len(data), self.num_joints)]
            ok = self.move_joint_via_points(pts)
            self._move_jnt_via_pub.publish(Bool(data=ok))
        except Exception as e:
            self.get_logger().error('JointVia error: ' + str(e))
            self._move_jnt_via_pub.publish(Bool(data=False))

    # --- Core helpers ---

# function for getting the current cartesian tcp pose
    def get_current_cartesian_tcp_pose(self) -> List[float]:
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        status = self._robot.getRobotStatus('cartesianPosition')
        return list(status)

# function for publishing the command 
    def publish_command(self, positions: List[float], tfs: Duration) -> None:
        if self._program is None:
            raise RuntimeError('Program not configured')
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = tfs
        msg.points = [pt]
        self.cmd_pub.publish(msg)

# function for validating the trajectory
    def validate_trajectory(self, points: List[JointTrajectoryPoint]) -> None:
        if not points:
            raise InvalidTrajectory('No points')
        last = None
        for i, pt in enumerate(points):
            if len(pt.positions) != self.num_joints:
                raise InvalidTrajectory(f'Bad point {i}')
            if last is not None and pt.time_from_start <= last:
                raise InvalidTrajectory(f'Time non-monotonic {i}')
            last = pt.time_from_start

# function for cartesian to joint
    def cartesian_2_joint(self, goal: List[float], ref: Optional[List[float]] = None) -> List[float]:
        if not isinstance(goal, list) or len(goal) not in (6,7):
            raise TypeError('Pose len must be 6 or 7')
        refj = ref or self._current_joint_state
        if self.require_elbow_up:
            return self.get_elbow_up_ik(goal, refj)
        return self.get_ik(goal, refj)

# function for getting the ik 
    def get_ik(self, pose: List[float], seed: List[float]) -> List[float]:
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        for i in range(100):
            np.random.seed(i)
            trial = seed + np.random.randn(self.num_joints)*1e-2
            try:
                sol = self._robot.ik_fk('ik', target_pose=pose, current_joint=trial)
            except Exception:
                continue
            if not np.any(np.isnan(sol)):
                return sol
        raise RuntimeError('IK failed')

# function for getting the elbow up ik
    def get_elbow_up_ik(self, pose: List[float], seed: List[float]) -> List[float]:
        if self._robot is None or self._elbow_checker is None:
            raise RuntimeError('IK/elbow not configured')
        tries = [(0,0),(0,np.pi),(np.pi,0),(np.pi,np.pi)]
        for dx, dy in tries:
            t = deepcopy(seed)
            t[5] += (-dx if t[5]>=0 else dx)
            t[6] += (-dy if t[6]>=0 else dy)
            try:
                sol = self.get_ik(pose, t)
                if self._elbow_checker.is_up(sol):
                    return sol
            except Exception:
                continue
        raise RuntimeError('Elbow-up IK failed')

# function for setting the motion till force 
    def set_motion_till_force(self, forces: Optional[List[float]] = None, reflex: bool = True):
        f = forces if forces else [0.0,0.0,1.0]
        if self._robot is None:
            raise RuntimeError('Robot not configured')
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=f,
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=reflex
        )

# function for move joint to cartesian 
    def move_joint_to_cartesian(self, goal: List[float], ref: Optional[List[float]] = None) -> bool:
        jp = self.cartesian_2_joint(goal, ref)
        return self.move_joint_to_joint(jp)

# function for move joint to joint
    def move_joint_to_joint(self, goal: List[float]) -> bool:
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {'target_joint': [goal], 'speed': self.speed_move_joint, 'acceleration': self.acc_move_joint, 'interpolator':1, 'enable_blending':True}
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Joint, **prop, cmd_id=pid, current_joint_angles=self._current_joint_state, reusable_id=0)
        return self.is_id_successful(pid)[0]

# function for move linear
    def move_linear(self, goal: List[float]) -> bool:
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {'target_pose': [self.get_current_cartesian_tcp_pose(), goal], 'speed': self.speed_move_linear, 'acceleration': self.acc_move_linear, 'blending':False, 'blend_radius':0.0}
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Linear, **prop, cmd_id=pid, current_joint_angles=self._current_joint_state, reusable_id=0)
        return self.is_id_successful(pid)[0]

# function for move linear through points 
    def move_linear_via_points(self, goals: List[List[float]]) -> bool:
        poses = [self.get_current_cartesian_tcp_pose()] + goals
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {'target_pose': poses, 'speed': self.speed_move_linear, 'acceleration': self.acc_move_linear, 'blend_radius':0.01}
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Linear, **prop, cmd_id=pid, current_joint_angles=self._current_joint_state, reusable_id=0)
        return self.is_id_successful(pid)[0]

# function for mobe joint via points
    def move_joint_via_points(self, traj: List[List[float]]) -> bool:
        if not isinstance(traj, list) or any(len(pt)!=self.num_joints for pt in traj):
            raise TypeError('Invalid joint trajectory')
        if self._program is None or self._id_manager is None:
            raise RuntimeError('Program not configured')
        prop = {'target_joint': traj, 'speed': self.speed_move_joint, 'acceleration': self.acc_move_joint, 'interpolator':1, 'enable_blending':True}
        pid = self._id_manager.update_id()
        self._program.set_command(cmd.Joint, **prop, cmd_id=pid, current_joint_angles=self._current_joint_state, reusable_id=0)
        return self.is_id_successful(pid)[0]

# function for checking if the id is successful or not 
    def is_id_successful(self, cmd_id: int) -> Tuple[bool, Optional[str]]:

        return True, None

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
