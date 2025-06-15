#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import AsyncIOExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from moveit_commander import MoveGroupCommander
from typing import List, Optional
from tf_transformations import quaternion_from_euler
import numpy as np
import time
import asyncio
from copy import deepcopy


from your_robot_sdk import ProgramInterface, RobotInterface, RobotStateInterface, ElbowChecker

class MairaKinematics(Node):
    def __init__(self):
        super().__init__('maira_kinematics')


        self.declare_parameter('cartesian_pose', [0.0]*6)

        self.group = MoveGroupCommander('arm')

        self._program = ProgramInterface()
        self._robot = RobotInterface()
        self._robot_state = RobotStateInterface()
        self._elbow_checker = ElbowChecker()
        self.require_elbow_up = False

        # --- Publishers for convenience services ---
        self.pub_mjc = self.create_publisher(Bool, 'move_joint_to_cartesian/result', 10)
        self.pub_mjj = self.create_publisher(Bool, 'move_joint_to_joint/result', 10)
        self.pub_ml  = self.create_publisher(Bool, 'move_linear/result', 10)
        self.pub_mlp = self.create_publisher(Bool, 'move_linear_via_points/result', 10)
        self.pub_mjv = self.create_publisher(Bool, 'move_joint_via_points/result', 10)

        # --- Subscribers for convenience services ---
        self.create_subscription(Pose,        'move_joint_to_cartesian', self._on_move_cartesian, 10)
        self.create_subscription(JointState, 'move_joint_to_joint',     self._on_move_joint,     10)
        self.create_subscription(Pose,        'move_linear',             self._on_move_linear,    10)
        self.create_subscription(PoseArray,   'move_linear_via_points',  self._on_move_linear_via_points, 10)
        self.create_subscription(FollowJointTrajectory, 'move_joint_via_points', self._on_move_joint_via_points, 10)
        self.create_subscription(Int32MultiArray,       'execute_ids',           self._on_execute_ids,    10)

        # Action Server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )


    @staticmethod
    def _throw_if_joint_invalid(joint_states: List[float], num_joints: int) -> None:
        if isinstance(joint_states, tuple):
            joint_states = list(joint_states)
        if not (isinstance(joint_states, list) and len(joint_states) == num_joints):
            raise TypeError(f"[ERROR] joint_states must be list of length {num_joints}")

    @staticmethod
    def _throw_if_pose_invalid(pose: List[float]) -> None:
        if not (isinstance(pose, list) and len(pose) == 6):
            raise TypeError("[ERROR] pose must be list of length 6")


    def _get_current_joint_state(self) -> List[float]:
        return self._robot_state.getRobotStatus("jointAngles")

    def get_current_joint_state(self) -> List[float]:
        return self._get_current_joint_state()

    # --- Convenience Service Callbacks ---
    def _on_move_cartesian(self, msg: Pose) -> None:
        cart = [msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        joint_tgt = self._cartesian_to_joint(cart)
        ok = self._move_joint(joint_tgt)
        self.pub_mjc.publish(Bool(data=ok))

    def _on_move_joint(self, msg: JointState) -> None:
        ok = self._move_joint(list(msg.position))
        self.pub_mjj.publish(Bool(data=ok))

    def _on_move_linear(self, msg: Pose) -> None:
        cart = [msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        ok = self._move_linear(cart)
        self.pub_ml.publish(Bool(data=ok))

    def _on_move_linear_via_points(self, msg: PoseArray) -> None:
        poses = [[p.position.x,p.position.y,p.position.z,
                  p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
                 for p in msg.poses]
        ok = self._move_linear_via_points(poses)
        self.pub_mlp.publish(Bool(data=ok))

    def _on_move_joint_via_points(self, msg: FollowJointTrajectory) -> None:
        trajs = [pt.positions for pt in msg.points]
        ok = self._move_joint_via_points(trajs)
        self.pub_mjv.publish(Bool(data=ok))

    def _on_execute_ids(self, msg: Int32MultiArray) -> None:
        ids = list(msg.data)
        feas = [True]*len(ids)
        ok = self._execute_ids(ids, feas)
        self.get_logger().info(f"execute_ids {ids} -> {ok}")

    # --- FollowJointTrajectory Callbacks ---
    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info('Received new trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):

        try:
            curr = self.get_current_joint_state()
            self.get_logger().info(f"[Action] Current joints: {curr}")
        except Exception as e:
            self.get_logger().warn(f"Failed read state: {e}")


        goal = goal_handle.request
        num_j = len(goal.trajectory.joint_names)
        try:
            for pt in goal.trajectory.points:
                self._throw_if_joint_invalid(pt.positions, num_j)
        except Exception as e:
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return goal_handle.abort(res)

        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = goal.trajectory.joint_names
        prev = 0.0
        for idx, pt in enumerate(goal.trajectory.points):
            if goal_handle.is_cancel_requested:
                res = FollowJointTrajectory.Result()
                res.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return goal_handle.cancel(res)
            tfs = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
            await asyncio.sleep(max(0.0, tfs - prev))
            prev = tfs
            fb.desired = pt
            fb.actual = pt
            fb.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"[Action] step {idx+1}/{len(goal.trajectory.points)}")

  
        cart = self.get_parameter('cartesian_pose').get_parameter_value().double_array_value
        self.get_logger().info(f"[Action] linear to {cart}")
        try:
            self._throw_if_pose_invalid(cart)
            if not self._move_linear(cart):
                raise RuntimeError("move_linear failed")
            self.wait(1.0)
        except Exception:
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            return goal_handle.abort(res)


        try:
            self._execute_if_successful(0)
            self.finish()
        except Exception:
            pass


        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return goal_handle.succeed(res)


    def _move_joint(self, joints: List[float]) -> bool:
        self._throw_if_joint_invalid(joints, len(joints))
        try:
            self._program.execute(joints)
            return True
        except Exception as e:
            self.get_logger().error(f"_move_joint error: {e}")
            return False

    def _move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None
    ) -> bool:
        self._throw_if_pose_invalid(goal_pose)
        kwargs = {}
        if speed is not None:
            kwargs['velocity_scaling_factor'] = speed/100.0
        if acc is not None:
            kwargs['acceleration_scaling_factor'] = acc/100.0
        q = quaternion_from_euler(goal_pose[3],goal_pose[4],goal_pose[5])
        ros_pose = Pose()
        ros_pose.position = Point(x=goal_pose[0],y=goal_pose[1],z=goal_pose[2])
        ros_pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        try:
            plan,fraction = self.group.compute_cartesian_path([ros_pose],0.01,0.0,True)
            self.get_logger().info(f"frac: {fraction}")
            if kwargs:
                plan = self.group.retime_trajectory(self.group.get_current_state(),plan,**kwargs)
            return bool(self.group.execute(plan,wait=True))
        except Exception as e:
            self.get_logger().error(f"_move_linear error: {e}")
            return False

    def _move_linear_via_points(self, poses: List[List[float]]) -> bool:
        return all(self._move_linear(p) for p in poses)

    def _move_joint_via_points(self, trajs: List[List[float]]) -> bool:
        return all(self._move_joint(t) for t in trajs)

    def _execute_ids(self, ids: List[int], feas: List[bool]) -> bool:
        for idx, ok in zip(ids, feas):
            if not ok:
                return False
            self._program.execute([idx])
        return True

    def _execute_if_successful(self, id: int) -> bool:
        plan, feas = self._is_id_successful(id)
        if plan and feas:
            self._program.execute([id])
        return plan

    def finish(self) -> None:
        self._program.finish()

    @staticmethod
    def wait(time_s: float) -> None:
        time.sleep(time_s)


def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    executor = AsyncIOExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
