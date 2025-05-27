#!/usr/bin/env python3
import threading
import time
from copy import deepcopy
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, PoseArray

import numpy as np

from neura_apps.gui_program.program import Program
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.robot import Robot
from neurapy.state_flag import cmd
from neurapy.utils import CmdIDManager
from neurapy_ai.clients.database_client import DatabaseClient
from neurapy_ai.utils.types import Pose as NeuraPose, EndEffector
from neurapy_ai_utils.functions.utils import init_logger
from neurapy_ai_utils.robot.elbow_checker import ElbowChecker


class ThreadSafeCmdIDManager:
    """Thread-safe wrapper around CmdIDManager."""

    def __init__(self, id_manager: CmdIDManager = None):
        self._lock = threading.Lock()
        self._mgr = id_manager or CmdIDManager()

    def update_id(self) -> int:
        """
        Atomically increment and return the new ID.
        """
        with self._lock:
            return self._mgr.update_id()


class MairaKinematics(Node):
    """Access methods to plan and execute motions via the MAiRA control API."""

    def __init__(
        self,
        speed_move_joint: int = 20,
        speed_move_linear: float = 0.1,
        rot_speed_move_linear: float = 0.87266463,
        acc_move_joint: int = 20,
        acc_move_linear: float = 0.1,
        rot_acc_move_linear: float = 1.74532925,
        blending_radius: float = 0.005,
        require_elbow_up: bool = True,
        id_manager: Optional[CmdIDManager] = None,
        robot_handler: Optional[Robot] = None,
    ):
        super().__init__("maira_kinematics")
        self._logger = self.get_logger()
        self._logger.info("Initializing MairaKinematics node")

        # Motion parameters
        self.speed_move_joint = speed_move_joint
        self.acc_move_joint = acc_move_joint
        self.speed_move_linear = speed_move_linear
        self.acc_move_linear = acc_move_linear
        self.blending_radius = blending_radius
        self.require_elbow_up = require_elbow_up

        # Thread-safe command ID manager
        self._id_manager = ThreadSafeCmdIDManager(id_manager=id_manager)

        # Robot interface
        self._robot = robot_handler or Robot()
        self._robot_state = RobotStatus(self._robot)
        self._program = Program(self._robot)
        self.num_joints = self._robot.dof

        if self.require_elbow_up:
            self._elbow_checker = ElbowChecker(
                dof=self.num_joints,
                robot_name=self._robot.robot_name,
            )

        self._database_client = DatabaseClient()

        # Publishers
        self.joint_publish = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_mjc_res = self.create_publisher(Bool, 'move_joint_to_cartesian/result', 10)
        self.pub_mjj_res = self.create_publisher(Bool, 'move_joint_to_joint/result', 10)
        self.pub_ml_res = self.create_publisher(Bool, 'move_linear/result', 10)
        self.pub_mlp_res = self.create_publisher(Bool, 'move_linear_via_points/result', 10)
        self.pub_mjv_res = self.create_publisher(Bool, 'move_joint_via_points/result', 10)
        self.pub_ctj = self.create_publisher(JointState, 'cartesian_to_joint_state', 10)
        self.pub_execute_if_successful = self.create_publisher(Bool, 'execute_if_successful/result', 10)
        self.pub_ik_solution = self.create_publisher(JointState, 'get_ik_solution/result', 10)
        self.pub_elbow_up_ik_solution = self.create_publisher(JointState, 'get_elbow_up_ik_solution/result', 10)
        self.pub_set_motion_till_force = self.create_publisher(Bool, 'set_motion_till_force/result', 10)
        self.pub_plan_mjj = self.create_publisher(String, 'plan_motion_joint_to_joint/result', 10)
        self.pub_plan_ml = self.create_publisher(String, 'plan_motion_linear/result', 10)
        self.pub_plan_mlp = self.create_publisher(String, 'plan_motion_linear_via_points/result', 10)
        self.pub_plan_mjv = self.create_publisher(String, 'plan_motion_joint_via_points/result', 10)

        # Subscriptions
        self.create_subscription(Bool, 'get_current_joint_state', self._get_current_joint_state_callback, 10)
        self.create_subscription(Bool, 'get_current_cartesian_pose', self._get_current_cartesian_pose_callback, 10)
        self.create_subscription(Int32, 'execute_if_successful', self._execute_if_successful_callback, 10)
        self.create_subscription(Pose, 'get_ik_solution', self._get_ik_solution_callback, 10)
        self.create_subscription(Pose, 'get_elbow_up_ik_solution', self._get_elbow_up_ik_solution_callback, 10)
        self.create_subscription(Float32MultiArray, 'set_motion_till_force', self.set_motion_till_force, 10)

        # Movement callbacks
        self.create_subscription(Pose, 'move_unified_pose', self.unified_pose_callback, 10)
        self.create_subscription(Pose, 'move_joint_to_cartesian', self.move_joint_to_cartesian_callback, 10)
        self.create_subscription(JointState, 'move_joint_to_joint', self.move_joint_to_joint_callback, 10)
        self.create_subscription(Pose, 'move_linear', self.move_linear_callback, 10)
        self.create_subscription(PoseArray, 'move_linear_via_points', self.move_linear_via_points_callback, 10)
        self.create_subscription(JointTrajectory, 'move_joint_via_points', self.move_joint_via_points_callback, 10)
        self.create_subscription(Int32MultiArray, 'execute_ids', self.execute_callback, 10)
        self.create_subscription(JointState, 'plan_motion_joint_to_joint', self.plan_motion_joint_to_joint_callback, 10)
        self.create_subscription(Pose, 'plan_motion_linear', self.plan_motion_linear_callback, 10)
        self.create_subscription(PoseArray, 'plan_motion_linear_via_points', self.plan_motion_linear_via_points_callback, 10)
        self.create_subscription(JointTrajectory, 'plan_motion_joint_via_points', self.plan_motion_joint_via_points_callback, 10)
        self.create_subscription(Pose, 'cartesian_to_joint_state', self.cartesian_2_joint_callback, 10)

    # ---- Core IK and state methods ----
    def _is_id_successful(self, cmd_id: int) -> (bool, bool):
        # TODO: implement real check against self._program or self._robot_state
        return True, True

    def _get_current_joint_state_callback(self, _: Bool) -> None:
        state = self._get_current_joint_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = state
        self.pub_current_joint_state.publish(msg)

    def _get_current_cartesian_pose_callback(self, _: Bool) -> None:
        pose = self._get_current_cartesian_pose()
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = pose[:3]
        # Orientation packed as quaternion
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = pose[3:]
        self.pub_current_cartesian_pose.publish(msg)

    # ---- Movement callback implementations ----
    def move_joint_to_cartesian_callback(self, msg: Pose) -> None:
        target = [
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]
        res = self.move_joint_to_cartesian(target)
        self.pub_mjc_res.publish(Bool(data=res))
        self._logger.info(f"move_joint_to_cartesian → {res}")

    def move_joint_to_joint_callback(self, msg: JointState) -> None:
        res = self.move_joint_to_joint(msg.position)
        self.pub_mjj_res.publish(Bool(data=res))
        self._logger.info(f"move_joint_to_joint → {res}")

    def move_linear_callback(self, msg: Pose) -> None:
        target = [
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]
        res = self.move_linear(target)
        self.pub_ml_res.publish(Bool(data=res))
        self._logger.info(f"move_linear → {res}")

    def move_linear_via_points_callback(self, msg: PoseArray) -> None:
        poses = [
            [p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in msg.poses
        ]
        res = self.move_linear_via_points(poses)
        self.pub_mlp_res.publish(Bool(data=res))
        self._logger.info(f"move_linear_via_points → {res}")

    def move_joint_via_points_callback(self, msg: JointTrajectory) -> None:
        traj = [pt.positions for pt in msg.points]
        res = self.move_joint_via_points(traj)
        self.pub_mjv_res.publish(Bool(data=res))
        self._logger.info(f"move_joint_via_points → {res}")

    def execute_callback(self, msg: Int32MultiArray) -> None:
        ids = list(msg.data)
        feas = [True] * len(ids)
        self.execute(ids, feas)
        self._logger.info(f"execute ids {ids}")

    # ---- Planning callbacks ----
    def plan_motion_joint_to_joint_callback(self, msg: JointState) -> None:
        ok, pid, last = self.plan_motion_joint_to_joint(msg.position)
        self.pub_plan_mjj.publish(String(data=f"{ok},{pid},{last}"))

    def plan_motion_linear_callback(self, msg: Pose) -> None:
        ok, pid, last = self.plan_motion_linear([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        self.pub_plan_ml.publish(String(data=f"{ok},{pid},{last}"))

    def plan_motion_linear_via_points_callback(self, msg: PoseArray) -> None:
        poses = [[p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                 for p in msg.poses]
        ok, pid, last = self.plan_motion_linear_via_points(poses)
        self.pub_plan_mlp.publish(String(data=f"{ok},{pid},{last}"))

    def plan_motion_joint_via_points_callback(self, msg: JointTrajectory) -> None:
        traj = [pt.positions for pt in msg.points]
        ok, pid, last = self.plan_motion_joint_via_points(traj)
        self.pub_plan_mjv.publish(String(data=f"{ok},{pid},{last}"))

    def cartesian_2_joint_callback(self, msg: Pose) -> None:
        target = [
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.position = self.cartesian_2_joint(target)
        self.pub_ctj.publish(js)
        self._logger.info(f"cartesian_to_joint → {js.position}")


    def move_joint_to_cartesian(self, goal_pose: List[float]) -> bool:
        raise NotImplementedError("Core move_joint_to_cartesian logic not yet implemented")

    def move_joint_to_joint(self, joint_states: List[float]) -> bool:
        raise NotImplementedError("Core move_joint_to_joint logic not yet implemented")

    def move_linear(self, goal_pose: List[float]) -> bool:
        raise NotImplementedError("Core move_linear logic not yet implemented")

    def move_linear_via_points(self, poses: List[List[float]]) -> bool:
        raise NotImplementedError("Core move_linear_via_points logic not yet implemented")

    def move_joint_via_points(self, traj: List[List[float]]) -> bool:
        raise NotImplementedError("Core move_joint_via_points logic not yet implemented")

    def _get_current_joint_state(self) -> List[float]:
        return self._robot_state.getRobotStatus("jointAngles")

    def _get_current_cartesian_pose(self) -> List[float]:
        pose_quat = self._robot_state.getRobotStatus("cartesianPosition")
        return NeuraPose(pose_quat[:3], pose_quat[-4:]).to_list()

    def cartesian_2_joint(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: Optional[List[float]] = None,
    ) -> List[float]:
        if not isinstance(goal_pose_cartesian, list) or len(goal_pose_cartesian) != 7:
            raise TypeError("goal_pose_cartesian must be a list of length 7 (x,y,z,qx,qy,qz,qw)")

        if reference_joint_states is None:
            reference_joint_states = self._get_current_joint_state()

        if not self.require_elbow_up:
            return self._get_ik_solution(goal_pose_cartesian, reference_joint_states)
        else:
            return self._get_elbow_up_ik_solution(goal_pose_cartesian, reference_joint_states)

    def _get_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float],
    ) -> List[float]:
        solution = []
        for i in range(100):
            np.random.seed(i)
            dummy = reference_joint_states + np.random.randn(self.num_joints) * 1e-2
            try:
                sol = self._robot.ik_fk("ik", target_pose=goal_pose_cartesian, current_joint=dummy)
            except Exception:
                continue
            if not np.any(np.isnan(sol)):
                return sol.tolist() if hasattr(sol, 'tolist') else sol
        raise ValueError("IK solution not found")

    def _get_elbow_up_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float],
    ) -> List[float]:
        deltas = [[0.0, 0.0], [0.0, np.pi], [np.pi, 0.0], [np.pi, np.pi]]
        for i in deltas:
            seed = deepcopy(reference_joint_states)
            seed[5] += i[0] if reference_joint_states[5] < 0 else -i[0]
            seed[6] += i[1] if reference_joint_states[6] < 0 else -i[1]
            try:
                sol = self._get_ik_solution(goal_pose_cartesian, seed)
                if self._elbow_checker.is_up(sol):
                    return sol
            except Exception:
                continue
        raise ValueError("Could not find elbow up solution")

    def set_motion_till_force(self, msg: Float32MultiArray) -> None:
        forces = list(msg.data)
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=forces,
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=True,
        )

    def finish(self) -> None:
        MairaKinematics._ID = 30000.0
        self._program.finish()


def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
