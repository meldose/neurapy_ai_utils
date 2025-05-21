from typing import List, Tuple, Union

from neurapy_ai_utils.grippers import get_gripper_module
from neurapy_ai_utils.grippers.gripper_interface import (
    DummyGripper,
    GripperInterface,
)
from neurapy_ai_utils.robot.kinematics_interface import KinematicsInterface
from neurapy_ai.utils.types import EndEffector

# created class Robot

class Robot:
    def __init__(
        self,
        kinematics: KinematicsInterface,
        gripper: GripperInterface = DummyGripper(),
    ):
        self.gripper = gripper
        self.kinematics = kinematics

# created function for cahnging gripper to 
    def change_gripper_to(self, end_effector: EndEffector):
        self.gripper = get_gripper_module(
            end_effector.name, end_effector.neura_typename
        )(name=end_effector.name)
        self.kinematics.change_gripper_to(end_effector)

# created function for set motion program
    def set_motion_param(
        self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float
    ):
        self.kinematics.set_motion_param(speed_mj, speed_ml, acc_mj, acc_ml)

# created function for getting the current joint state
    def get_current_joint_state(self) -> List[float]:
        return self.kinematics.get_current_joint_state()

# created function for getting the curretn cartesian tcp pose
    def get_current_cartesian_tcp_pose(self) -> List[float]:
        return self.kinematics.get_current_cartesian_tcp_pose()

# created function for move linear
    def move_linear(self, tcp_pose, speed=None, acc=None):
        self.kinematics.move_linear(tcp_pose, speed, acc)

# created function for join to cartesian
    def move_joint_to_cartesian(
        self, tcp_pose, reference_joint_states=None, speed=None, acc=None
    ):
        self.kinematics.move_joint_to_cartesian(
            tcp_pose, reference_joint_states, speed, acc
        )

# created function for moving the join to joint
    def move_joint_to_joint(self, tcp_pose, speed=None, acc=None):
        self.kinematics.move_joint_to_joint(tcp_pose, speed, acc)

# created function for  joint through points
    def move_joint_via_points(self, trajectory, speed=None, acc=None):
        self.kinematics.move_joint_via_points(trajectory, speed, acc)

# created function for linear through joints
    def move_linear_via_points(
        self, goal_poses, speed=None, acc=None, blending_radius=None
    ):
        self.kinematics.move_linear_via_points(
            goal_poses, speed, acc, blending_radius=blending_radius
        )

# created function for wait 
    def wait(self, time_s):
        self.kinematics.wait(time_s)


# created function for execute
    def execute(self, ids: List[int], execution_feasibilities: List[bool]):
        return self.kinematics.execute(
            ids=ids, execution_feasibilities=execution_feasibilities
        )

# created function for plan motion joint to cartesain
    def plan_motion_joint_to_cartesian(
        self,
        goal_pose,
        reference_joint_states=None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        return self.kinematics.plan_motion_joint_to_cartesian(
            goal_pose=goal_pose,
            reference_joint_states=reference_joint_states,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            reusable=reusable,
        )

# created function for plan motion for joint to joint
    def plan_motion_joint_to_joint(
        self,
        goal_pose: List[float],
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        return self.kinematics.plan_motion_joint_to_joint(
            goal_pose=goal_pose,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            reusable=reusable,
        )

# created function for plan motin linear
    def plan_motion_linear(
        self,
        goal_pose: List[float],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        return self.kinematics.plan_motion_linear(
            goal_pose=goal_pose,
            start_cartesian_pose=start_cartesian_pose,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            reusable=reusable,
        )

# created function for motion linear through points
    def plan_motion_linear_via_points(
        self,
        goal_poses: List[List[float]],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        rot_speed=None,
        rot_acc=None,
        blending_radius=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        return self.kinematics.plan_motion_linear_via_points(
            goal_poses=goal_poses,
            start_cartesian_pose=start_cartesian_pose,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            rot_speed=rot_speed,
            rot_acc=rot_acc,
            blending_radius=blending_radius,
            reusable=reusable,
        )

# created function for motion joints through points
    def plan_motion_joint_via_points(self,
        trajectory: List[List[float]],
        start_joint_states: Union[List[float], None] = None,
        speed: float = None,
        acc: float = None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        return self.kinematics.plan_motion_joint_via_points(
            trajectory=trajectory,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            reusable=reusable,
        )

# created function for set motion till force 
    def set_motion_till_force(
        self, 
        stopping_forces: List[float]=[0.0, 0.0, 1.0], 
        reflex_mode_after_contact: bool=True
    ) -> None:
        self.kinematics.set_motion_till_force( 
            stopping_forces=stopping_forces, 
            reflex_mode_after_contact=reflex_mode_after_contact
        )
