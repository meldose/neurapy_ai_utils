from abc import ABC, abstractmethod
from typing import List, Tuple, Union, Optional
from neurapy_ai.utils.types import EndEffector

# created class KinematicsInterface

class KinematicsInterface(ABC):

# created function for changing gripper to
    @abstractmethod
    def change_gripper_to(self, end_effector: EndEffector):
        print("change_gripper_to not implemented!")
        raise NotImplementedError

# created function for getting joint states
    @abstractmethod
    def get_current_joint_state(self) -> List[float]:
        print("get_current_joint_state not implemented!")
        raise NotImplementedError

# created function for getting current cartesina tcp pose 
    @abstractmethod
    def get_current_cartesian_tcp_pose(self) -> List[float]:
        print("get_current_cartesian_tcp_pose not implemented!")
        raise NotImplementedError

# created function for converting joint to cartesian 
    @abstractmethod
    def move_joint_to_cartesian(self, goal_pose, speed=None, acc=None):
        print("move_joint_to_cartesian not implemented!")
        raise NotImplementedError

# created function for move_linear
    @abstractmethod
    def move_linear(self, goal_pose, speed=None, acc=None):
        print("move_cartesian not implemented!")
        raise NotImplementedError

# created function for wait 
    @abstractmethod
    def wait(self, time_s):
        print("wait not implemented")
        raise NotImplementedError

# created function for cartesain to joint motion
    @abstractmethod
    def cartesian_2_joint(self, pose, reference_joint_states=None):
        print("cartesian_2_joint not implemented")
        raise NotImplementedError

# created function for joint to joint 
    @abstractmethod
    def move_joint_to_joint(self, pose_joint, speed=None, acc=None):
        print("move_joint_to_joint")
        raise NotImplementedError

# created function for joint movement through points
    @abstractmethod
    def move_joint_via_points(self, trajectory, speed=None, acc=None):
        print("move_joint_via_points")
        raise NotImplementedError

# created function for moving linear through points
    @abstractmethod
    def move_linear_via_points(
        self,
        goal_poses,
        speed=None,
        acc=None,
        rot_speed=None,
        rot_acc=None,
        blending_radius=None,
    ):
        print("move_joint_via_points")
        raise NotImplementedError

# created function for setting the motion param
    @abstractmethod
    def set_motion_param(
        self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float
    ):
        print("set_motion")
        raise NotImplementedError

# created function for setting the motion till force 
    @abstractmethod
    def set_motion_till_force(
        self, stopping_forces: List[float], reflex_mode_after_contact: bool
    ) -> None:
        print("set_motion_till_force")
        raise NotImplementedError

# created function for excecuting      
    @abstractmethod
    def execute(self, ids: List[int], execution_feasibilities: List[bool]):
        print("execute")
        raise NotImplementedError

# created function for plan motion for joint to cartesian 
    @abstractmethod
    def plan_motion_joint_to_cartesian(
        self,
        goal_pose,
        reference_joint_states=None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        print("plan_motion_joint_to_cartesian")
        raise NotImplementedError

# created function for motion join to joint 
    @abstractmethod
    def plan_motion_joint_to_joint(
        self,
        goal_pose: List[float],
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        print("plan_motion_joint_to_joint")
        raise NotImplementedError

# created function for plan motion linear 
    @abstractmethod
    def plan_motion_linear(
        self,
        goal_pose: List[float],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        print("plan_motion_linear")
        raise NotImplementedError

# created function for plan motion linear though points
    @abstractmethod
    def plan_motion_linear_via_points(
        self,
        goal_poses: List[List[float]],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
        blending_radius: Optional[float] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        print("plan_motion_linear_via_points")
        raise NotImplementedError

# created function for plan motion joints through pointss
    @abstractmethod
    def plan_motion_joint_via_points(
        self,
        trajectory: List[List[float]],
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        print("plan_motion_joint_via_points")
        raise NotImplementedError

# created function for finish
    @abstractmethod
    def finish(
        self,
    ) -> None:
        print("finish")
        raise NotImplementedError

# created function for clearing the ids
    @abstractmethod
    def clear_ids(
        self,
        ids: List[int],
    ) -> bool:
        print("clear_ids")
        raise NotImplementedError

#created class DummyKinematics

class DummyKinematics(KinematicsInterface):
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints

# created function set motion param
    def set_motion_param(
        self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float
    ):
        print("set_motion_param")
        message = (
            "speed_mj: "
            + speed_mj
            + " speed_ml: "
            + speed_ml
            + " acc_mj: "
            + acc_mj
            + " acc_ml: "
            + acc_ml
        )
        print(message)

# created function for gettingt the current joint state

    def get_current_joint_state(self) -> List[float]:
        current_joint_state = [0.0] * self.num_joints
        print("current_joint_state: ", str(current_joint_state))
        return current_joint_state

# created function for moving joint state to state

    def move_joint_to_joint(self, pose_joint, speed=None, acc=None):
        print("move_joint_to_joint")

# created function for moving joint to cartesian
    def move_joint_to_cartesian(self, goal_pose, speed=None, acc=None):
        print("move_joint_to_cartesian: ", str(goal_pose))
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)

# created function for move_linear
    def move_linear(self, goal_pose, speed=None, acc=None):
        print("move_linear: " + str(goal_pose))
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)

# created function for joints through points
    def move_joint_via_points(self, trajectory, speed=None, acc=None):
        print("move_joint_via_points: " + str(trajectory))
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)

# created function for move linear through points
    def move_linear_via_points(self, trajectory, speed=None, acc=None):
        print("move_joint_via_points: " + str(trajectory))
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)

# created function for wait
    def wait(self, time_s):
        print("dummy wait ", time_s)

# created function for cartesian to joint
    def cartesian_2_joint(self, pose, reference_joint_states=None):
        return [0] * self.num_joints

# created function for executing 
    def execute(self, ids: List[int], execution_feasibilities: List[bool]):
        print(
            f"dummy execute ids {str(ids)} and execution_feasibilities {str(execution_feasibilities)}"
        )

# created function for  moiton joint to cartesian
    def plan_motion_joint_to_cartesian(
        self,
        goal_pose,
        reference_joint_states=None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        print(
            f"plan_motion_joint_to_cartesian: \n goal_pose: {str(goal_pose)} \n reference_joint_states: {str(reference_joint_states)} \n start_joint_states : {str(start_joint_states)}"
        )
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)
        return True, 1, [0] * self.num_joints
    
# created function for joint to joint
    def plan_motion_joint_to_joint(
        self,
        goal_pose: List[float],
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        print(
            f"plan_motion_joint_to_joint: \n goal_pose: {str(goal_pose)} \n start_joint_states : {str(start_joint_states)}"
        )
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)
        return True, 2, [0] * self.num_joints

# created function for motion linear
    def plan_motion_linear(
        self,
        goal_pose: List[float],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed=None,
        acc=None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        print(
            f"plan_motion_joint_to_cartesian: \n goal_pose: {str(goal_pose)} \n start_cartesian_pose: {str(start_cartesian_pose)} \n start_joint_states : {str(start_joint_states)}"
        )
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)
        return True, 3, [0] * self.num_joints

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
        print(
            f"plan_motion_linear_via_points: \n goal_poses: {str(goal_poses)} \n start_cartesian_pose: {str(start_cartesian_pose)} \n start_joint_states : {str(start_joint_states)}"
        )
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)
        return True, 4, [0] * self.num_joints


# created function for motion joint through points 
    def plan_motion_joint_via_points(
        self,
        trajectory: List[List[float]],
        start_joint_states: Union[List[float], None] = None,
        speed: float = None,
        acc: float = None,
        reusable=False,
    ) -> Tuple[bool, int, List[float]]:
        print(
            f"plan_motion_joint_via_points: \n trajectory: {str(trajectory)} \n start_joint_states : {str(start_joint_states)}"
        )
        message = (
            "speed: "
            + (speed if speed else "default")
            + " acc: "
            + (acc if acc else "default")
        )
        print(message)
        return True, 5, [0] * self.num_joints
