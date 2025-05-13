import time
from copy import deepcopy
from typing import List, Optional, Tuple, Union
import threading

import numpy as np
from neura_apps.gui_program.program import Program
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.component import Component
from neurapy.robot import Robot
from neurapy.state_flag import calculation, cmd
from neurapy.utils import CmdIDManager
from neurapy_ai.clients.database_client import DatabaseClient
from neurapy_ai.utils.types import Pose, EndEffector
from neurapy_ai_utils.functions.utils import init_logger
from omniORB import CORBA

from neurapy_ai_utils.robot.kinematics_interface import KinematicsInterface
from neurapy_ai_utils.robot.elbow_checker import ElbowChecker



class ThreadSafeCmdIDManager:
    """Thread safe implementation of CmdIDManager. This class has a thread lock
    that ensures that only one process updates and read the ID at a time by
    returning the updated id immediately on update, during which the update
    method is locked.
    """
## initialise the elements ############

    def __init__(self, id_manager: CmdIDManager = None):
        """Initialise optionally with an existing id manager
        instance.

        Parameters
        ----------
        id_manager : CmdIDManager
            _description_
        """
        self.id_manager_lock = threading.Lock()
        self.id_manager = id_manager if id_manager else CmdIDManager()


# defining the function for updating the id ###########

    def update_id(self) -> int:
        """Update the command id by incrementing and return the updated id.

        Returns
        -------
        int
            Recently updated command id
        """
        self.id_manager_lock.acquire()
        self.id_manager.update_id()
        plan_id = self.id_manager.get_id()
        self.id_manager_lock.release()
        return plan_id


class MairaKinematics(KinematicsInterface):
    """Access methods to plan and execute use the MAiRA control API.

    Parameters
    ----------
    KinematicsInterface : ABC
        Interface with kinematic methods to define.
    """
 #### intialise the elements ############
 
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
        id_manager: CmdIDManager = None,
        robot_handler: Robot = None,
    ):
        """Provide wrapped function for robot motion.

        Parameters
        ----------
        speed_move_joint: int, optional
            Global joint speed for move joint in percentage. Defaults to 10.
        speed_move_linear: float, optional
            Global translation speed for move linear in m/sec. Defaults to 0.05.
        rot_speed_move_linear: float, optional
            Global linear speed of rotation for move linear in rad/sec. Defaults
            to 0.87266463.
        acc_move_joint: int, optional
            Global joint acceleration for move joint in percentage. Defaults to
            20.
        acc_move_linear: float, optional
            Global linear acceleration for move linear in m/sec2. Defaults to
            0.1.
        rot_acc_move_linear: float, optional
            Global rotational acceleration for move linear. Defaults to
            1.74532925.
        blending_radius: float, optional
            Global blending radius for blending linear motion, in m. Defaults to
            0.005.
        require_elbow_up: float, optional
            Assert IK solutions with MAiRA's elbox (Axis 4) up, defaults to
            True to return only elbow up solutions.
        id_manager: CmdIDManager, optional
            Pass an external neurapy.utils.CmdIDManager to manage planner IDs,
            else a new CmdIDManager with a default start ID of 3e4 will used.
        robot_handler: Robot, optional
            Pass existing instance of neurapy.Robot, else a new instance is created
        """
        self._logger = init_logger(name=self.__class__.__name__)
        self.speed_move_joint = speed_move_joint
        self.speed_move_linear = speed_move_linear
        self.rot_speed_move_linear = rot_speed_move_linear
        self.acc_move_joint = acc_move_joint
        self.acc_move_linear = acc_move_linear
        self.rot_acc_move_linear = rot_acc_move_linear
        self.blending_radius = blending_radius
        self.require_elbow_up = require_elbow_up
        self._id_manager = ThreadSafeCmdIDManager(id_manager=id_manager)

        self._robot = Robot() if not robot_handler else robot_handler
        self._robot_state = RobotStatus(self._robot)
        self._program = Program(self._robot)

        self.num_joints = self._robot.dof
        if require_elbow_up:
            self._elbow_checker = ElbowChecker(
                dof=self.num_joints, robot_name=self._robot.robot_name
            )

        self._database_client = DatabaseClient()

###### Defining the function for changing the gripper #########

    def change_gripper_to(self, end_effector: EndEffector) -> None:
        """Choose gripper to control.

        Parameters
        ----------
        end_effector : neurapy_ai.utils.types.EndEffector
            End effector parameters as saved on database.

        """
        # Set tool only if not already set
        for tool in self._robot.get_tools():
            if tool["name"] == end_effector.name: # checking if the tool name is there not 
                return

        tcp_pose = end_effector.tcp_pose.to_list()
        tool_params = [0] * 16
        tool_params[1] = tcp_pose[3]
        tool_params[2] = tcp_pose[4]
        tool_params[3] = tcp_pose[5]
        tool_params[4] = tcp_pose[0]
        tool_params[5] = tcp_pose[1]
        tool_params[6] = tcp_pose[2]

        self._robot.set_tool(
            tool_name=end_effector.name, tool_params=tool_params # setting the tool name
        )

###  defining function fot motion program ###

    def set_motion_param(
        self, speed_mj: float, speed_ml: float, acc_mj: float, acc_ml: float
    ) -> None:
        """Set the default motion speed and acceleration for move joint and
        move linear.

        Parameters
        ----------
        speed_mj : float
            Speed for move joint, from 0 - 100
        speed_ml : float
            Speed for move linear, m/sec
        acc_mj : float
            Acceleration for move joint, from 0 - 100
        acc_ml : float
            Acceleration for move linear, m/sec2

        """
        self.speed_move_joint = speed_mj # setting the move_joint 
        self.speed_move_linear = speed_ml # setting the move_linear 
        
        self.acc_move_joint = acc_mj # setting the accleration to move_joint
        self.acc_move_linear = acc_ml # setting the acceleration to move_linear 
        
### defining function for wait ######


    def wait(self, time_s: float) -> None:
        """Wait for given time

        Parameters
        ----------
        time_s: float
            Waiting time in seconds

        Raises
        ------
        RuntimeError:
            If action failed

        """
        time.sleep(time_s) # setting time sleep for wait 

### defining function for throw if trajectory invalid ####

    def _throw_if_trajectory_invalid(
        self, trajectory: List[List[float]]
    ) -> None:
        """Throw error if given trajectory is not valid.

        Parameters
        ----------
        trajectory : List[List[float]]
            Joint trajectory

        """
        if not (isinstance(trajectory, list)): # if not reuired trajectory raise the error 
            raise TypeError(
                f"[ERROR] trajectory should be a List[List[float*\
                    {self.num_joints}]]!"
            )
        for joint_states in trajectory: # checking the joint states in the trajectory 
            self._throw_if_joint_invalid(joint_states)

#### function for if joint invalid ###########

    def _throw_if_joint_invalid(self, joint_states: List[float]) -> None:
        """Throw error if given joint states is not valid.

        Parameters
        ----------
        joint_states : List[float]
            Joint trajectory

        """
        if isinstance(joint_states, tuple): # checking if the join states are tuple or not , convert them into list 
            joint_states = list(joint_states)

        if not (
            (isinstance(joint_states, list)) # if in the list , checking if the lenght of the joint states are eual to the number of joint
            and len(joint_states) == self.num_joints
        ):
            raise TypeError(
                f"[ERROR] joint_states should be a list with length \
                    {self.num_joints}!"
            )


####### funnction for throw if pose invalud ##########

    def _throw_if_pose_invalid(self, pose: List[float]) -> None:
        """Throw error if given pose is not valid.

        Parameters
        ----------
        joint_states : List[float]
            Joint trajectory

        """
        if not ((isinstance(pose, list)) and len(pose) == 6): # checking if the pose is in the list and len of the pose is zero or not
            raise TypeError("[ERROR] goal_pose should be a list with length 6!") # raise the error

### function for throw if list pose is invlaid ##########

    def _throw_if_list_poses_invalid(self, goal_poses: List[List[float]]):
        """Throw error if given list of poses is not valid.

        Parameters
        ----------
        joint_states : List[List[float]]
            Joint trajectory

        """
        if not isinstance(goal_poses, list): # checking if the goal pose are in the list or not 
            raise TypeError(
                "[ERROR] goal_pose should be a List[List[float*6]]!"
            )
        for pose in goal_poses: # checking the goal_poses
            self._throw_if_pose_invalid(pose)


###### function for speed to percent ########

    # TODO not really used and implemented!
    def _speed_to_percent(self, speed_mps):
        if speed_mps is None: # if the speed_mps is None 
            speed_mps = self.speed_move_joint # setting the speeds_mps as speed_move_joint 
        return 50

###### function for acceleration to percent ########

    # TODO not really used and implemented!
    def _acc_to_percent(self, acc): 
        if acc is None: # if the acceleration is None 
            acc = self.acc_move_joint # setting the acceleration to move_to_joint
        return 50 # returns acceleration as 50


#### function for gettting curretn joint state ############

    def _get_current_joint_state(self) -> List[float]:
        """Return current joint states.

        Return
        ------
        List[float]
            Joint states

        """
        return self._robot_state.getRobotStatus("jointAngles") # returning the robot state wiht Joint Angles 


######## functin for getting curretn cartesian pose ###############

    def _get_current_cartesian_pose(self) -> List[float]:
        """Return current cartesian pose of the active TCP of the robot.

        Returns
        -------
        List[float]
            6D pose of the TCP as list of 6 floats [x, y, z, rx, ry, rz]

        """
        pose_quat = self._robot_state.getRobotStatus("cartesianPosition") # setting the robot status as cartesian position
        return Pose(pose_quat[:3], pose_quat[-4:]).to_list() # return the pose with X,Y,Z  and qx,qy,qz and qw 


 ######## function to execute if succesful or not  ###############
 
    def _execute_if_successful(self, id: int) -> bool:
        """Execute trajectory for given id if successfull.

        Parameters
        ----------
        id : int
            Id of trajectory to be executed

        Returns
        -------
        bool
            Returns True if execution successfull. False, otherwise.

        """
        plan_success, execution_feasibility = self._is_id_successful(id) # if exectue is successful set the plan_success
        self._logger.debug(
            "MairaKinematics::execute_if_successful: plan_success: "
            + str(plan_success)
            + " execution_feasibility: "
            + str(execution_feasibility)
            + " id: "
            + str(id)
        )
        if plan_success and execution_feasibility: # if success then 
            self._logger.debug("*" * 40)
            self._logger.debug(f"execute id {id}")
            self._program.execute([id])
        self._logger.info(
            f"After execute id: {id}. joint state is: \
                {self._get_current_joint_state()}"
        )
        return plan_success # return the plan_success

    def get_current_joint_state(self) -> List[float]:
        """Get the current joint state as a list of floats

        Returns
        -------
        List[float]
            Current joint positions
        """
        return self._get_current_joint_state() # getting the current joint state 


##### function for getting the current cartesian tcp pose ##########

    def get_current_cartesian_tcp_pose(self) -> List[float]:
        """Get the current cartesian tcp pose

        Returns
        -------
        List[float]
            Current Cartesian TCP pose as [x, y, z, r, p, y]
        """ 
        return self._get_current_cartesian_pose() # getting the current cartesian pose 
    
 ########### function for defining the finish ########
 
    def finish(self) -> None:
        """Delete planned motion buffer from NeuraPy. Only call this when
        program is going to be stopped."""
        MairaKinematics._ID = 3e4
        self._program.finish()

### defining function for cartesian to joint ############

    def cartesian_2_joint(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: Optional[List[float]] = None,
    ) -> List[float]:
        """Get the inverse kinematics for a given pose and current joint state

        Parameters
        ----------
        goal_pose_cartesian : List[float]
            Cartesian pose of tcp frame
        reference_joint_states : Optional[List[float]], optional
            Joint positions to seed IK solver. Default is None, to use current
            joint positions.

        Returns
        -------
        List[float]
            Joint positions from IK solution.

        Raises
        ------
        TypeError
            Wrong input type

        ValueError
            IK solver failed

        """
        if not isinstance(goal_pose_cartesian, list): # checking if the goal pose cartesian is in the list or not 
            raise TypeError("cartesian_pose must be of type list")

        self._throw_if_pose_invalid(goal_pose_cartesian) 

        reference_joint_states = (
            self._get_current_joint_state()
            if reference_joint_states is None
            else reference_joint_states
        ) # setting the current joint states
        if not self.require_elbow_up:
            return self._get_ik_solution(
                goal_pose_cartesian=goal_pose_cartesian,
                reference_joint_states=reference_joint_states,
            ) # return the ik solution
        else:
            return self._get_elbow_up_ik_solution(
                goal_pose_cartesian=goal_pose_cartesian,
                reference_joint_states=reference_joint_states,
            ) # getting the elbow_up_ik solution


###### defining function for gettig the ik solution ###########

    def _get_ik_solution(
        self,
        goal_pose_cartesian: List[float], # setting the goal_pose _cartesian
        reference_joint_states: List[float], # setting the reference joint states
    ) -> List[float]:
        """Get the inverse kinematics for a given cartesion pose and reference joint states.

        Parameters
        ----------
        goal_pose_cartesian : list[float]
            TCP goal pose in cartesian as a list.
        reference_joint_states: list[float]
            Reference joint states to seed the IK solution for the given pose.

        Returns
        -------
        List[float]
            Joint positions from IK solution.

        Raises
        ------
        ValueError
            IK solver failed

        """
        solution = []
        for i in range(100):
            np.random.seed(i)
            dummy_array = (
                reference_joint_states + np.random.randn(self.num_joints) * 1e-2
            )
            try:
                solution = self._robot.ik_fk(
                    "ik",
                    target_pose=goal_pose_cartesian,
                    current_joint=dummy_array,
                )
            except Exception as e:
                self._logger.debug(e)
                self._logger.debug(f"No IK solution found after {i} attempts")
                continue
            if not np.any(np.isnan(solution)):
                return solution
        else:
            raise ValueError(f"IK solution return nan.")


 ###########function foir getting the elbow up ik solution ###############
 
    def _get_elbow_up_ik_solution(
        self,
        goal_pose_cartesian: List[float],
        reference_joint_states: List[float],
    ) -> List[float]:
        """Get the inverse kinematics for a given cartesion pose and reference joint states.

        Parameters
        ----------
        goal_pose_cartesian : list[float]
            TCP goal pose in cartesian as a list.
        reference_joint_states: list[float]
            Reference joint states to seed the IK solution for the given pose.

        Returns
        -------
        List[float]
            Joint positions from IK solution.

        Raises
        ------
        ValueError
            IK solver failed.

        """
        retry_delta_angle_states = [
            [0.0, 0.0],
            [0.0, np.pi],
            [np.pi, 0.0],
            [np.pi, np.pi],
        ]
        for retry_delta_angle_state in retry_delta_angle_states:
            seed_joint = deepcopy(reference_joint_states)
            seed_joint[5] = (
                reference_joint_states[5] + retry_delta_angle_state[0]
                if reference_joint_states[5] < 0.0
                else reference_joint_states[5] - retry_delta_angle_state[0]
            )
            seed_joint[6] = (
                reference_joint_states[6] + retry_delta_angle_state[1]
                if reference_joint_states[6] < 0.0
                else reference_joint_states[6] - retry_delta_angle_state[1]
            )
            try:
                solution = self._get_ik_solution(
                    goal_pose_cartesian=goal_pose_cartesian,
                    reference_joint_states=seed_joint,
                )
                if self._elbow_checker.is_up(solution):
                    self._logger.debug("Elbow up!")
                    return solution
            except:
                pass
        else:
            self._logger.debug("Elbow down!")
            raise ValueError("Could not find elbow up solution")


###### defining the function for motion till force ##########

    def set_motion_till_force(
        self, 
        stopping_forces: List[float]=[0.0, 0.0, 1.0], 
        reflex_mode_after_contact: bool=True
    ) -> None:
        """
        Set the motion behavior for the robot to stop based on forces, with an option for reflex mode after contact.

        This function configures the robot to continue moving until the specified stopping forces are reached. 
        Optionally, it can activate reflex mode after contact is detected.

        Parameters
        ----------
        stopping_forces : List[float], optional
            A list of three values representing the stopping forces for motion. The default is [0.0, 0.0, 1.0].
            These values define the force threshold at which the robot will stop its motion.
        reflex_mode_after_contact : bool, optional
            A flag to indicate whether reflex mode should be activated after contact. The default is True.
            If set to True, reflex mode will be activated once the stopping forces are detected.

        Returns
        -------
        None
        """
        self._robot.set_go_till_forces_mode_for_next_spline(
            stopping_forces=stopping_forces, 
            stopping_force_multiplier=1.0,
            activate_reflex_mode_after_contact=reflex_mode_after_contact
        )


##### defining function for move joint to cartesian

    def move_joint_to_cartesian(
        self,
        goal_pose: List[float], # setting the goal_pose 
        reference_joint_states: Union[List[float], None] = None, # setting the joint states
        speed: Optional[int] = None, # setting the speed
        acc: Optional[int] = None, # setting the acceleration
    ) -> bool:
        """Execute move joint action with given goal, speed and acceleration

        Parameters
        ----------
        goal_pose: list[float]
            TCP goal pose in cartesian as a list
        reference_joint_states: Union[List[float],None]
            Reference joint states to seed the IK solution for the given pose.
            Defaults to None, to use the current joint positions.
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.

        Returns
        -------
        bool
            Move command success

        Raises
        ------
        TypeError
            If get wrong input type
        RuntimeError
            If action failed
        """
        self._throw_if_pose_invalid(goal_pose)

        joint_pose = self.cartesian_2_joint(goal_pose, reference_joint_states) # converting the cartesian to joint 
        return self.move_joint_to_joint(joint_pose, speed, acc) # returning the move joint to joint 



### defining the function for moving joint to joint ############

    def move_joint_to_joint(
        self,
        goal_pose: List[float], # setting the goal pose 
        speed: Optional[int] = None, # setting the speed for the movement 
        acc: Optional[int] = None, # setting the acceleration for the robot movement 
    ) -> bool: 
        """Execute move joint action with given goal, speed and acceleration

        Parameters
        ----------
        goal_pose: list[float]
            Goal joint positions
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.

        Returns
        -------
        bool
            Move command success

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """
        self._throw_if_joint_invalid(goal_pose)

        joint_property = {
            "target_joint": [goal_pose],
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "interpolator": 1,
            "enable_blending": True,
        }

        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **joint_property,
            cmd_id=plan_id,
            current_joint_angles=self._get_current_joint_state(),
            reusable_id=0,
        )
        return self._execute_if_successful(id=plan_id)


########## defining function for move_linear ##############

    def move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
    ) -> bool:
        """Execute move linear action with given goal, speed and acceleration

        Parameters
        ----------
        goal_pose: list
            TCP goal pose in cartesian as a list
        speed: Optional[float], optional
            Translation speed in m/sec. Defaults to None, to use the class'
            default value.
        acc: Optional[float], optional
            Translation acceleration in m2/sec. Defaults to None, to use the
            class' default value.
        rot_speed: Optional[float], optional
            Rotation speed in rad/sec. Defaults to None, to use the class'
            default value.
        rot_acc: Optional[float], optional
            Rotation acceleration in rad/sec2. Defaults to None, to use the
            class' default value.

        Returns
        -------
        bool
            Move command success

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """
        self._throw_if_pose_invalid(goal_pose)

        linear_property = {
            "target_pose": [self._get_current_cartesian_pose(), goal_pose],
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "blending": False,
            "blend_radius": 0.0,
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **linear_property,
            cmd_id=plan_id,
            current_joint_angles=self._get_current_joint_state(),
            reusable_id=0,
        )
        return self._execute_if_successful(id=plan_id)

## defining function for move linear through point s##########

    def move_linear_via_points(
        self,
        goal_poses: List[List[float]], # setting the  poses
        speed: Optional[float] = None,# setting the  speed
        acc: Optional[float] = None,# setting the acc
        rot_speed: Optional[float] = None,# setting the  rot speed
        rot_acc: Optional[float] = None,# setting the rot_acc
        blending_radius: Optional[float] = None,# setting the blending radius
    ) -> bool:
        """Execute move linear via points action with given goal, speed and
        acceleration

        Parameters
        ----------
        goal_poses: list
            TCP goal cartesian poses list
        speed: Optional[float], optional
            Translation speed in m/sec. Defaults to None, to use the class'
            default value.
        acc: Optional[float], optional
            Translation acceleration in m/sec2. Defaults to None, to use the
            class' default value.
        rot_speed : Optional[float], optional
            Rotational speed in rad/sec. Defaults to None, to use the
            class' default value.
        rot_acc : Optional[float], optional
            Rotational acceleration in rad/sec2. Defaults to None, to use the
            class' default value.
        blending_radius : Optional[float], optional
            Blending radius in m. Defaults to None, to use the
            class' default value.

        Returns
        -------
        bool
            Move command success

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """
        self._throw_if_list_poses_invalid(goal_poses)

        goal_poses.insert(0, self._get_current_cartesian_pose())
        linear_property = {
            "target_pose": goal_poses,
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "blend_radius": (
                0.01 if (blending_radius is None) else blending_radius
            ),
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **linear_property,
            cmd_id=plan_id,
            current_joint_angles=plan_id,
            reusable_id=0,
        )
        return self._execute_if_successful(id=plan_id) # return the execution if successful 
    
### defining function for moving joint via points #########

    def move_joint_via_points(
        self,
        trajectory: List[List[float]],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """
        Execute move joint via points action with given joint trajectory, speed
        and acceleration

        THIS WILL NOT BE A BLEND MOVEMENT AS NEURAPY DO NOT SUPPORT IT YET FOR
        JOINT INTERPOLATION!

        Parameters
        ----------
        trajectory : List[List[float]]
            Joint trajectory
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.

        Returns
        -------
        bool
            Move command success

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """
        self._throw_if_trajectory_invalid(trajectory)

        joint_property = {
            "target_joint": trajectory,
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "interpolator": 1,
            "enable_blending": True,
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **joint_property,
            cmd_id=plan_id,
            current_joint_angles=self._get_current_joint_state(),
            reusable_id=0,
        )
        return self._execute_if_successful(id=plan_id)


## defining function for executing motion for ids  #######

    def execute(self, ids: List[int], execution_feasibilities: List[bool]):
        """Execute the motion for given id when the plan is succeed.

        Parameters
        ----------
        ids : List[int]
            A list of id of the planned motion to be executed
        execution_feasibilities : List[bool]
            A List of flag indicate the execution feasibility, false will be jumped over

        """
        self._logger.debug(
            "MairaKinematics: Execute: execution_feasibility"
            + str(execution_feasibilities)
            + "ID: "
            + str(ids)
        )
        executable_ids = [
            id
            for id, execution_feasibility in zip(ids, execution_feasibilities)
            if execution_feasibility == True
        ]
        self._logger.info(f"Executable ids: {executable_ids}")
        self._program.execute(executable_ids)

    def plan_motion_joint_to_cartesian(
        self,
        goal_pose,
        reference_joint_states: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[bool, int, List[float]]:
        """Plan motion for move joint action with given goal, speed and acceleration

        Parameters
        ----------
        goal_pose: list
            TCP goal pose in cartesian as a list
        reference_joint_states: Union[List[float],None]
            Reference joint states to seed the IK solution for the given pose.
            Defaults to None, to use the current joint positions.
        start_joint_states: Union[List[float],None]
            Start joint states. Defaults to None, to use the current joint
            positions.
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.
        reusable: Optional[bool], optional
            Reuse the planned ID for repeated executions. Defaults to False,
            the plan will be executable only once.

        Returns
        -------
        Tuple[Tuple[bool,bool], int, List[float]]
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed
        """
        if start_joint_states is None: # if the start joint states is None 
            start_joint_states = self._get_current_joint_state() # setting the joint states with the current joint states

        self._throw_if_pose_invalid(goal_pose)
        self._throw_if_joint_invalid(start_joint_states)

        try:
            joint_pose = self.cartesian_2_joint(
                goal_pose, reference_joint_states
            )
        except ValueError as e:
            self._logger.error(e)
            return False, None, []

        return self.plan_motion_joint_to_joint(
            goal_pose=joint_pose,
            start_joint_states=start_joint_states,
            speed=speed,
            acc=acc,
            reusable=reusable,
        )
        
### defining function for plan motion from joint to joint ######

    def plan_motion_joint_to_joint(
        self,
        goal_pose: List[float],
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        """Plan motion for move joint action with given goal, speed and
        acceleration.

        Parameters
        ----------
        goal_pose: list[float]
            Goal joint positions
        start_joint_states: Union[List[float],None]
            Start joint states. Defaults to None, to use the current joint
            positions.
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.
        reusable: Optional[bool], optional
            Reuse the planned ID for repeated executions. Defaults to False,
            the plan will be executable only once.

        Returns
        -------
        Tuple[Tuple[bool,bool], int, List[float]]
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed
        """

        if start_joint_states is None: # if the start joint states is None 
            start_joint_states = self._get_current_joint_state() # getting the current joint states

        self._throw_if_joint_invalid(goal_pose) 
        self._throw_if_joint_invalid(start_joint_states)

        if start_joint_states is None:
            start_joint_states = self._get_current_joint_state()

        joint_property = {
            "target_joint": [goal_pose],
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "interpolator": 1,
            "enable_blending": True,
        }

        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **joint_property,
            cmd_id=plan_id,
            current_joint_angles=start_joint_states,
            reusable_id=1 if reusable else 0,
        )
        success_flags = self._is_id_successful(plan_id)
        last_joint_state = None
        if all(success_flags):
            last_joint_state = self._program.get_last_joint_configuration(
                plan_id
            )
        return (
            success_flags,
            plan_id,
            last_joint_state,
        )


#### defining function for moiton linear #######

    def plan_motion_linear(
        self,
        goal_pose: List[float],
        start_cartesian_pose: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        """Plan motion for move linear action with given goal, speed and
        acceleration.

        Parameters
        ----------
        goal_pose: List[float]
            TCP goal cartesian pose as list
        start_cartesian_pose: Union[List[float],None]
            Start tcp cartesian pose relative to the base frame. Defaults to
            None, to use the current tcp pose.
        start_joint_states: Union[List[float],None]
            Start joint states. Defaults to None, to use the current joint
            positions.
        speed: Optional[float], optional
            Translation speed in m/sec. Defaults to None, to use the class'
            default value.
        acc: Optional[float], optional
            Translation acceleration in m2/sec. Defaults to None, to use the
            class' default value.
        reusable : Optional[bool], optional
            Reuse the planned ID for repeated executions. Defaults to False, the
            plan will be executable only once.

        Returns
        -------
        Tuple[Tuple[bool, bool], int, List[float]]
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed

        """

        if not all([start_cartesian_pose, start_joint_states]): # checking start cartesian pose and start joint states
            # Set start state to current pose only if neither of
            # start pose nor start joint state are set
            start_cartesian_pose = self._get_current_cartesian_pose() # getting the current cartesian pose 
            start_joint_states = self._get_current_joint_state() # getting the curretn joint state

        self._throw_if_pose_invalid(goal_pose) 
        self._throw_if_pose_invalid(start_cartesian_pose)
        self._throw_if_joint_invalid(start_joint_states)

        linear_property = {
            "target_pose": [start_cartesian_pose, goal_pose],
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "blending": False,
            "blend_radius": 0.0,
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **linear_property,
            cmd_id=plan_id,
            current_joint_angles=start_joint_states,
            reusable_id=1 if reusable else 0,
        )
        success_flags = self._is_id_successful(plan_id)
        last_joint_state = None
        if all(success_flags):
            last_joint_state = self._program.get_last_joint_configuration(
                plan_id
            )
        return (
            
            success_flags,
            plan_id,
            last_joint_state,
        )

#### defining function for plan motion through points########

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
        """Plan motion for move linear via points action with given goal, speed
        and acceleration.

        Parameters
        ----------
        goal_poses: list
            TCP goal cartesian poses list
        start_cartesian_pose: Union[List[float],None]
            Start tcp cartesian pose relative to the base frame. Defaults to
            None, to use the current tcp pose.
        start_joint_states: Union[List[float],None]
            Start joint states. Defaults to None, to use the current joint
            positions.
        speed: Optional[float], optional
            Translation speed in m/sec. Defaults to None, to use the class'
            default value.
        acc: Optional[float], optional
            Translation acceleration in m/sec2. Defaults to None, to use the
            class' default value.
        rot_speed : Optional[float], optional
            Rotational speed in rad/sec. Defaults to None, to use the
            class' default value.
        rot_acc : Optional[float], optional
            Rotational acceleration in rad/sec2. Defaults to None, to use the
            class' default value.
        blending_radius : Optional[float], optional
            Blending radius in m. Defaults to None, to use the
            class' default value.
        reusable : Optional[bool], optional
            Reuse the planned ID for repeated executions. Defaults to False, the
            plan will be executable only once.

        Returns
        -------
        Tuple[Tuple[bool, bool], int, List[float]]
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed

        """
        if not all([start_cartesian_pose, start_joint_states]): # checking if the it is start cartesian posea and start joint states
            # Set start state to current pose only if neither of
            # start pose nor start joint state are set
            start_cartesian_pose = self._get_current_cartesian_pose() # getting the current the cartesian poses
            start_joint_states = self._get_current_joint_state() # getting the current joint state 

        self._throw_if_list_poses_invalid(goal_poses)
        self._throw_if_pose_invalid(start_cartesian_pose)
        self._throw_if_joint_invalid(start_joint_states)

        goal_poses.insert(0, self._get_current_cartesian_pose())
        linear_property = {
            "target_pose": goal_poses,
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "blend_radius": (
                0.01 if (blending_radius is None) else blending_radius
            ),
        }

        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Linear,
            **linear_property,
            cmd_id=plan_id,
            current_joint_angles=start_joint_states,
            reusable_id=1 if reusable else 0,
        )
        success_flags = self._is_id_successful(plan_id)
        last_joint_state = None
        if all(success_flags):
            last_joint_state = self._program.get_last_joint_configuration(
                plan_id
            )
        return (
            success_flags,
            plan_id,
            last_joint_state,
        )
        
## defining function for planning motion through points########

    def plan_motion_joint_via_points(
        self,
        trajectory: List[List[float]],
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        """
        Plan motion for move joint via points action with given joint
        trajectory, speed and acceleration.

        THIS WILL NOT BE A BLEND MOVEMENT AS NEURAPY DO NOT SUPPORT IT YET FOR
        JOINT INTERPOLATION!

        Parameters
        ----------
        trajectory : List[List[float]]
            Joint trajectory
        start_joint_states: Union[List[float],None]
            Start joint states. Defaults to None, to use the current joint
            positions.
        speed: Optional[int], optional
            Joint speed as percentage value [0, 100]. Defaults to None, to use
            the class' default value.
        acc: Optional[int], optional
            Joint acceleration as percentage value [0, 100]. Defaults to None,
            to use the class' default value.
        reusable: Optional[bool], optional
            Reuse the planned ID for repeated executions. Defaults to False,
            the plan will be executable only once.

        Returns
        -------
        Tuple[Tuple[bool,bool], int, List[float]]
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed
        """
        if start_joint_states is None: # if the start joint state is None 
            start_joint_states = self._get_current_joint_state() # getting the current joint state

        self._throw_if_trajectory_invalid(trajectory)
        self._throw_if_joint_invalid(start_joint_states)

        joint_property = {
            "target_joint": trajectory,
            "speed": self.speed_move_joint if speed is None else speed,
            "acceleration": self.acc_move_joint if acc is None else acc,
            "interpolator": 1,
            "enable_blending": True,
        }
        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **joint_property,
            cmd_id=plan_id,
            current_joint_angles=start_joint_states,
            reusable_id=1 if reusable else 0,
        )
        success_flags = self._is_id_successful(plan_id)
        last_joint_state = None
        if all(success_flags):
            last_joint_state = self._program.get_last_joint_configuration(
                plan_id
            )
        return (
            success_flags,
            plan_id,
            last_joint_state,
        )

#### defining function for clearign the ids ############

    def clear_ids(self, ids: List[int]) -> bool: 
        """Clear given plan IDs from memory stack to prevent overload.

        Parameters
        ----------
        ids : List[int]
            Plan IDs

        Returns
        -------
        bool
            Clearing success

        """
        rts = Component(self._robot, "RTS")
        corba_cmd_id = CORBA.Any(
            CORBA.TypeCode("IDL:omg.org/CORBA/DoubleSeq:1.0"), ids
        )
        success = rts.callService("clearSplineId", [corba_cmd_id]) == 0 # setting the success rate 
        if not success: # if not success raise the error 
            self._logger.warning(f"Fail to delete ids {ids}")
        return success
    
#### defining the function for checking the id is succesfull or not #############

    def _is_id_successful(
        self, plan_id: int, timeout: float = 10.0
    ) -> Tuple[bool, bool]:
        """Check if planning for given id is successful.

        Parameters
        ----------
        plan_id : int
            Plan ID to check.
        timeout : float, optional
            Timeout to check for plan success in sec, by default 10.0

        Returns
        -------
        Tuple[bool, bool]
            Tuple of flags for plan success and execution feasibility

        Raises
        ------
        Exception
            Given ID does not exist.

        """
        try:
            t_start = time.time() # setting the start time  
            status = self._program.get_plan_status(plan_id) # setting the status 
            while (
                status != calculation.Failed # if the status is not Failed 
                and status != calculation.Success # if the status si not success
                and (time.time() - t_start) < timeout # checking if the time is less than the actual timeout 
            ):
                time.sleep(0.01)
                status = self._program.get_plan_status(plan_id)
            if status == calculation.Success: # if the status is sucess
                return True, True # return True 
            else: 
                return False, False # else return false condition
        except Exception as ex:
            self._logger.error(f"Motion Id {plan_id} doesn't exist: {str(ex)}") # raise the Exception values 
            raise ex
