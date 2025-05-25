import sys
from typing import List, Tuple, Union, Optional

import moveit_commander
import rospy
import tf.transformations as tfs 
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from sensor_msgs.msg import JointState
from moveit_commander import MoveItCommanderException

from neurapy_ai_utils.robot.kinematics_interface import KinematicsInterface
from neurapy_ai_utils.functions.utils import init_logger
from neurapy_ai.utils.types import EndEffector
from neurapy_ai.utils.ros_conversions import pose_2_geometry_msg_pose, Pose


class MoveitKinematics(KinematicsInterface):
    
 #### intialise the elements ######   
    def __init__(
        self,
    ):
        self._logger = init_logger(name=self.__class__.__name__)
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        try:
            rospy.wait_for_service("/compute_ik", 2.0)
        except rospy.exceptions.ROSException as e:
            self._logger.error("Forward kinematic server not ready!")
            raise e
        self._logger.info("Forward kinematic server is ready.")
        self.ik_client = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        # init dict for savinf trajectories and ids
        self.trajectory_dict = dict()
        self.traj_id = 0
        # init paramter for cartesian path trajectory retiming
        self.kwargs = {"algorithm": "iterative_time_parameterization"}
        
##### defining the function for changing the gripper to control ################

    def change_gripper_to(self, end_effector: EndEffector) -> None:
        """Choose gripper to control.

        Parameters
        ----------
        end_effector : neurapy_ai.utils.types.EndEffector
            End effector parameters as saved on database.
        """
        # TODO set endeffector pose
        self.group.set_endeffector(end_effector.name) # setting the end effector 
        
######### defining the function for movement from joint to cartesian #############

    def move_joint_to_cartesian(
        self,
        goal_pose: List[float],
        reference_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """Execute move joint action with given goal, speed and acceleration

        Parameters
        ----------
        goal_pose: list
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

        if not (speed is None): # if the speed is not None
            self.group.set_max_velocity_scaling_factor(speed / 100) # settign the max velocity scaling factor 
        if not (acc is None):
            self.group.set_max_acceleration_scaling_factor(acc / 100) # setting the max acceleration scaling factor 

        if not isinstance(goal_pose, list): # checking if the goal_pose is in the list or not 
            raise TypeError("goal_pose must be of type list")

        try:
            pose_goal = pose_2_geometry_msg_pose(Pose.from_list(goal_pose))
            success = self.group.go(pose_goal, wait=True)
            self.group.stop()
            return success
        except MoveItCommanderException:
            joint_goal = self.cartesian_2_joint(
                goal_pose, reference_joint_states
            )
            success = self.group.go(
                joint_goal[0 : len(self.group.get_active_joints())], wait=True
            )
            self.group.stop()
            return success
        except Exception as e:
            raise RuntimeError(
                "Failed to execute move_joint_to_cartesian action: " + str(e)
            )

####### creating function for move joint to joint ############

    def move_joint_to_joint(
        self,
        pose_joint: List[float],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """Execute move joint action with given goal, speed and acceleration

        Parameters
        ----------
        pose_joint: list[float]
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

        if not (speed is None): # checking if the speed is None 
            self.group.set_max_velocity_scaling_factor(speed / 100)
        if not (acc is None):
            self.group.set_max_velocity_scaling_factor(acc / 100)

        if not isinstance(pose_joint, list):
            raise TypeError("pose_joint must be a list of joint positions")

        try:
            success = self.group.go(pose_joint, wait=True)
            self.group.stop()
            return success
        except Exception as e:
            raise RuntimeError(
                "Failed to execute move_joint_to_joint action: " + str(e)
            )
            
##### creating function for joint movement throgh points ##############

    def move_joint_via_points(
        self,
        trajectory: List[List[float]],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
    ) -> bool:
        """
        Execute move joint via points action with given joint trajectory, speed
        and acceleration

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

        if not isinstance(trajectory, list):
            raise TypeError("trajectory must be a list of joint positions")

        if not (speed is None):
            self.group.set_max_velocity_scaling_factor(speed / 100)
        if not (acc is None):
            self.group.set_max_velocity_scaling_factor(acc / 100)

        try:
            for joint_point in trajectory:
                success = self.group.go(joint_point, wait=True)
                self.group.stop()
            return success
        except Exception as e:
            raise RuntimeError(
                "Failed to execute move_joint_via_points action: " + str(e)
            )
            
####### creating fucntion for move linear through points ############

    def move_linear_via_points(
        self,
        goal_poses: List[List[float]],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
        blending_radius: Optional[float] = None,
    ) -> bool:
        """Execute move linear via points action with given goal, speed and
        acceleration

        Parameters
        ----------
        goal_poses: list[list[float]]
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

        # TODO rotational velocity, rotational acceleration and blending radius not implemented yet
        if rot_speed or rot_acc or blending_radius is not None:
            self._logger.info(
                "Rotational velocity and acceleration limits and blending radius not implemented yet"
            )

        if not isinstance(goal_poses, list):
            raise TypeError("goal_poses must be a list of joint positions")

        if speed is not None:
            self.kwargs["velocity_scaling_factor"] = speed / 100
        if acc is not None:
            self.kwargs["acceleration_scaling_factor"] = acc / 100

        try:
            waypoints = []
            for point in goal_poses:
                wpose = pose_2_geometry_msg_pose(Pose.from_list(point))
                waypoints.append(wpose)
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, 0.01, 0.0
            )
            self._logger.info("Move linear fraction is: " + str(fraction))
            if speed or acc is not None:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(), plan, **self.kwargs
                )
                self._logger.info(
                    "Retiming trajectory so velocity and acceleration scaling factors."
                )
            success = self.group.execute(plan, wait=True)
            return success
        except Exception as e:
            raise RuntimeError(
                "Failed to execute move_linear_via_points: " + str(e)
            )
            
########### creatign function for move_linear ############

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
        # TODO rot_speed and rot_acc
        if speed is not None:
            self.kwargs["velocity_scaling_factor"] = speed / 100
        if acc is not None:
            self.kwargs["acceleration_scaling_factor"] = acc / 100

        if not isinstance(goal_pose, list):
            raise TypeError("goal_pose must be a list of joint positions")

        waypoints = []
        wpose = pose_2_geometry_msg_pose(Pose.from_list(goal_pose))
        waypoints.append(wpose)

        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            self._logger.info("Move linear fraction is: " + str(fraction))
            if speed or acc is not None:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(), plan, **self.kwargs
                )
                self._logger.info(
                    "Retiming trajectory so velocity and acceleration scaling factors."
                )
            success = self.group.execute(plan, wait=True)
            return success
        except Exception as e:
            raise RuntimeError("Failed to execute move_linear: " + str(e))
        
### creating function for wait signal #############

    def wait(self, time_s: float) -> None:
        """Wait for given time in seconds.

        Parameters
        ----------
        time_s: float
            Waiting time in seconds
        Raises
        ------
        RuntimeError:
            If action failed
        """
        try:
            rospy.sleep(time_s)
        except Exception as e:
            raise RuntimeError("Failed to execute wait: " + str(e))
        
######### creating function for converting cartesian to joint ###############

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
            Joint position from IK solution.

        Raises
        -------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """

        if not isinstance(goal_pose_cartesian, list):
            raise TypeError("cartesian_pose must be of type list")

        refstate = reference_joint_states
        if refstate is None:
            try:
                refstate = rospy.wait_for_message(
                    "/joint_states", JointState, 1.0
                )
            except rospy.exceptions.ROSException as e:
                self._logger.error("Getting joint state failed! " + str(e))
                return []
        goal_pose = pose_2_geometry_msg_pose(
            Pose.from_list(goal_pose_cartesian)
        )
        req = GetPositionIKRequest()
        req.ik_request.group_name = "arm"
        req.ik_request.robot_state.joint_state.position = list(
            refstate.position[0:7]
        )
        req.ik_request.robot_state.joint_state.header.frame_id = "/world"
        req.ik_request.pose_stamped.pose = goal_pose
        req.ik_request.robot_state.joint_state.name = (
            self.group.get_active_joints()
        )

        try:
            res: GetPositionIK._response_class = self.ik_client(req)
        except rospy.exceptions.ROSException as ex:
            self._logger.error(ex)
            return []

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self._logger.error(f"IK Failed. MoveIrErrorCode: {res.error_code}")
            solution = []
        else:
            solution = res.solution.joint_state.position
        return solution
    

### creating function for setting the motion program  ########

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
        self.group.set_max_velocity_scaling_factor = speed_mj / 100
        self.group.set_max_acceleration_scaling_factor = acc_mj / 100
        self.kwargs["velocity_scaling_factor"] = speed_ml / 100
        self.kwargs["acceleration_scaling_factor"] = acc_ml / 100
        
########### creating function for getting the current joint states #########

    def get_current_joint_state(self) -> List[float]:
        """Get the current joint state as a list of floats

        Returns
        -------
        List[float]
            Current joint positions
        """

        joint_positions = self.group.get_current_joint_values()
        return joint_positions

########## creating the fucntion for getting the current cartesian tcp pose ########

    def get_current_cartesian_tcp_pose(self) -> List[float]:
        """Get the current tcp pose as a list of floats

        Returns
        -------
        List[float]
            Current tcp pose
        """

        wpose = self.group.get_current_pose().pose
        (r, p, y) = tfs.euler_from_quaternion(
            [
                wpose.orientation.x,
                wpose.orientation.y,
                wpose.orientation.z,
                wpose.orientation.w,
            ]
        )
        tcp_pose = (
            wpose.position.x,
            wpose.position.y,
            wpose.position.z,
            r,
            p,
            y,
        )
        return tcp_pose
    
### creating function for excecuting ############

    def execute(
        self,
        ids: List[int],
        execution_feasibilities: List[bool],
    ) -> None:
        """Execute a given plan.

        Parameters
        ----------
        ids : List[int]
            A list of id of the planned motion to be executed
        execution_feasibilities : List[bool]
            A List of flag indicate the execution feasibility, false will be jumped over

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Action failed
        """

        if not isinstance(ids, list):
            raise TypeError("given ids must be of type list")
        for id in ids:
            if not isinstance(id, int):
                raise TypeError("given id must be of type int")
            try:
                trajectory_msg = self.trajectory_dict.get(id)
                if trajectory_msg is None:
                    raise RuntimeError("Id not available")
                self._logger.debug(f"Executable trajectory with id: {ids[0]}")
                self.group.execute(trajectory_msg, wait=True)
            except Exception as e:
                raise RuntimeError("Failed to execute: " + str(e))
            
####### creating function for plan motion to cartesian ##########

    def plan_motion_joint_to_cartesian(
        self,
        goal_pose: List[float],
        reference_joint_states: Union[List[float], None] = None,
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
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

        if not (speed is None):
            self.group.set_max_velocity_scaling_factor(speed / 100)
        if not (acc is None):
            self.group.set_max_velocity_scaling_factor(acc / 100)
        if not (start_joint_states is None):
            self.group.set_start_state(start_joint_states)

        try:
            pose_goal = pose_2_geometry_msg_pose(Pose.from_list(goal_pose))
            self.group.set_pose_target(pose_goal)
            (
                success,
                trajectory_msg,
                planning_time,
                error_code,
            ) = self.group.plan()
            if success:
                if reusable:
                    traj_id = self._add_id(trajectory_msg)
                last_joint_state = trajectory_msg.joint_trajectory.points[
                    -1
                ].positions
                return ((success, success), traj_id, last_joint_state)
        except MoveItCommanderException:
            joint_goal = self.cartesian_2_joint(
                goal_pose, reference_joint_states
            )
            self.group.set_pose_target(
                joint_goal[0 : len(self.group.get_active_joints())], wait=True
            )
            (
                success,
                trajectory_msg,
                planning_time,
                error_code,
            ) = self.group.plan()
            if success:
                traj_id = self._add_id(trajectory_msg)
                last_joint_state = trajectory_msg.joint_trajectory.points[
                    -1
                ].positions
                return ((success, success), traj_id, last_joint_state)
        except Exception as e:
            raise RuntimeError(
                "Failed to execute plan_motion_joint_to_cartesian action: "
                + str(e)
            )
            
###### creating function for plan motion to joint ##########

    def plan_motion_joint_to_joint(
        self,
        goal_pose: List[float],
        start_joint_states: Union[List[float], None] = None,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        reusable: Optional[bool] = False,
    ) -> Tuple[Tuple[bool, bool], int, List[float]]:
        """Plan motion for move joint action with given goal, speed and
        acceleration

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

        if not (speed is None):
            self.group.set_max_velocity_scaling_factor(speed / 100)
        if not (acc is None):
            self.group.set_max_velocity_scaling_factor(acc / 100)
        if not (start_joint_states is None):
            self.group.set_start_state(start_joint_states)

        try:
            self.group.set_joint_value_target(goal_pose)
            (
                success,
                trajectory_msg,
                planning_time,
                error_code,
            ) = self.group.plan()
            if success:
                if reusable:
                    traj_id = self._add_id(trajectory_msg)
                last_joint_state = trajectory_msg.joint_trajectory.points[
                    -1
                ].positions
                return ((success, success), traj_id, last_joint_state)
        except Exception as e:
            raise RuntimeError(
                "Failed to execute plan_motion_joint_to_joint action: " + str(e)
            )
            
###### creating function for plan motion linear ##############

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
        acceleration

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
        Tuple[RobotTrajectory,float]
            Tuple consisting of the plan and fraction.

        Raises
        ------
        TypeError
            Wrong input type
        RuntimeError
            Planning failed

        """
        if speed is not None:
            self.kwargs["velocity_scaling_factor"] = speed / 100
        if acc is not None:
            self.kwargs["acceleration_scaling_factor"] = acc / 100

        if not isinstance(goal_pose, list):
            raise TypeError("goal_pose must be a list of joint positions")
        waypoints = []

        if not isinstance(goal_pose, list):
            raise TypeError("goal_pose must be a list")

        if not (start_cartesian_pose is None):
            if not isinstance(start_cartesian_pose, list):
                raise TypeError("start_cartesian_pose must be a list")
            wpose = pose_2_geometry_msg_pose(
                Pose.from_list(start_cartesian_pose)
            )
            waypoints.append(wpose)
        else:
            wpose = self.group.get_current_pose().pose
            waypoints.append(wpose)
        wpose = pose_2_geometry_msg_pose(Pose.from_list(goal_pose))
        waypoints.append(wpose)

        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            self._logger.info("Move linear fraction is: " + str(fraction))
            if speed or acc is not None:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(), plan, **self.kwargs
                )
                self._logger.info(
                    "Retiming trajectory so velocity and acceleration scaling factors."
                )
            if reusable:
                traj_id = self._add_id(plan)
            last_joint_state = plan.joint_trajectory.points[-1].positions
            return ((True, True), traj_id, last_joint_state)
        except Exception as e:
            raise RuntimeError(
                "Failed to execute plan_motion_linear action: " + str(e)
            )
            
######### creating function for plan motion for joint through points ##############

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
        trajectory, speed and acceleration

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
        # TODO use reusable
        if not (speed is None):
            self.group.set_max_velocity_scaling_factor(speed / 100)
        if not (acc is None):
            self.group.set_max_velocity_scaling_factor(acc / 100)
        if not (start_joint_states is None):
            self.group.set_start_state(start_joint_states)

        try:
            for goal_pose in trajectory:
                self.group.set_joint_value_target(goal_pose)
                (
                    success,
                    trajectory_msg,
                    planning_time,
                    error_code,
                ) = self.group.plan()
                if success:
                    if reusable:
                        traj_id = self._add_id(trajectory_msg)
                    last_joint_state = trajectory_msg.joint_trajectory.points[
                        -1
                    ].positions
                    return ((success, success), traj_id, last_joint_state)
        except Exception as e:
            raise RuntimeError(
                "Failed to execute plan_motion_joint_to_joint action: " + str(e)
            )


###### creating function for plan motion linear through points ###########

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
        # TODO rotational velocity, rotational acceleration and blending radius not implemented yet
        if rot_speed or rot_acc or blending_radius is not None:
            self._logger.info(
                "Rotational velocity and acceleration limits and blending radius not implemented yet"
            )

        if speed is not None:
            self.kwargs["velocity_scaling_factor"] = speed / 100
        if acc is not None:
            self.kwargs["acceleration_scaling_factor"] = acc / 100

        waypoints = []

        if not isinstance(goal_poses, list):
            raise TypeError("goal_pose must be a list")

        if not (start_cartesian_pose is None):
            if not isinstance(start_cartesian_pose, list):
                raise TypeError("start_cartesian_pose must be a list")
            wpose = pose_2_geometry_msg_pose(
                Pose.from_list(start_cartesian_pose)
            )
            waypoints.append(wpose)
        else:
            wpose = self.group.get_current_pose().pose
            waypoints.append(wpose)

        for goal_pose in goal_poses:
            wpose = pose_2_geometry_msg_pose(Pose.from_list(goal_pose))
            waypoints.append(wpose)

        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            self._logger.info("Move linear fraction is: " + str(fraction))
            if speed or acc is not None:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(), plan, **self.kwargs
                )
                self._logger.info(
                    "Retiming trajectory so velocity and acceleration scaling factors."
                )
            if reusable:
                traj_id = self._add_id(plan)
            last_joint_state = plan.joint_trajectory.points[-1].positions
            return ((True, True), traj_id, last_joint_state)
        except Exception as e:
            raise RuntimeError(
                "Failed to execute plan_motion_linear action: " + str(e)
            )
            
####### defining the function for fisniu

    def finish(self) -> None:
        # TODO
        self._logger.error("not implemented yet")
        
######## setting function for clearing the ids ###############

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
        if not isinstance(ids, list):
            raise TypeError("given ids must be of type list")
        try:
            for id in ids:
                del self.trajectory_dict[id]
                self._logger.info(
                    "Successfully deleted trajectory with id " + str(id)
                )
            return True
        except Exception as e:
            raise RuntimeError("Failed to delete ids: " + str(e))
        
####### setting function for adding the id #############

    def _add_id(
        self,
        trajectory_msg: RobotTrajectory,
    ) -> int:
        """
        Add trajectory with given ID to trajectory dictionary

        Parameters
        ----------
        trajectory : RobotTrajectory
            Joint trajectory

        Returns
        -------
        traj_id: int
            Tuple consisting of (plan success, plan feasibility), result plan ID
            that points to planned trajectory, last joint positions in the
            result trajectory.

        Raises
        ------
        RuntimeError
            Planning failed
        """
        try:
            self.traj_id += 1
            self.trajectory_dict[self.traj_id] = trajectory_msg
            return self.traj_id
        except Exception as e:
            raise RuntimeError("Failed to save trajectory: " + str(e))
