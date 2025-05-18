import threading # import threading module
from typing import List # importing List module
import time
import rclpy # importing rclpy
from rclpy.node import Node # importing Node 
from std_msgs.msg import Bool, String, Int32MultiArray # imported Bool, String, Int32MultiArray
from sensor_msgs.msg import JointState, JointTrajectory # imported JointState, JointTrajectory
from geometry_msgs.msg import Pose, PoseArray # imported Pose, PoseArray
import numpy as np # imported numpy module
from copy import deepcopy # imported copy module
from typing import List ,Optional # imported List, Optional
from std_msgs.msg import Bool, Int32, Float32MultiArray



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


### creating class ThreadSafCmdIDManager ###########

class ThreadSafeCmdIDManager:
    """Thread-safe wrapper around CmdIDManager."""

    def __init__(self, id_manager: CmdIDManager = None):
        self._lock = threading.Lock()
        self._mgr = id_manager or CmdIDManager()

### definig an function for updating the id ###############

    def update_id(self) -> int:
        """
        Atomically increment and return the new ID.
        """
        with self._lock:
            return self._mgr.update_id()



### Defining the class MairaKinematics #########

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
        id_manager: CmdIDManager = None,
        robot_handler: Robot = None,
    ):
        super().__init__("maira_kinematics") #intialise the node

        # Motion parameters
        self.speed_move_joint = speed_move_joint # setting the move joint
        self.acc_move_joint = acc_move_joint # setting up the acc move joint
        self.speed_move_linear = speed_move_linear # setting up the move linear
        self.acc_move_linear = acc_move_linear # setting up the acc for move linear
        self.require_elbow_up = require_elbow_up # setting up the reuire elbow up

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

        self._database_client = DatabaseClient() # setting up the Database client 

        # Publishers
        self.joint_publish = self.create_publisher(JointState, 'joint_states', 10)

        self.pub_mjc_res = self.create_publisher(Pose, 'move_joint_to_cartesian/result', 10)
        self.pub_mjj_res = self.create_publisher(Pose, 'move_joint_to_joint/result', 10)
        self.pub_ml_res = self.create_publisher(Pose, 'move_linear/result', 10)
        self.pub_mlp_res = self.create_publisher(Pose, 'move_linear_via_points/result', 10)
        self.pub_mjv_res = self.create_publisher(Pose, 'move_joint_via_points/result', 10)
        self.pub_ctj=self.create_publisher(JointState,"cartesian_to_joint_state",10)
        # self.pub_plan_mjc = self.create_publisher(String, 'plan_motion_joint_to_cartesian/result', 10)
        self.pub_plan_mjj = self.create_publisher(String, 'plan_motion_joint_to_joint/result', 10)
        self.pub_plan_ml = self.create_publisher(String, 'plan_motion_linear/result', 10)
        self.pub_plan_mlp = self.create_publisher(String, 'plan_motion_linear_via_points/result', 10)
        self.pub_plan_mjv = self.create_publisher(String, 'plan_motion_joint_via_points/result', 10)
        # publish the current joint state when requested
        self.pub_current_joint_state = self.create_publisher(JointState, 'current_joint_state', 10)

        # publish the current Cartesian TCP pose when requested
        self.pub_current_cartesian_pose = self.create_publisher(Pose, 'current_cartesian_pose', 10)

        # publish result of execute_if_successful(id)
        self.pub_execute_if_successful = self.create_publisher(Bool, 'execute_if_successful/result', 10)
        # publish the IK solution for a given goal pose
        self.pub_ik_solution = self.create_publisher(JointState, 'get_ik_solution/result', 10)
        # publish the elbow-up IK solution
        self.pub_elbow_up_ik_solution = self.create_publisher(JointState, 'get_elbow_up_ik_solution/result', 10)
        # acknowledge that set_motion_till_force has been applied
        self.pub_set_motion_till_force = self.create_publisher(Bool, 'set_motion_till_force/result', 10)
        # trigger to read out current joint state
        self.create_subscription(Bool,'get_current_joint_state',self._get_current_joint_state,10)
        # trigger to read out current Cartesian TCP pose
        self.create_subscription(Bool,'get_current_cartesian_pose',self._get_current_cartesian_pose,10)
        # request execution check for a single ID
        self.create_subscription(Int32,'execute_if_successful',self._execute_if_successful,10)

        # request a plain IK solution
        self.create_subscription(Pose,'get_ik_solution',self._get_ik_solution,10)
        # request an elbow-up IK solution
        self.create_subscription(Pose,'get_elbow_up_ik_solution',self._get_elbow_up_ik_solution,10)
        # configure motion-till-force (expects 3 floats in data[])
        self.create_subscription(Float32MultiArray,'set_motion_till_force',self.set_motion_till_force,10)

        # Subscribers 
        self.create_subscription(Pose,"move_unified_pose",self.unified_pose_callback,10)
        self.create_subscription(Bool, 'move_joint_to_cartesian', self.move_joint_to_cartesian, 10)
        self.create_subscription(JointState, 'move_joint_to_joint', self.move_joint_to_joint, 10)
        self.create_subscription(Bool, 'move_linear', self.move_linear, 10)
        self.create_subscription(PoseArray, 'move_linear_via_points', self.move_linear_via_points, 10)
        self.create_subscription(JointTrajectory, 'move_joint_via_points', self.move_joint_via_points, 10)
        self.create_subscription(Int32MultiArray, 'execute_ids', self.execute, 10)
        # self.create_subscription(Pose, 'plan_motion_joint_to_cartesian', self.plan_motion_joint_to_cartesian, 10)
        self.create_subscription(JointState, 'plan_motion_joint_to_joint', self.plan_motion_joint_to_joint, 10)
        self.create_subscription(Pose, 'plan_motion_linear', self.plan_motion_linear, 10)
        self.create_subscription(PoseArray, 'plan_motion_linear_via_points', self.plan_motion_linear_via_points, 10)
        self.create_subscription(JointTrajectory, 'plan_motion_joint_via_points', self.plan_motion_joint_via_points, 10)
        self.create_subscription(Pose,"cartesian_to_joint_state",self.cartesian_2_joint,10)


# defining the function for unified pose callback #########

    def unified_pose_callback(self, msg: Pose, goal_pose: List[float]):
        goal_pose = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]

        self.goal_pose_callback(msg)
        self.linear_pose_callback(msg)

        joint_property = {
            "target_joint": [goal_pose],
            "speed": self.speed_move_joint,
            "acceleration": self.acc_move_joint,
            "interpolator": 1,
            "enable_blending": True,
        }

        linear_property = {
            "target_pose": [self._get_current_cartesian_pose(), goal_pose],
            "speed": self.speed_move_linear,
            "acceleration": self.acc_move_linear,
            "blending": False,
            "blend_radius": 0.0,
        }

        plan_id = self._id_manager.update_id() if self._id_manager else 0

        if self._program:
            self._program.set_command(
                cmd.Joint,
                **joint_property,
                cmd_id=plan_id,
                current_joint_angles=self._get_current_joint_state(),
                reusable_id=0,
            )

            self._program.set_command(
                cmd.Linear,
                **linear_property,
                cmd_id=plan_id,
                current_joint_angles=self._get_current_joint_state(),
                reusable_id=0,
            )

            success = self._execute_if_successful(id=plan_id)

            if success:
                self.get_logger().info(f"Joint motion executed with plan ID {plan_id}")
            else:
                self.get_logger().error(f"Execution failed for plan ID {plan_id} with pose: {goal_pose}")

        if self._robot:
            joint_positions = self._robot.solve_ik(goal_pose)
            if joint_positions is not None:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self._robot.get_joint_names()
                joint_state_msg.position = joint_positions
                self.joint_publish.publish(joint_state_msg)
                self.get_logger().info(f"Published joint positions: {joint_positions}")

            else:

                joint_positions = self.cartesian_2_joint(goal_pose)
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self._robot.get_joint_names()
                js.position = joint_positions
                self.pub_ctj.publish(js)
                self.get_logger().info(f"cartesian_to_joint → {joint_positions}")

        else:
            self.get_logger().error("cartesian_to_joint failed")

    reference_joint_states = None
    speed = None
    acc = None


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
            (isinstance(joint_states, list)) # if joint states is in the list , checking if the lenght of the joint states are eual to the number of joint
            and len(joint_states) == self.num_joints
        ):
            raise TypeError(
                f"[ERROR] joint_states should be a list with length \
                    {self.num_joints}!"
            ) # raise the Type error


####### function for throw if pose invalid ##########

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

###### FUNCTIONS NEED TO WORK ###########################

# ###### function for speed to percent ########


#     def _speed_to_percent(self, speed_mps):
#         if speed_mps is None: # if the speed_mps is None 
#             speed_mps = self.speed_move_joint # setting the speeds_mps as speed_move_joint 
#         return 50 # return the speed as 50 

# ###### function for acceleration to percent ########


#     def _acc_to_percent(self, acc): 
#         if acc is None: # if the acceleration is None 
#             acc = self.acc_move_joint # setting the acceleration to move_to_joint
#         return 50 # returns acceleration as 50

#################################################################


#### function for gettting current joint state ############

    def _get_current_joint_state(self) -> List[float]:
        """Return current joint states.

        Return
        ------
        List[float]
            Joint states

        """
        return self._robot_state.getRobotStatus("jointAngles") # returning the robot state wiht Joint Angles 


######## function for getting current cartesian pose ###############

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
    
##### defining function for getting the current joint state ###########

    def get_current_joint_state(self) -> List[float]:
        """
        Retrieves the current joint positions of the robot.

        Returns
        -------
        List[float]
            A list of float values representing the current joint angles/positions.
            These are typically expressed in radians or degrees, depending on the robot's configuration.
        """
        # Delegate the call to the internal method that interfaces with the robot hardware or simulator.
        # This method abstracts the low-level details of reading joint states.
        return self._get_current_joint_state()


##### function for getting the current cartesian tcp pose ##########

    def get_current_cartesian_tcp_pose(self) -> List[float]:
        """Get the current cartesian tcp pose

        Returns
        -------
        List[float]
            Current Cartesian TCP pose as [x, y, z, r, p, y]
        """ 
        return self._get_current_cartesian_pose() # getting the current cartesian tcp poses
    
 ########### function for defining the finish ########
 
    def finish(self) -> None:
        """Delete planned motion buffer from NeuraPy. Only call this when
        program is going to be stopped."""
        MairaKinematics._ID = 3e4 # setting the MairaKinematics class to 30000.0
        self._program.finish() # stop the execution and disengage the robot controller

### defining function for converting the cartesian to joint ############

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
            raise TypeError("cartesian_pose must be of type list") # raise the type of error 

        self._throw_if_pose_invalid(goal_pose_cartesian) 

        reference_joint_states = (self._get_current_joint_state())  # getting the current joint states
        
        if reference_joint_states is None:# if the reference joint states is None 

            if not self.require_elbow_up: # if it is not elbow up
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
        solution = [] # creating an empty list of solution
        for i in range(100):
            np.random.seed(i) # ensure each iteration has a consistent random seed number
            dummy_array = (
                reference_joint_states + np.random.randn(self.num_joints) * 1e-2
            ) # multiplying the scale noise 
            try:
                solution = self._robot.ik_fk(
                    "ik",
                    target_pose=goal_pose_cartesian, 
                    current_joint=dummy_array,
                ) # getting the forward kinematics by setting the target_pose and current _joint
            except Exception as e:
                self._logger.debug(e) # raise an exception error 
                self._logger.debug(f"No IK solution found after {i} attempts")
                continue
            if not np.any(np.isnan(solution)):
                return solution
        else:
            raise ValueError(f"IK solution return none.") # raise an Value Error


 ###########function for getting the elbow up ik solution ###############
 
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
    # orientation in different axis     
        retry_delta_angle_states = [
            [0.0, 0.0],
            [0.0, np.pi],
            [np.pi, 0.0],
            [np.pi, np.pi],
        ] 
        for retry_delta_angle_state in retry_delta_angle_states:
            seed_joint = deepcopy(reference_joint_states)
            seed_joint[5] = (
                reference_joint_states[5] + retry_delta_angle_state[0] # seed joint [5] is updated with respect to reference jont states and and retry delta angle state
                if reference_joint_states[5] < 0.0
                else reference_joint_states[5] - retry_delta_angle_state[0]
            )
            seed_joint[6] = (
                reference_joint_states[6] + retry_delta_angle_state[1] # seed joint [6] is updated with respect to reference jont states and and retry delta angle state
                if reference_joint_states[6] < 0.0
                else reference_joint_states[6] - retry_delta_angle_state[1]
            )
            try:
                solution = self._get_ik_solution(
                    goal_pose_cartesian=goal_pose_cartesian,
                    reference_joint_states=seed_joint,
                ) # getting the ik solution
                if self._elbow_checker.is_up(solution):
                    self._logger.debug("Elbow up!")
                    return solution
            except:
                pass
        else:
            self._logger.debug("Elbow down!")
            raise ValueError("Could not find elbow up solution") # raise the Value Error 


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




#### TO DO MOVEMENT FOR JOINT MOTION #########################


##### defining function for move joint to cartesian


    def move_joint_to_cartesian(self, msg: Pose) -> None:
        res = self.move_joint_to_cartesian([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        self.pub_mjc_res.publish(Bool(data=res))
        self.get_logger().info(f"move_joint_to_cartesian → {res}")

##### defining function for move joint to joint ###########

    def move_joint_to_joint(self, msg: JointState) -> None:
        res = self.move_joint_to_joint(msg.position)
        self.pub_mjj_res.publish(Bool(data=res))
        self.get_logger().info(f"move_joint_to_joint → {res}")

##### defining function for move linear ##############

    def move_linear(self, msg: Pose) -> None:
        res = self.move_linear([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        self.pub_ml_res.publish(Bool(data=res))
        self.get_logger().info(f"move_linear → {res}")

##### defining function for move lienar via points ############

    def move_linear_via_points(self, msg: PoseArray) -> None:
        poses = [
            [p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in msg.poses
        ]
        res = self.move_linear_via_points(poses)
        self.pub_mlp_res.publish(Bool(data=res))
        self.get_logger().info(f"move_linear_via_points → {res}")


##### defining function for move joint via points ############

    def move_joint_via_points(self, msg: JointTrajectory) -> None:
        traj = [pt.positions for pt in msg.points]
        res = self.move_joint_via_points(traj)
        self.pub_mjv_res.publish(Bool(data=res))
        self.get_logger().info(f"move_joint_via_points → {res}")

##### defining function for execution ###################

    def execute(self, msg: Int32MultiArray) -> None:
        ids = list(msg.data)
        feas = [True] * len(ids)
        self.execute(ids, feas)
        self.get_logger().info(f"execute ids {ids}")

    # # Planning callbacks
    # def plan_motion_joint_to_cartesian(self, msg: Pose) -> None:
    #     ok, pid, last = self.plan_motion_joint_to_cartesian([
    #         msg.position.x, msg.position.y, msg.position.z,
    #         msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    #     ])
    #     payload = f"{ok},{pid},{last}"
    #     self.pub_plan_mjc.publish(String(data=payload))

##### defining function for motion joint to joint ###############

    def plan_motion_joint_to_joint(self, msg: JointState) -> None:
        ok, pid, last = self.plan_motion_joint_to_joint(msg.position)
        payload = f"{ok},{pid},{last}"
        self.pub_plan_mjj.publish(String(data=payload))

##### defining function for plan motion linear ##############

    def plan_motion_linear(self, msg: Pose) -> None:
        ok, pid, last = self.plan_motion_linear([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        payload = f"{ok},{pid},{last}"
        self.pub_plan_ml.publish(String(data=payload))

##### defining function for moiton linear via points ###############

    def plan_motion_linear_via_points(self, msg: PoseArray) -> None:
        poses = [
            [p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in msg.poses
        ]
        ok, pid, last = self.plan_motion_linear_via_points(poses)
        payload = f"{ok},{pid},{last}"
        self.pub_plan_mlp.publish(String(data=payload))

##### defining function for plan motion via points #################

    def plan_motion_joint_via_points(self, msg: JointTrajectory) -> None:
        traj = [pt.positions for pt in msg.points]
        ok, pid, last = self.plan_motion_joint_via_points(traj)
        payload = f"{ok},{pid},{last}"
        self.pub_plan_mjv.publish(String(data=payload))

##### defining main function ######
def main(args=None):
    rclpy.init(args=args)
    kinematics = MairaKinematics()
    rclpy.spin(kinematics)
    kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()